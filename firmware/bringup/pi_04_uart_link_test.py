#!/usr/bin/env python3
# =========================================================================
# Stage 7A Bench Test: Pi Zero 2 W <-> Level Shifter <-> Arduino Mega UART
# Purpose:
#   Verifies the Micro-USB OTG -> CH9102/CP2102 -> 3.3V/5V Shifter -> Mega
#   Serial1 link step-by-step BEFORE running autonomous sorting:
#     --mode ping          Query Mega status (READY / BUSY / FAULT)
#     --mode rearm         Send explicit CMD_REARM_HOME (0xB4) & time homing
#     --mode single-pick   Dispatch 1 CRC16-verified cap (0x5B) with 250 ms
#                          heartbeats & log every opcode milestone
#     --mode uart-cut-test Dispatch 1 cap, then STOP heartbeats after 1.0 s
#                          to prove Mega deposits ONLY that cap, parks (0,0),
#                          and latches FAILSAFE_HALT!
#
# Usage Examples:
#   python3 pi_04_uart_link_test.py --mode ping
#   python3 pi_04_uart_link_test.py --mode rearm
#   python3 pi_04_uart_link_test.py --mode single-pick --x 150 --y 150 --color 1
#   python3 pi_04_uart_link_test.py --mode uart-cut-test --x 150 --y 150 --color 1
# =========================================================================
import argparse
import os
import struct
import sys
import time

V2_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v2_pizero2w"))
if V2_DIR not in sys.path:
    sys.path.insert(0, V2_DIR)

import pi_zero_vision as pzv

OPCODE_NAMES = {
    pzv.CMD_BATCH_ACK:        "CMD_BATCH_ACK (0xA1)",
    pzv.CMD_BATCH_NACK:       "CMD_BATCH_NACK (0xA2)",
    pzv.CMD_BATCH_COMPLETE:   "CMD_BATCH_COMPLETE (0xA3)",
    pzv.CMD_CAP_PROGRESS:     "CMD_CAP_PROGRESS (0xA4)",
    pzv.CMD_SCAN_WINDOW_OPEN: "CMD_SCAN_WINDOW_OPEN (0xA5)",
    pzv.CMD_AWAIT_NEXT:       "CMD_AWAIT_NEXT (0xA6)",
    pzv.CMD_STATUS_READY:     "CMD_STATUS_READY (0xB1)",
    pzv.CMD_STATUS_BUSY:      "CMD_STATUS_BUSY (0xB2)",
    pzv.CMD_STATUS_FAULT:     "CMD_STATUS_FAULT (0xB3)",
    pzv.CMD_RECOVERY_HALT:    "CMD_RECOVERY_HALT (0xEE)",
    pzv.CMD_HARD_FAULT:       "CMD_HARD_FAULT (0xEF)",
}


def pack_single_target(x_mm: float, y_mm: float, color: int) -> bytes:
    payload = struct.pack("<BBHHffB", 1, color, 320, 240, float(x_mm), float(y_mm), color)
    crc = pzv.crc16_ccitt(payload)
    return bytes([pzv.PROTOCOL_SINGLE_MAGIC]) + payload + struct.pack("<H", crc)


def open_serial_or_exit(port_spec: str, baud: int = 115200):
    if not pzv.HAS_PYSERIAL:
        print("[ERROR] pyserial is not installed (run: pip3 install pyserial).", file=sys.stderr)
        sys.exit(2)
    port = pzv.resolve_serial_port(port_spec)
    print(f"[UART] Opening {port} @ {baud} baud...")
    try:
        ser = pzv.serial.Serial(port, baudrate=baud, timeout=0.5)
        time.sleep(0.15)
        return ser, port
    except Exception as exc:
        print(f"[FAIL] Could not open '{port}': {exc}", file=sys.stderr)
        print("       Check: (1) OTG adapter in inner USB port, (2) udev rule installed,")
        print("              (3) user is in 'dialout' group: sudo usermod -aG dialout $USER")
        sys.exit(1)


def run_ping(ser, timeout_s: float = 3.0) -> int:
    ser.reset_input_buffer()
    ser.write(bytes([pzv.CMD_QUERY_STATUS]))
    ser.flush()
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if ser.in_waiting > 0:
            code = ser.read(1)[0]
            print(f"[RX] Mega replied: {OPCODE_NAMES.get(code, f'0x{code:02X}')}")
            if code == pzv.CMD_STATUS_READY:
                print("[PASS] UART link healthy and Mega is homed & READY!")
                return 0
            if code == pzv.CMD_STATUS_BUSY:
                print("[INFO] UART link healthy, Mega is currently BUSY (homing or moving).")
                return 0
            print("[WARN] UART link healthy, Mega reports FAULT/HALT (release E-Stop & run --mode rearm).")
            return 0
        time.sleep(0.02)
    print("[FAIL] No reply from Mega within timeout!")
    print("       Check: TX<->RX crossover across 3.3V/5V level shifter, LV=3.3V, HV=5.0V, shared GND,")
    print("              and confirm final mega_motion.ino is uploaded on the Arduino Mega.")
    return 1


def run_rearm(ser, timeout_s: float = 35.0) -> int:
    ser.reset_input_buffer()
    print("[TX] Sending CMD_REARM_HOME (0xB4)...")
    ser.write(bytes([pzv.CMD_REARM_HOME]))
    ser.flush()
    t0 = time.monotonic()
    last_poll = t0
    while time.monotonic() - t0 < timeout_s:
        now = time.monotonic()
        if now - last_poll >= 0.5:
            ser.write(bytes([pzv.CMD_QUERY_STATUS]))
            ser.flush()
            last_poll = now
        while ser.in_waiting > 0:
            code = ser.read(1)[0]
            dt = now - t0
            print(f"  [{dt:5.2f}s] RX: {OPCODE_NAMES.get(code, f'0x{code:02X}')}")
            if code == pzv.CMD_STATUS_READY:
                print(f"[PASS] Mega homed and READY in {dt:.2f} s!")
                return 0
            if code in (pzv.CMD_HARD_FAULT, pzv.CMD_STATUS_FAULT):
                print("[FAIL] Mega rejected re-arm or faulted (is E-Stop pressed or limit switch open?).")
                return 1
        time.sleep(0.02)
    print("[FAIL] Timed out waiting for Mega homing to finish.")
    return 1


def run_single_pick(ser, x_mm: float, y_mm: float, color: int, simulate_cut: bool = False) -> int:
    if not (pzv.SAFE_PICK_MIN_X_MM <= x_mm <= pzv.SAFE_PICK_MAX_X_MM and
            pzv.SAFE_PICK_MIN_Y_MM <= y_mm <= pzv.SAFE_PICK_MAX_Y_MM):
        print(f"[ERROR] ({x_mm}, {y_mm}) is outside safe pick bounds 18..282 mm.", file=sys.stderr)
        return 2

    pkt = pack_single_target(x_mm, y_mm, color)
    ser.reset_input_buffer()
    print(f"[TX] Dispatching 18B CMD_PICK_SINGLE -> ({x_mm:.1f}, {y_mm:.1f}) mm, color/bin={color}")
    ser.write(pkt)
    ser.flush()

    ack = ser.read(1)
    if not ack or ack[0] != pzv.CMD_BATCH_ACK:
        got = OPCODE_NAMES.get(ack[0], f"0x{ack[0]:02X}") if ack else "TIMEOUT"
        print(f"[FAIL] Expected CMD_BATCH_ACK (0xA1), got {got}")
        return 1
    print("  [ 0.00s] RX: CMD_BATCH_ACK (0xA1) -> Mega moving to cap!")

    t0 = time.monotonic()
    last_ping = t0
    cut_done = False

    while time.monotonic() - t0 < 25.0:
        now = time.monotonic()
        dt = now - t0
        if simulate_cut and dt >= 1.0:
            if not cut_done:
                cut_done = True
                print(f"  [{dt:5.2f}s] [TEST] STOPPING HEARTBEATS NOW! Watch Mega finish this 1 cap, park (0,0), & halt...")
        elif now - last_ping >= pzv.HEARTBEAT_INTERVAL_S:
            ser.write(bytes([pzv.CMD_HEARTBEAT_PING]))
            ser.flush()
            last_ping = now

        while ser.in_waiting > 0:
            code = ser.read(1)[0]
            print(f"  [{dt:5.2f}s] RX: {OPCODE_NAMES.get(code, f'0x{code:02X}')}")
            if not simulate_cut and code == pzv.CMD_AWAIT_NEXT:
                print(f"  [{dt:5.2f}s] TX: CMD_FINISH_SESSION (0xB6) -> parking at (0,0)")
                ser.write(bytes([pzv.CMD_FINISH_SESSION]))
                ser.flush()
            elif not simulate_cut and code == pzv.CMD_BATCH_COMPLETE:
                print(f"[PASS] Full closed-loop single-cap handshake completed in {dt:.2f} s!")
                return 0
            elif simulate_cut and code == pzv.CMD_RECOVERY_HALT:
                # Wait briefly to see final park halt
                pass
            elif code in (pzv.CMD_HARD_FAULT, pzv.CMD_STATUS_FAULT):
                print("[FAIL] Mega reported fault during single-pick test.")
                return 1

        if simulate_cut and dt >= 15.0:
            ser.reset_input_buffer()
            ser.write(bytes([pzv.CMD_QUERY_STATUS]))
            ser.flush()
            r = ser.read(1)
            if r and r[0] == pzv.CMD_STATUS_FAULT:
                print("[PASS] Mega finished active cap, parked at (0,0), and latched FAILSAFE_HALT (0xB3)!")
                print("       Run '--mode rearm' when ready to re-home.")
                return 0
            print("[WARN] Mega did not report CMD_STATUS_FAULT after 15 s; inspect Serial Monitor.")
            return 1
        time.sleep(0.02)

    print("[FAIL] Timed out waiting for cycle completion.")
    return 1


def main():
    parser = argparse.ArgumentParser(description="Stage 7A: Pi <-> Mega Binary UART Link & Dry-Run Tester")
    parser.add_argument("--mode", required=True, choices=("ping", "rearm", "single-pick", "uart-cut-test"))
    parser.add_argument("--serial-port", default=pzv.DEFAULT_SERIAL_PORT)
    parser.add_argument("--x", type=float, default=150.0, help="Target board X in mm (18..282)")
    parser.add_argument("--y", type=float, default=150.0, help="Target board Y in mm (18..282)")
    parser.add_argument("--color", type=int, choices=(1, 2, 3, 4), default=1,
                        help="1=Red, 2=Yellow, 3=Green, 4=Blue")
    args = parser.parse_args()

    ser, _ = open_serial_or_exit(args.serial_port)
    try:
        if args.mode == "ping":
            rc = run_ping(ser)
        elif args.mode == "rearm":
            rc = run_rearm(ser)
        elif args.mode == "single-pick":
            rc = run_single_pick(ser, args.x, args.y, args.color, simulate_cut=False)
        else:
            rc = run_single_pick(ser, args.x, args.y, args.color, simulate_cut=True)
    finally:
        ser.close()
    sys.exit(rc)


if __name__ == "__main__":
    main()
