#!/usr/bin/env python3
# =========================================================================
# Raspberry Pi Zero 2 W - Autonomous CapSorter Vision & Plan Generator (V2)
# Status: Pre-Validation Engineering Firmware (requires bench calibration)
# Hardware: Quad-Core Cortex-A53 @ 1 GHz | 512 MB RAM | 16 GB microSD
# Camera:   8MP Sony IMX219 160-deg CSI Camera (or separately calibrated USB Webcam)
# Link:     UART0 (/dev/serial0 @ 115200 baud) <-> 3.3V/5V Shifter <-> Mega Serial1
# =========================================================================
import argparse
import math
import os
import struct
import sys
import time
import cv2
import numpy as np

try:
    import serial
    HAS_PYSERIAL = True
except ImportError:
    HAS_PYSERIAL = False

try:
    from picamera2 import Picamera2
    HAS_PICAMERA2 = True
except ImportError:
    HAS_PICAMERA2 = False

# Protocol Constants (must match firmware/common/packet_protocol.h identically)
MAX_BATCH_CAPS      = 100
PROTOCOL_MAGIC_BYTE = 0x5A
CMD_BATCH_ACK       = 0xA1
CMD_BATCH_NACK      = 0xA2
CMD_BATCH_COMPLETE  = 0xA3
CMD_CAP_PROGRESS    = 0xA4
CMD_QUERY_STATUS    = 0xB0
CMD_STATUS_READY    = 0xB1
CMD_STATUS_BUSY     = 0xB2
CMD_STATUS_FAULT    = 0xB3
CMD_REARM_HOME      = 0xB4
CMD_HEARTBEAT_PING  = 0xF0
CMD_RECOVERY_HALT   = 0xEE
CMD_HARD_FAULT      = 0xEF

COLOR_RED, COLOR_YELLOW, COLOR_GREEN, COLOR_BLUE = 1, 2, 3, 4
MIN_CAP_SEPARATION_MM = 24.0  # Physical caps are 28-32 mm diameter; prevents duplicate picks

# Supervision Timeouts (dynamic batch ceiling + per-cap stall timer)
HEARTBEAT_INTERVAL_S  = 0.25
READY_POLL_INTERVAL_S = 0.5
MAX_SINGLE_CAP_S      = 25.0  # Reset whenever Mega emits CMD_CAP_PROGRESS
PER_CAP_BUDGET_S      = 18.0  # Worst-case cap cycle + margin
BATCH_OVERHEAD_S      = 30.0  # Parking & handshake margin

DEFAULT_CALIB_PATHS = {
    "picamera2_imx219": "/home/pi/capsorter/calibration_csi_imx219.npz",
    "usb_v4l2":         "/home/pi/capsorter/calibration_usb.npz",
}

# Bin Coordinates (mm) outside 300x300 mm sorting board
BIN_COORDS = {
    COLOR_RED:    (355.0, 55.0,  1),
    COLOR_YELLOW: (355.0, 130.0, 2),
    COLOR_GREEN:  (355.0, 205.0, 3),
    COLOR_BLUE:   (355.0, 280.0, 4),
}

# HSV Thresholds for Diffused LED Top Illumination (verify on bench under target lighting)
HSV_RANGES = {
    COLOR_RED:    [((0, 100, 70), (10, 255, 255)), ((168, 100, 70), (180, 255, 255))],
    COLOR_YELLOW: [((18, 110, 90), (35, 255, 255))],
    COLOR_GREEN:  [((38, 85, 65),  (85, 255, 255))],
    COLOR_BLUE:   [((98, 100, 70), (132, 255, 255))],
}


class CalibrationError(RuntimeError):
    """Raised when camera calibration is missing, malformed, unverified, or mismatched."""


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def load_and_validate_calibration(
    calib_file: str,
    expected_camera_type: str,
    expected_size: tuple,
) -> dict:
    """
    Fail-closed calibration loader. Refuses missing files, uncalibrated placeholders,
    mismatched camera types (e.g. CSI calibration on USB webcam), resolution mismatches,
    or malformed/non-finite matrices.
    """
    if not calib_file or not os.path.isfile(calib_file):
        raise CalibrationError(
            f"Calibration file not found: '{calib_file}'. "
            f"Run fisheye_calibrate.py for camera_type='{expected_camera_type}' before enabling motion."
        )

    try:
        cal = np.load(calib_file, allow_pickle=False)
    except Exception as exc:
        raise CalibrationError(f"Failed to read calibration archive '{calib_file}': {exc}") from exc

    required = ("K", "D", "H", "camera_type", "lens_model", "image_size", "calibrated")
    missing = [k for k in required if k not in cal.files]
    if missing:
        raise CalibrationError(
            f"Calibration file '{calib_file}' is missing required keys {missing}. "
            "Re-generate with fisheye_calibrate.py."
        )

    is_calibrated = bool(np.asarray(cal["calibrated"]).item())
    if not is_calibrated:
        raise CalibrationError(
            f"Calibration file '{calib_file}' is marked uncalibrated (calibrated=False). "
            "Motion inhibited until real checkerboard + fiducial calibration is completed."
        )

    cal_camera_type = str(np.asarray(cal["camera_type"]).item())
    if cal_camera_type != expected_camera_type:
        raise CalibrationError(
            f"Calibration camera_type mismatch: file has '{cal_camera_type}', "
            f"but active camera is '{expected_camera_type}'. Use a calibration file "
            "captured on this exact camera and lens."
        )

    lens_model = str(np.asarray(cal["lens_model"]).item())
    if lens_model not in ("fisheye", "pinhole"):
        raise CalibrationError(f"Unsupported lens_model '{lens_model}' in '{calib_file}'.")

    cal_size = tuple(int(v) for v in np.asarray(cal["image_size"]).reshape(-1))
    if cal_size != tuple(expected_size):
        raise CalibrationError(
            f"Calibration image_size {cal_size} does not match configured capture size {tuple(expected_size)}."
        )

    try:
        K = np.asarray(cal["K"], dtype=np.float64).reshape((3, 3))
        H = np.asarray(cal["H"], dtype=np.float64).reshape((3, 3))
        D = np.asarray(cal["D"], dtype=np.float64).reshape((-1, 1))
    except ValueError as exc:
        raise CalibrationError(f"Invalid matrix dimensions in '{calib_file}': {exc}") from exc

    if lens_model == "fisheye" and D.shape != (4, 1):
        raise CalibrationError(f"Fisheye distortion vector D must have shape (4, 1), got {D.shape}.")
    if lens_model == "pinhole" and D.shape[0] < 4:
        raise CalibrationError(f"Pinhole distortion vector D must have >=4 coefficients, got {D.shape}.")

    if not (np.all(np.isfinite(K)) and np.all(np.isfinite(D)) and np.all(np.isfinite(H))):
        raise CalibrationError(f"Calibration matrices in '{calib_file}' contain NaN or Inf values.")
    if abs(np.linalg.det(H)) < 1e-12:
        raise CalibrationError(f"Homography matrix H in '{calib_file}' is singular.")

    return {
        "K": K,
        "D": D,
        "H": H,
        "camera_type": cal_camera_type,
        "lens_model": lens_model,
        "image_size": cal_size,
    }


class PiZeroVisionController:
    def __init__(
        self,
        calib_file: str = None,
        serial_port: str = "/dev/serial0",
        camera_type: str = None,
        width: int = 640,
        height: int = 480,
        init_hardware: bool = True,
    ):
        self.width = int(width)
        self.height = int(height)
        self.batch_id = 1
        self.kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

        # Resolve active camera type explicitly before loading calibration
        if camera_type is None:
            self.camera_type = "picamera2_imx219" if HAS_PICAMERA2 else "usb_v4l2"
        else:
            if camera_type not in DEFAULT_CALIB_PATHS:
                raise ValueError(f"Unsupported camera_type '{camera_type}'")
            self.camera_type = camera_type

        resolved_calib = calib_file or DEFAULT_CALIB_PATHS[self.camera_type]

        # Fail-closed calibration check (NO silent fallback to hard-coded matrices)
        cal = load_and_validate_calibration(
            calib_file=resolved_calib,
            expected_camera_type=self.camera_type,
            expected_size=(self.width, self.height),
        )
        self.K = cal["K"]
        self.D = cal["D"]
        self.H = cal["H"]
        self.lens_model = cal["lens_model"]

        # Precompute undistortion lookup maps using the validated camera's lens model
        if self.lens_model == "fisheye":
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                self.K, self.D, np.eye(3, dtype=np.float64), self.K,
                (self.width, self.height), cv2.CV_16SC2,
            )
        else:
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                self.K, self.D, None, self.K,
                (self.width, self.height), cv2.CV_16SC2,
            )

        self.uart = None
        self.picam2 = None
        self.usb_cam = None

        if init_hardware:
            if HAS_PYSERIAL:
                self.uart = serial.Serial(serial_port, baudrate=115200, timeout=0.5)
            if self.camera_type == "picamera2_imx219":
                if not HAS_PICAMERA2:
                    raise RuntimeError("picamera2_imx219 requested, but Picamera2 is not installed.")
                self.picam2 = Picamera2()
                # raw={"size": (1640, 1232)} forces full-sensor 2x2 binning on IMX219 before
                # ISP downscaling to 640x480, preserving wide FOV.
                # In Picamera2/DRM naming, "RGB888" outputs 24-bit [B, G, R] matching OpenCV BGR.
                cfg = self.picam2.create_still_configuration(
                    main={"size": (self.width, self.height), "format": "RGB888"},
                    raw={"size": (1640, 1232)},
                )
                self.picam2.configure(cfg)
                self.picam2.start()
                time.sleep(0.8)  # Warm up sensor under diffused LED light plate
                self.picam2.set_controls({"AeEnable": False, "AwbEnable": False})
            else:
                self.usb_cam = cv2.VideoCapture(0)
                if not self.usb_cam.isOpened():
                    raise RuntimeError("Failed to open USB camera device 0.")
                self.usb_cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                self.usb_cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.usb_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

    def undistort_frame(self, raw_bgr: np.ndarray) -> np.ndarray:
        return cv2.remap(raw_bgr, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

    def capture_rectified_frame(self) -> np.ndarray:
        if self.picam2 is not None:
            raw = self.picam2.capture_array("main")
        elif self.usb_cam is not None:
            for _ in range(4):
                self.usb_cam.grab()
            ok, raw = self.usb_cam.read()
            if not ok:
                raise RuntimeError("USB camera frame acquisition failed")
        else:
            raise RuntimeError("No active camera hardware initialized")
        if raw.shape[1] != self.width or raw.shape[0] != self.height:
            raise CalibrationError(
                f"Captured frame size {(raw.shape[1], raw.shape[0])} does not match "
                f"calibrated resolution {(self.width, self.height)}."
            )
        return self.undistort_frame(raw)

    def deduplicate_spatial_nms(self, candidates: list) -> list:
        sorted_cands = sorted(candidates, key=lambda c: c.get("_area", 0.0), reverse=True)
        accepted = []
        for cand in sorted_cands:
            if any(
                math.hypot(cand["boardX_mm"] - a["boardX_mm"],
                           cand["boardY_mm"] - a["boardY_mm"]) < MIN_CAP_SEPARATION_MM
                for a in accepted
            ):
                continue
            item = {k: v for k, v in cand.items() if k != "_area"}
            item["capId"] = len(accepted) + 1
            accepted.append(item)
        return accepted[:MAX_BATCH_CAPS]

    def detect_caps(self, rectified_bgr: np.ndarray) -> list:
        hsv = cv2.cvtColor(rectified_bgr, cv2.COLOR_BGR2HSV)
        candidates = []

        for color_id, ranges in HSV_RANGES.items():
            mask = np.zeros((self.height, self.width), dtype=np.uint8)
            for lower, upper in ranges:
                mask |= cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if not (500.0 < area < 6000.0):
                    continue
                perim = cv2.arcLength(cnt, True)
                circularity = (4.0 * math.pi * area) / (perim * perim + 1e-5)
                if circularity < 0.74:
                    continue

                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue
                u = M["m10"] / M["m00"]
                v = M["m01"] / M["m00"]

                pt = np.array([[[u, v]]], dtype=np.float32)
                board_pt = cv2.perspectiveTransform(pt, self.H)[0][0]
                x_mm, y_mm = float(board_pt[0]), float(board_pt[1])

                if 0.0 <= x_mm <= 300.0 and 0.0 <= y_mm <= 300.0:
                    _, _, bin_id = BIN_COORDS[color_id]
                    candidates.append({
                        "capId": 0,
                        "color": color_id,
                        "pixelX": int(round(u)),
                        "pixelY": int(round(v)),
                        "boardX_mm": x_mm,
                        "boardY_mm": y_mm,
                        "targetBinId": bin_id,
                        "_area": area,
                    })
        return self.deduplicate_spatial_nms(candidates)

    def optimize_path(self, caps: list) -> list:
        ordered, remaining = [], [dict(c) for c in caps]
        cur_x, cur_y = 0.0, 0.0
        seq_id = 1
        while remaining:
            remaining.sort(key=lambda c: math.hypot(c["boardX_mm"] - cur_x, c["boardY_mm"] - cur_y))
            nxt = remaining.pop(0)
            nxt["capId"] = seq_id
            seq_id += 1
            ordered.append(nxt)
            cur_x, cur_y, _ = BIN_COORDS[nxt["color"]]
        return ordered

    def pack_batch_packet(self, ordered_caps: list) -> bytes:
        if not (1 <= len(ordered_caps) <= MAX_BATCH_CAPS):
            raise ValueError(f"Batch size must be in 1..{MAX_BATCH_CAPS}, got {len(ordered_caps)}")
        payload = bytearray()
        for c in ordered_caps:
            # Matches struct CapTarget: <BBHHffB (15 bytes packed)
            payload += struct.pack(
                "<BBHHffB",
                c["capId"], c["color"], c["pixelX"], c["pixelY"],
                c["boardX_mm"], c["boardY_mm"], c["targetBinId"],
            )
        crc = crc16_ccitt(bytes(payload))
        # Matches struct BatchHeader: <BHBH (6 bytes packed)
        header = struct.pack("<BHBH", PROTOCOL_MAGIC_BYTE, self.batch_id, len(ordered_caps), crc)
        self.batch_id = ((self.batch_id + 1) & 0xFFFF) or 1
        return header + bytes(payload)

    def write_paced_uart(self, packet: bytes, chunk_size: int = 32, sleep_fn=time.sleep):
        # Pace in 32-byte chunks with 2 ms pause so Mega's 64-byte RX ring buffer never overflows
        for offset in range(0, len(packet), chunk_size):
            self.uart.write(packet[offset:offset + chunk_size])
            self.uart.flush()
            if offset + chunk_size < len(packet):
                sleep_fn(0.002)

    def wait_for_mega_ready(
        self,
        timeout_s: float = None,
        poll_interval_s: float = READY_POLL_INTERVAL_S,
        clock=time.monotonic,
        sleep_fn=time.sleep,
    ) -> bool:
        """
        Explicit READY/BUSY/FAULT startup handshake. Never transmits motion data
        until Mega reports CMD_STATUS_READY after homing finishes.
        """
        if self.uart is None:
            return False

        start_t = clock()
        last_poll_t = -1e9
        while timeout_s is None or (clock() - start_t < timeout_s):
            now = clock()
            if now - last_poll_t >= poll_interval_s:
                self.uart.write(bytes([CMD_QUERY_STATUS]))
                self.uart.flush()
                last_poll_t = now

            while self.uart.in_waiting > 0:
                resp = self.uart.read(1)[0]
                if resp == CMD_STATUS_READY:
                    return True
                if resp in (CMD_STATUS_FAULT, CMD_HARD_FAULT, CMD_RECOVERY_HALT):
                    print(
                        f"[FAULT] Mega reported status 0x{resp:02X}; "
                        "motion inhibited until operator clears fault & re-arms.",
                        file=sys.stderr,
                    )
                    return False
                # CMD_STATUS_BUSY: keep polling until Mega finishes homing

            sleep_fn(0.02)
        return False

    def transmit_batch_and_supervise(
        self,
        ordered_caps: list,
        clock=time.monotonic,
        sleep_fn=time.sleep,
    ) -> str:
        """
        Transmits a CRC16-verified batch and maintains 250 ms heartbeat pings
        for the entire batch duration (bounded by per-cap progress and dynamic
        worst-case total time rather than a fixed 180 s cutoff).

        Returns one of: "COMPLETE", "NACK", "FAULT", "TIMEOUT".
        """
        if not ordered_caps or self.uart is None:
            return "NACK"
        packet = self.pack_batch_packet(ordered_caps)

        ack_received = False
        for _ in range(3):
            self.uart.reset_input_buffer()
            self.write_paced_uart(packet, sleep_fn=sleep_fn)
            ack = self.uart.read(1)
            if ack:
                code = ack[0]
                if code == CMD_BATCH_ACK:
                    ack_received = True
                    break
                if code in (CMD_HARD_FAULT, CMD_RECOVERY_HALT, CMD_STATUS_FAULT):
                    return "FAULT"
            sleep_fn(0.05)

        if not ack_received:
            return "NACK"

        start_t = clock()
        last_progress_t = start_t
        max_total_s = len(ordered_caps) * PER_CAP_BUDGET_S + BATCH_OVERHEAD_S

        while True:
            self.uart.write(bytes([CMD_HEARTBEAT_PING]))
            self.uart.flush()
            sleep_fn(HEARTBEAT_INTERVAL_S)

            while self.uart.in_waiting > 0:
                resp = self.uart.read(1)[0]
                if resp == CMD_CAP_PROGRESS:
                    last_progress_t = clock()
                elif resp == CMD_BATCH_COMPLETE:
                    return "COMPLETE"
                elif resp in (CMD_RECOVERY_HALT, CMD_HARD_FAULT, CMD_STATUS_FAULT):
                    return "FAULT"

            now = clock()
            if (now - last_progress_t > MAX_SINGLE_CAP_S) or (now - start_t > max_total_s):
                print(
                    "[TIMEOUT] Mega stopped reporting progress or exceeded dynamic batch ceiling.",
                    file=sys.stderr,
                )
                return "TIMEOUT"


def main():
    parser = argparse.ArgumentParser(description="CapSorter V2 Raspberry Pi Zero 2 W Vision Controller")
    parser.add_argument("--calib", help="Path to validated camera calibration .npz file")
    parser.add_argument("--camera-type", choices=("picamera2_imx219", "usb_v4l2"))
    parser.add_argument("--serial-port", default="/dev/serial0")
    args = parser.parse_args()

    try:
        controller = PiZeroVisionController(
            calib_file=args.calib,
            serial_port=args.serial_port,
            camera_type=args.camera_type,
            init_hardware=True,
        )
    except CalibrationError as exc:
        print(f"[FAIL-CLOSED CALIBRATION ERROR] {exc}", file=sys.stderr)
        sys.exit(2)

    print("[V2] Waiting for Arduino Mega CMD_STATUS_READY handshake...")
    while not controller.wait_for_mega_ready(timeout_s=5.0):
        print("[V2] Mega not READY (homing or fault latched); retrying status query...")
        time.sleep(1.0)
    print("[V2] Mega READY. Starting vision loop.")

    while True:
        if not controller.wait_for_mega_ready(timeout_s=2.0):
            print("[V2] Mega unavailable or faulted; waiting for operator re-arm & READY...")
            time.sleep(1.0)
            continue

        frame = controller.capture_rectified_frame()
        caps = controller.optimize_path(controller.detect_caps(frame))
        if caps:
            status = controller.transmit_batch_and_supervise(caps)
            if status != "COMPLETE":
                print(
                    f"[V2] Batch ended with status={status}. Inhibiting new scans until Mega is READY.",
                    file=sys.stderr,
                )
                time.sleep(1.0)
        time.sleep(0.5)


if __name__ == "__main__":
    main()
