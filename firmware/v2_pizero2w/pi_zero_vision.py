#!/usr/bin/env python3
# =========================================================================
# Raspberry Pi Zero 2 W - Autonomous CapSorter Vision & Plan Generator (V2)
# Hardware: Quad-Core Cortex-A53 @ 1 GHz | 512 MB RAM | 16 GB microSD
# Camera:   8MP Sony IMX219 160-deg CSI Camera (or USB Webcam fallback)
# Link:     UART0 (/dev/serial0 @ 115200 baud) -> 3.3V/5V Shifter -> Mega
# =========================================================================
import struct
import time
import math
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

# Protocol Constants (matches packet_protocol.h)
PROTOCOL_MAGIC_BYTE = 0x5A
CMD_BATCH_ACK       = 0xA1
CMD_BATCH_NACK      = 0xA2
CMD_BATCH_COMPLETE  = 0xA3
CMD_HEARTBEAT_PING  = 0xF0
CMD_RECOVERY_HALT   = 0xEE

COLOR_RED, COLOR_YELLOW, COLOR_GREEN, COLOR_BLUE = 1, 2, 3, 4

# Bin Coordinates (mm) outside 300x300mm sorting board
BIN_COORDS = {
    COLOR_RED:    (355.0, 55.0,  1),
    COLOR_YELLOW: (355.0, 130.0, 2),
    COLOR_GREEN:  (355.0, 205.0, 3),
    COLOR_BLUE:   (355.0, 280.0, 4),
}

# HSV Thresholds for Diffused LED Top Illumination
HSV_RANGES = {
    COLOR_RED:    [((0, 100, 70), (10, 255, 255)), ((168, 100, 70), (180, 255, 255))],
    COLOR_YELLOW: [((18, 110, 90), (35, 255, 255))],
    COLOR_GREEN:  [((38, 85, 65),  (85, 255, 255))],
    COLOR_BLUE:   [((98, 100, 70), (132, 255, 255))],
}

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc

class PiZeroVisionController:
    def __init__(self, calib_file="/home/pi/capsorter/calibration.npz",
                 serial_port="/dev/serial0", init_hardware=True):
        self.width, self.height = 640, 480
        self.batch_id = 1
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        # Load 160-deg Fisheye (K, D) and 3x3 Planar Homography (H) matrices
        try:
            cal = np.load(calib_file)
            self.K = cal["K"].astype(np.float64).reshape((3, 3))
            self.D = cal["D"].astype(np.float64).reshape((4, 1))
            self.H = cal["H"].astype(np.float64).reshape((3, 3))
        except Exception:
            # Default calibrated 640x480 matrices if calibration.npz not yet generated
            self.K = np.array([[245.8, 0.0, 320.0], [0.0, 245.8, 240.0], [0.0, 0.0, 1.0]], dtype=np.float64)
            self.D = np.array([[-0.042], [0.011], [-0.004], [0.0005]], dtype=np.float64)
            self.H = np.array([[0.6579, 0.0, -60.526], [0.0, 0.7282, -24.757], [0.0, 0.0, 1.0]], dtype=np.float64)

        # Precompute 160-deg fisheye undistortion lookup maps in RAM (~5 ms remap)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            self.K, self.D, np.eye(3, dtype=np.float64), self.K,
            (self.width, self.height), cv2.CV_16SC2
        )

        self.uart = None
        self.picam2 = None
        self.usb_cam = None

        if init_hardware:
            if HAS_PYSERIAL:
                self.uart = serial.Serial(serial_port, baudrate=115200, timeout=0.5)
            if HAS_PICAMERA2:
                self.picam2 = Picamera2()
                # Note: In Picamera2/DRM naming, "RGB888" outputs 24-bit [B, G, R]
                # byte order in memory, matching OpenCV's default BGR format directly.
                cfg = self.picam2.create_still_configuration(
                    main={"size": (self.width, self.height), "format": "RGB888"}
                )
                self.picam2.configure(cfg)
                self.picam2.start()
                time.sleep(0.5)
                # Lock auto-exposure and white balance under diffused LED light plate
                self.picam2.set_controls({"AeEnable": False, "AwbEnable": False})
            else:
                self.usb_cam = cv2.VideoCapture(0)
                self.usb_cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.usb_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

    def undistort_frame(self, raw_bgr: np.ndarray) -> np.ndarray:
        return cv2.remap(raw_bgr, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

    def capture_rectified_frame(self) -> np.ndarray:
        if self.picam2 is not None:
            raw = self.picam2.capture_array("main")
        elif self.usb_cam is not None:
            ok, raw = self.usb_cam.read()
            if not ok:
                raise RuntimeError("Camera frame acquisition failed")
        else:
            raise RuntimeError("No active camera hardware initialized")
        return self.undistort_frame(raw)

    def detect_caps(self, rectified_bgr: np.ndarray) -> list:
        hsv = cv2.cvtColor(rectified_bgr, cv2.COLOR_BGR2HSV)
        detected = []
        cap_id = 1

        for color_id, ranges in HSV_RANGES.items():
            mask = np.zeros((self.height, self.width), dtype=np.uint8)
            for lower, upper in ranges:
                mask |= cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if not (500.0 < area < 6000.0):
                    continue
                perim = cv2.arcLength(cnt, True)
                circularity = (4.0 * math.pi * area) / (perim * perim + 1e-5)
                if circularity < 0.75:
                    continue

                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue
                u = M["m10"] / M["m00"]
                v = M["m01"] / M["m00"]

                # Map rectified pixel (u, v) -> physical board (X_mm, Y_mm)
                pt = np.array([[[u, v]]], dtype=np.float32)
                board_pt = cv2.perspectiveTransform(pt, self.H)[0][0]
                x_mm, y_mm = float(board_pt[0]), float(board_pt[1])

                if 0.0 <= x_mm <= 300.0 and 0.0 <= y_mm <= 300.0:
                    _, _, bin_id = BIN_COORDS[color_id]
                    detected.append({
                        "capId": cap_id,
                        "color": color_id,
                        "pixelX": int(round(u)),
                        "pixelY": int(round(v)),
                        "boardX_mm": x_mm,
                        "boardY_mm": y_mm,
                        "targetBinId": bin_id
                    })
                    cap_id += 1
        return detected

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
        payload = bytearray()
        for c in ordered_caps:
            # Matches struct CapTarget: <BBHHffB (15 bytes packed)
            payload += struct.pack(
                "<BBHHffB",
                c["capId"], c["color"], c["pixelX"], c["pixelY"],
                c["boardX_mm"], c["boardY_mm"], c["targetBinId"]
            )
        crc = crc16_ccitt(bytes(payload))
        # Matches struct BatchHeader: <BHBH (6 bytes packed)
        header = struct.pack("<BHBH", PROTOCOL_MAGIC_BYTE, self.batch_id, len(ordered_caps), crc)
        self.batch_id = ((self.batch_id + 1) & 0xFFFF) or 1
        return header + bytes(payload)

    def transmit_batch_and_supervise(self, ordered_caps: list) -> bool:
        if not ordered_caps or self.uart is None:
            return False
        packet = self.pack_batch_packet(ordered_caps)
        self.uart.reset_input_buffer()
        self.uart.write(packet)
        self.uart.flush()

        # Wait for CMD_BATCH_ACK (0xA1) from Arduino Mega
        ack = self.uart.read(1)
        if not ack or ack[0] != CMD_BATCH_ACK:
            return False

        # Send 250ms heartbeat pings while Mega sorts batch & returns to (0,0)
        start_t = time.monotonic()
        while time.monotonic() - start_t < 120.0:
            self.uart.write(bytes([CMD_HEARTBEAT_PING]))
            self.uart.flush()
            time.sleep(0.25)
            while self.uart.in_waiting > 0:
                resp = self.uart.read(1)[0]
                if resp in (CMD_BATCH_COMPLETE, CMD_RECOVERY_HALT):
                    return resp == CMD_BATCH_COMPLETE
        return False

if __name__ == "__main__":
    controller = PiZeroVisionController()
    time.sleep(1.5)  # Allow Arduino Mega to complete startup homing
    while True:
        frame = controller.capture_rectified_frame()
        caps = controller.optimize_path(controller.detect_caps(frame))
        if caps:
            controller.transmit_batch_and_supervise(caps)
        time.sleep(0.5)
