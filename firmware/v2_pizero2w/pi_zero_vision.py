#!/usr/bin/env python3
# =========================================================================
# Raspberry Pi Zero 2 W - Autonomous CapSorter Vision & Closed-Loop Planner (V2)
# Status: Pre-Validation Engineering Firmware (requires bench calibration)
# Hardware: Quad-Core Cortex-A53 @ 1 GHz | 512 MB RAM | 16 GB microSD
# Camera:   8MP Sony IMX219 160-deg CSI Camera (top-center, looking straight down)
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
MAX_BATCH_CAPS        = 100
PROTOCOL_MAGIC_BYTE   = 0x5A
PROTOCOL_SINGLE_MAGIC = 0x5B
CMD_BATCH_ACK         = 0xA1
CMD_BATCH_NACK        = 0xA2
CMD_BATCH_COMPLETE    = 0xA3
CMD_CAP_PROGRESS      = 0xA4
CMD_SCAN_WINDOW_OPEN  = 0xA5
CMD_AWAIT_NEXT        = 0xA6
CMD_QUERY_STATUS      = 0xB0
CMD_STATUS_READY      = 0xB1
CMD_STATUS_BUSY       = 0xB2
CMD_STATUS_FAULT      = 0xB3
CMD_REARM_HOME        = 0xB4
CMD_PARK_FOR_SCAN     = 0xB5
CMD_FINISH_SESSION    = 0xB6
CMD_HEARTBEAT_PING    = 0xF0
CMD_RECOVERY_HALT     = 0xEE
CMD_HARD_FAULT        = 0xEF

COLOR_RED, COLOR_YELLOW, COLOR_GREEN, COLOR_BLUE = 1, 2, 3, 4

# Continuous 300x300 mm Open Flat Plywood Workspace & Gripper Geometry (verify on bench)
BOARD_SIZE_MM             = 300.0
CAP_RADIUS_MM             = 15.0   # Nominal beverage cap radius (28-32 mm diameter)
MIN_CAP_SEPARATION_MM     = 22.0   # Distinguishes touching 28 mm caps while merging split blobs
SAFE_PICK_MIN_X_MM        = 18.0   # Keeps cap rim on plate & carriage clear of X-home switch
SAFE_PICK_MAX_X_MM        = 282.0
SAFE_PICK_MIN_Y_MM        = 18.0   # Keeps cap rim on plate & carriage clear of Y-home switch
SAFE_PICK_MAX_Y_MM        = 282.0

# Parallel Gripper Jaw Footprint (jaws open along X-axis; configurable to "Y")
GRIPPER_JAW_AXIS          = "X"
JAW_INNER_HALF_MM         = 18.0   # Open jaw inner gap = 36 mm (+/-18 mm from center)
JAW_OUTER_HALF_MM         = 26.0   # Outer jaw finger face reaches +/-26 mm along jaw axis
JAW_WIDTH_HALF_MM         = 10.0   # Finger pad half-width perpendicular to jaw axis
JAW_CLEARANCE_PAD_MM      = 1.5    # Extra safety margin around neighboring cap rims

# Overhead Camera Bridge Occlusion & Closed-Loop Retry Protection
BRIDGE_OCCLUSION_HALF_MM  = 35.0   # Masked horizontal Y strip when bridge sits at BIN_Y (0 disables)
MISSED_PICK_RADIUS_MM     = 14.0   # Same-color cap still within 14 mm after pick => missed attempt
MAX_PICK_RETRIES          = 3      # Quarantine stuck/ungraspable cap zone after 3 failed attempts
MAX_SESSION_CYCLES        = 200    # Hard safety bound on closed-loop pick cycles per session

# Supervision Timeouts (dynamic batch ceiling + per-cap stall timer)
HEARTBEAT_INTERVAL_S  = 0.25
READY_POLL_INTERVAL_S = 0.5
MAX_SINGLE_CAP_S      = 25.0  # Reset whenever Mega emits CMD_CAP_PROGRESS / CMD_AWAIT_NEXT
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

    camera_height_mm = (
        float(np.asarray(cal["camera_height_mm"]).item())
        if "camera_height_mm" in cal.files else 0.0
    )
    cap_height_mm = (
        float(np.asarray(cal["cap_height_mm"]).item())
        if "cap_height_mm" in cal.files else 13.0
    )
    if not (math.isfinite(camera_height_mm) and math.isfinite(cap_height_mm)):
        raise CalibrationError(f"Non-finite parallax heights in '{calib_file}'.")

    return {
        "K": K,
        "D": D,
        "H": H,
        "camera_type": cal_camera_type,
        "lens_model": lens_model,
        "image_size": cal_size,
        "camera_height_mm": camera_height_mm,
        "cap_height_mm": cap_height_mm,
    }


def _circle_intersects_rect(cx: float, cy: float, radius: float, x0: float, y0: float, x1: float, y1: float) -> bool:
    """Exact 2D circle vs axis-aligned rectangle intersection check."""
    nearest_x = min(max(cx, x0), x1)
    nearest_y = min(max(cy, y0), y1)
    dx = cx - nearest_x
    dy = cy - nearest_y
    return (dx * dx + dy * dy) < (radius * radius)


class PiZeroVisionController:
    def __init__(
        self,
        calib_file: str = None,
        serial_port: str = "/dev/serial0",
        camera_type: str = None,
        width: int = 640,
        height: int = 480,
        init_hardware: bool = True,
        bridge_occlusion_half_mm: float = BRIDGE_OCCLUSION_HALF_MM,
        jaw_axis: str = GRIPPER_JAW_AXIS,
        camera_height_mm: float = None,
        cap_height_mm: float = None,
    ):
        self.width = int(width)
        self.height = int(height)
        self.batch_id = 1
        self.bridge_occlusion_half_mm = max(0.0, float(bridge_occlusion_half_mm))
        self.jaw_axis = str(jaw_axis).upper()
        if self.jaw_axis not in ("X", "Y"):
            raise ValueError(f"Unsupported jaw_axis '{jaw_axis}'; expected 'X' or 'Y'")

        self.kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

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
        self.H_inv = np.linalg.inv(self.H)
        self.lens_model = cal["lens_model"]
        self.camera_height_mm = (
            float(camera_height_mm) if camera_height_mm is not None else cal["camera_height_mm"]
        )
        self.cap_height_mm = (
            float(cap_height_mm) if cap_height_mm is not None else cal["cap_height_mm"]
        )

        # Derive scale (px/mm) and camera optical center on the board for parallax & Watershed splitting
        self.px_per_mm = self._estimate_px_per_mm()
        self.expected_cap_radius_px = CAP_RADIUS_MM * self.px_per_mm
        self.cam_center_board_mm = self._estimate_optical_center_board_mm()

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

    def _estimate_px_per_mm(self) -> float:
        pts_mm = np.array([[[150.0, 150.0]], [[170.0, 150.0]], [[150.0, 170.0]]], dtype=np.float32)
        uv = cv2.perspectiveTransform(pts_mm, self.H_inv).reshape(3, 2)
        dx = float(np.linalg.norm(uv[1] - uv[0])) / 20.0
        dy = float(np.linalg.norm(uv[2] - uv[0])) / 20.0
        scale = 0.5 * (dx + dy)
        return scale if (math.isfinite(scale) and scale > 0.2) else 1.5

    def _estimate_optical_center_board_mm(self) -> tuple:
        cx_px, cy_px = float(self.K[0, 2]), float(self.K[1, 2])
        pt = np.array([[[cx_px, cy_px]]], dtype=np.float32)
        mapped = cv2.perspectiveTransform(pt, self.H)[0][0]
        bx, by = float(mapped[0]), float(mapped[1])
        if math.isfinite(bx) and math.isfinite(by) and -50.0 <= bx <= 350.0 and -50.0 <= by <= 350.0:
            return (bx, by)
        return (BOARD_SIZE_MM * 0.5, BOARD_SIZE_MM * 0.5)

    def apply_parallax_correction(self, x_mm: float, y_mm: float) -> tuple:
        """
        Compensates outward radial parallax caused by cap height (h_cap ~ 13 mm)
        relative to the Z=0 calibrated board plane when viewed from overhead H_cam.
        Disabled automatically if camera_height_mm <= cap_height_mm or <= 0.
        """
        if self.camera_height_mm > self.cap_height_mm > 0.0:
            scale = 1.0 - (self.cap_height_mm / self.camera_height_mm)
            cx, cy = self.cam_center_board_mm
            return (cx + (x_mm - cx) * scale, cy + (y_mm - cy) * scale)
        return (x_mm, y_mm)

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

    def _split_touching_cap_contour(self, cnt: np.ndarray, binary_mask: np.ndarray) -> list:
        """
        Separates merged contours of 2+ touching same-color caps (pairs, triangles,
        grids) using Euclidean distance transform + local peak extraction while
        preserving interior plywood gaps between touching caps.
        """
        x, y, w, h = cv2.boundingRect(cnt)
        pad = 2
        contour_roi = np.zeros((h + 2 * pad, w + 2 * pad), dtype=np.uint8)
        shifted = cnt - np.array([[[x - pad, y - pad]]], dtype=cnt.dtype)
        cv2.drawContours(contour_roi, [shifted], -1, 255, cv2.FILLED)

        # Intersect with actual binary_mask so interior gaps (e.g. center of 3 touching caps) remain 0
        roi = np.zeros_like(contour_roi)
        roi[pad:pad + h, pad:pad + w] = binary_mask[y:y + h, x:x + w]
        roi = cv2.bitwise_and(roi, contour_roi)

        dist = cv2.distanceTransform(roi, cv2.DIST_L2, 5)
        dist_smooth = cv2.GaussianBlur(dist, (5, 5), 1.0)

        min_peak_val = max(4.0, 0.45 * self.expected_cap_radius_px)
        k_size = max(5, int(round(self.expected_cap_radius_px * 0.55)) | 1)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_size, k_size))
        dilated = cv2.dilate(dist_smooth, kernel)

        peak_mask = ((dist_smooth >= dilated - 1e-4) & (dist >= min_peak_val)).astype(np.uint8)
        num_labels, _, _, centroids = cv2.connectedComponentsWithStats(peak_mask)

        peaks = []
        for lbl in range(1, num_labels):
            px, py = float(centroids[lbl][0]), float(centroids[lbl][1])
            ix = min(max(int(round(px)), 0), dist.shape[1] - 1)
            iy = min(max(int(round(py)), 0), dist.shape[0] - 1)
            r_val = float(dist[iy, ix])
            if r_val >= min_peak_val:
                peaks.append((px + x - pad, py + y - pad, r_val))

        # Suppress secondary sub-peaks closer than 1.25 * expected_cap_radius_px
        peaks.sort(key=lambda p: p[2], reverse=True)
        min_sep_px = max(8.0, 1.25 * self.expected_cap_radius_px)
        kept = []
        for u, v, r_val in peaks:
            if any(math.hypot(u - ku, v - kv) < min_sep_px for ku, kv, _ in kept):
                continue
            kept.append((u, v, math.pi * r_val * r_val))
        return kept

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

    def detect_caps(self, rectified_bgr: np.ndarray, occluded_y_mm: float = None) -> list:
        """
        Detects bottle caps at arbitrary continuous (X, Y) coordinates on the 300x300 mm
        board, splitting touching same-color clusters, compensating radial cap-height
        parallax, masking the gantry bridge band during bin-side scans, and tagging
        safe mechanical reachability (`in_safe_bounds`).
        """
        hsv = cv2.cvtColor(rectified_bgr, cv2.COLOR_BGR2HSV)
        candidates = []

        nominal_area = math.pi * (self.expected_cap_radius_px ** 2)
        min_cap_area = max(350.0, 0.32 * nominal_area)
        max_single_area = max(2400.0, 1.65 * nominal_area)

        for color_id, ranges in HSV_RANGES.items():
            mask = np.zeros((self.height, self.width), dtype=np.uint8)
            for lower, upper in ranges:
                mask |= cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))

            mask_opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open)
            mask_closed = cv2.morphologyEx(mask_opened, cv2.MORPH_CLOSE, self.kernel_close)
            contours, _ = cv2.findContours(mask_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < min_cap_area:
                    continue
                perim = cv2.arcLength(cnt, True)
                circularity = (4.0 * math.pi * area) / (perim * perim + 1e-5)

                extracted = []
                if area <= max_single_area and circularity >= 0.72:
                    M = cv2.moments(cnt)
                    if M["m00"] > 0:
                        extracted.append((M["m10"] / M["m00"], M["m01"] / M["m00"], area))
                else:
                    # Merged same-color cluster or distorted silhouette -> distance-transform split
                    extracted.extend(self._split_touching_cap_contour(cnt, mask_opened))

                for u, v, est_area in extracted:
                    pt = np.array([[[u, v]]], dtype=np.float32)
                    board_pt = cv2.perspectiveTransform(pt, self.H)[0][0]
                    x_mm, y_mm = self.apply_parallax_correction(float(board_pt[0]), float(board_pt[1]))

                    if not (0.0 <= x_mm <= BOARD_SIZE_MM and 0.0 <= y_mm <= BOARD_SIZE_MM):
                        continue

                    # Ignore detections inside horizontal gantry bridge occlusion strip at bin
                    if (
                        occluded_y_mm is not None
                        and self.bridge_occlusion_half_mm > 0.0
                        and abs(y_mm - occluded_y_mm) <= self.bridge_occlusion_half_mm
                    ):
                        continue

                    in_safe = (
                        SAFE_PICK_MIN_X_MM <= x_mm <= SAFE_PICK_MAX_X_MM
                        and SAFE_PICK_MIN_Y_MM <= y_mm <= SAFE_PICK_MAX_Y_MM
                    )
                    _, _, bin_id = BIN_COORDS[color_id]
                    candidates.append({
                        "capId": 0,
                        "color": color_id,
                        "pixelX": int(round(u)),
                        "pixelY": int(round(v)),
                        "boardX_mm": x_mm,
                        "boardY_mm": y_mm,
                        "targetBinId": bin_id,
                        "in_safe_bounds": in_safe,
                        "_area": est_area,
                    })
        return self.deduplicate_spatial_nms(candidates)

    def count_jaw_collisions(self, target_cap: dict, all_caps: list) -> tuple:
        """
        Evaluates how many sides (0, 1, or 2) of the descending open parallel gripper
        jaws collide with neighboring caps along `self.jaw_axis`.
        Returns (blocked_sides, min_neighbor_dist_mm).
        """
        tx, ty = target_cap["boardX_mm"], target_cap["boardY_mm"]
        r_check = CAP_RADIUS_MM + JAW_CLEARANCE_PAD_MM
        neg_blocked = False
        pos_blocked = False
        min_dist = 1e9

        for other in all_caps:
            if other is target_cap:
                continue
            ox, oy = other["boardX_mm"], other["boardY_mm"]
            d = math.hypot(ox - tx, oy - ty)
            if d < 1e-3:
                continue
            if d < min_dist:
                min_dist = d

            if self.jaw_axis == "X":
                if not neg_blocked and _circle_intersects_rect(
                    ox, oy, r_check,
                    tx - JAW_OUTER_HALF_MM, ty - JAW_WIDTH_HALF_MM,
                    tx - JAW_INNER_HALF_MM, ty + JAW_WIDTH_HALF_MM,
                ):
                    neg_blocked = True
                if not pos_blocked and _circle_intersects_rect(
                    ox, oy, r_check,
                    tx + JAW_INNER_HALF_MM, ty - JAW_WIDTH_HALF_MM,
                    tx + JAW_OUTER_HALF_MM, ty + JAW_WIDTH_HALF_MM,
                ):
                    pos_blocked = True
            else:
                if not neg_blocked and _circle_intersects_rect(
                    ox, oy, r_check,
                    tx - JAW_WIDTH_HALF_MM, ty - JAW_OUTER_HALF_MM,
                    tx + JAW_WIDTH_HALF_MM, ty - JAW_INNER_HALF_MM,
                ):
                    neg_blocked = True
                if not pos_blocked and _circle_intersects_rect(
                    ox, oy, r_check,
                    tx - JAW_WIDTH_HALF_MM, ty + JAW_INNER_HALF_MM,
                    tx + JAW_WIDTH_HALF_MM, ty + JAW_OUTER_HALF_MM,
                ):
                    pos_blocked = True

        return (int(neg_blocked) + int(pos_blocked), min_dist)

    def select_next_cap(
        self,
        detected_caps: list,
        cur_xy: tuple = (0.0, 0.0),
        retry_records: list = None,
        quarantined_zones: list = None,
    ):
        """
        Clearance-first / outermost-first pick selector:
          1. Excludes caps outside safe mechanical reach (`in_safe_bounds == False`)
             and quarantined unpickable spots, while STILL checking them as obstacles.
          2. Priority order: (retry_penalty, blocked_sides, travel_dist, -min_neighbor_dist).
             Picks completely clear caps first, peels clusters from outside in, and
             minimizes Cartesian travel from the current bin.
        """
        retry_records = retry_records or []
        quarantined_zones = quarantined_zones or []
        cur_x, cur_y = float(cur_xy[0]), float(cur_xy[1])

        scored = []
        for cap in detected_caps:
            if not cap.get("in_safe_bounds", True):
                continue
            cx, cy, col = cap["boardX_mm"], cap["boardY_mm"], cap["color"]
            if any(
                q["color"] == col and math.hypot(cx - q["x"], cy - q["y"]) <= MISSED_PICK_RADIUS_MM
                for q in quarantined_zones
            ):
                continue

            retry_penalty = 1 if any(
                r["color"] == col and r["count"] > 0
                and math.hypot(cx - r["x"], cy - r["y"]) <= MISSED_PICK_RADIUS_MM
                for r in retry_records
            ) else 0

            blocked_sides, min_neighbor = self.count_jaw_collisions(cap, detected_caps)
            travel = math.hypot(cx - cur_x, cy - cur_y)
            scored.append(((retry_penalty, blocked_sides, travel, -min_neighbor), blocked_sides, cap))

        if not scored:
            return None

        scored.sort(key=lambda item: item[0])
        _, blocked_sides, best = scored[0]
        chosen = dict(best)
        chosen["blocked_sides"] = blocked_sides
        return chosen

    def _update_retry_state(
        self,
        last_target: dict,
        detected_caps: list,
        retry_records: list,
        quarantined_zones: list,
        occluded_y_mm: float = None,
    ) -> bool:
        """
        Checks whether `last_target` is still sitting at its previous location on an
        unoccluded region of the fresh scan. Quarantines spots after MAX_PICK_RETRIES.
        Returns True if `last_target` was visibly evaluated (not occluded), False otherwise.
        """
        if last_target is None:
            return False
        tx, ty, col = last_target["boardX_mm"], last_target["boardY_mm"], last_target["color"]

        # Do not judge pick outcome while last_target sits inside the masked bridge strip
        if (
            occluded_y_mm is not None
            and self.bridge_occlusion_half_mm > 0.0
            and abs(ty - occluded_y_mm) <= self.bridge_occlusion_half_mm
        ):
            return False

        already_quarantined = any(
            q["color"] == col and math.hypot(q["x"] - tx, q["y"] - ty) <= MISSED_PICK_RADIUS_MM
            for q in quarantined_zones
        )
        if already_quarantined:
            return True

        still_present = any(
            c["color"] == col and math.hypot(c["boardX_mm"] - tx, c["boardY_mm"] - ty) <= MISSED_PICK_RADIUS_MM
            for c in detected_caps
        )

        rec = next(
            (r for r in retry_records
             if r["color"] == col and math.hypot(r["x"] - tx, r["y"] - ty) <= MISSED_PICK_RADIUS_MM),
            None,
        )

        if still_present:
            if rec is None:
                rec = {"x": tx, "y": ty, "color": col, "count": 1}
                retry_records.append(rec)
            else:
                rec["x"], rec["y"] = tx, ty
                rec["count"] += 1
            if rec["count"] >= MAX_PICK_RETRIES:
                quarantined_zones.append({"x": tx, "y": ty, "color": col, "attempts": rec["count"]})
                print(
                    f"[WARN] Cap (color={col}) at ({tx:.1f}, {ty:.1f}) mm still present after "
                    f"{rec['count']} attempts; quarantining spot for this session.",
                    file=sys.stderr,
                )
        elif rec is not None:
            retry_records.remove(rec)
        return True

    def optimize_path(self, caps: list) -> list:
        """Legacy greedy path optimizer (retained for offline tools & unit tests)."""
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

    def pack_single_target_packet(self, cap: dict) -> bytes:
        """
        Packs an 18-byte CRC16-verified closed-loop single-target packet:
          [0x5B][CapTarget: <BBHHffB (15B)][CRC16: <H (2B)]
        """
        payload = struct.pack(
            "<BBHHffB",
            int(cap.get("capId", 1)) & 0xFF,
            int(cap["color"]) & 0xFF,
            int(cap["pixelX"]) & 0xFFFF,
            int(cap["pixelY"]) & 0xFFFF,
            float(cap["boardX_mm"]),
            float(cap["boardY_mm"]),
            int(cap["targetBinId"]) & 0xFF,
        )
        crc = crc16_ccitt(payload)
        return bytes([PROTOCOL_SINGLE_MAGIC]) + payload + struct.pack("<H", crc)

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

    def _send_single_target_with_ack(self, cap: dict, sleep_fn=time.sleep) -> str:
        packet = self.pack_single_target_packet(cap)
        for _ in range(3):
            self.uart.reset_input_buffer()
            self.uart.write(packet)
            self.uart.flush()
            ack = self.uart.read(1)
            if ack:
                code = ack[0]
                if code == CMD_BATCH_ACK:
                    return "ACK"
                if code in (CMD_HARD_FAULT, CMD_RECOVERY_HALT, CMD_STATUS_FAULT):
                    return "FAULT"
            sleep_fn(0.05)
        return "NACK"

    def _await_opcode_with_heartbeat(
        self,
        target_opcode: int,
        timeout_s: float = MAX_SINGLE_CAP_S,
        clock=time.monotonic,
        sleep_fn=time.sleep,
    ) -> str:
        start_t = clock()
        while True:
            self.uart.write(bytes([CMD_HEARTBEAT_PING]))
            self.uart.flush()
            sleep_fn(HEARTBEAT_INTERVAL_S)

            while self.uart.in_waiting > 0:
                resp = self.uart.read(1)[0]
                if resp == target_opcode:
                    return "OK"
                if resp in (CMD_RECOVERY_HALT, CMD_HARD_FAULT, CMD_STATUS_FAULT):
                    return "FAULT"

            if clock() - start_t > timeout_s:
                return "TIMEOUT"

    def run_closed_loop_sort_session(
        self,
        capture_fn=None,
        clock=time.monotonic,
        sleep_fn=time.sleep,
        max_cycles: int = MAX_SESSION_CYCLES,
    ) -> dict:
        """
        Executes a full closed-loop sorting session:
          1. Captures Scan #0 at unobstructed (0, 0).
          2. Sends one clearance-ranked CapTarget at a time via CMD_PICK_SINGLE (0x5B).
          3. On Mega CMD_SCAN_WINDOW_OPEN (0xA5) at the color bin, captures & analyzes
             the fresh board IN PARALLEL with the bin drop, updating bumped coordinates
             and missed-pick retry/quarantine state.
          4. On CMD_AWAIT_NEXT (0xA6), immediately dispatches the next cap from the bin,
             or executes CMD_PARK_FOR_SCAN (0xB5) for an unobstructed clear-FOV check
             before terminating cleanly with CMD_FINISH_SESSION (0xB6).
        """
        if self.uart is None:
            return {"status": "NACK", "picked": 0, "scans": 0, "quarantined": [], "out_of_bounds_seen": 0}

        cap_source = capture_fn or self.capture_rectified_frame
        retry_records = []
        quarantined_zones = []
        scans = 0
        picked = 0
        out_of_bounds_seen = 0

        # Scan #0: Unobstructed full-board scan while parked at (0, 0)
        frame = cap_source()
        scans += 1
        caps = self.detect_caps(frame, occluded_y_mm=None)
        out_of_bounds_seen += sum(1 for c in caps if not c.get("in_safe_bounds", True))
        next_cap = self.select_next_cap(caps, (0.0, 0.0), retry_records, quarantined_zones)

        if next_cap is None:
            return {
                "status": "COMPLETE",
                "picked": 0,
                "scans": scans,
                "quarantined": quarantined_zones,
                "out_of_bounds_seen": out_of_bounds_seen,
            }

        for _ in range(max_cycles):
            ack_status = self._send_single_target_with_ack(next_cap, sleep_fn=sleep_fn)
            if ack_status != "ACK":
                return {
                    "status": ack_status,
                    "picked": picked,
                    "scans": scans,
                    "quarantined": quarantined_zones,
                    "out_of_bounds_seen": out_of_bounds_seen,
                }

            last_target = next_cap
            bin_x, bin_y, _ = BIN_COORDS[last_target["color"]]
            cycle_start = clock()
            scanned_at_bin = False
            retry_evaluated = False
            await_received = False
            candidate_next = None

            while not await_received:
                self.uart.write(bytes([CMD_HEARTBEAT_PING]))
                self.uart.flush()
                sleep_fn(HEARTBEAT_INTERVAL_S)

                while self.uart.in_waiting > 0:
                    resp = self.uart.read(1)[0]
                    if resp == CMD_SCAN_WINDOW_OPEN and not scanned_at_bin:
                        # Pipelined capture while Mega lowers/drops/retracts at color bin
                        fresh = cap_source()
                        scans += 1
                        scanned_at_bin = True
                        self.uart.write(bytes([CMD_HEARTBEAT_PING]))
                        self.uart.flush()

                        fresh_caps = self.detect_caps(fresh, occluded_y_mm=bin_y)
                        out_of_bounds_seen += sum(1 for c in fresh_caps if not c.get("in_safe_bounds", True))
                        retry_evaluated = self._update_retry_state(
                            last_target, fresh_caps, retry_records, quarantined_zones, occluded_y_mm=bin_y
                        )
                        candidate_next = self.select_next_cap(
                            fresh_caps, (bin_x, bin_y), retry_records, quarantined_zones
                        )
                    elif resp == CMD_CAP_PROGRESS:
                        cycle_start = clock()
                    elif resp == CMD_AWAIT_NEXT:
                        await_received = True
                        break
                    elif resp in (CMD_RECOVERY_HALT, CMD_HARD_FAULT, CMD_STATUS_FAULT):
                        return {
                            "status": "FAULT",
                            "picked": picked,
                            "scans": scans,
                            "quarantined": quarantined_zones,
                            "out_of_bounds_seen": out_of_bounds_seen,
                        }

                if not await_received and (clock() - cycle_start > MAX_SINGLE_CAP_S):
                    print("[TIMEOUT] Mega stopped responding during closed-loop cap cycle.", file=sys.stderr)
                    return {
                        "status": "TIMEOUT",
                        "picked": picked,
                        "scans": scans,
                        "quarantined": quarantined_zones,
                        "out_of_bounds_seen": out_of_bounds_seen,
                    }

            picked += 1

            # Fallback capture if Mega signaled AWAIT_NEXT without prior SCAN_WINDOW_OPEN
            if not scanned_at_bin:
                fresh = cap_source()
                scans += 1
                self.uart.write(bytes([CMD_HEARTBEAT_PING]))
                self.uart.flush()
                fresh_caps = self.detect_caps(fresh, occluded_y_mm=bin_y)
                out_of_bounds_seen += sum(1 for c in fresh_caps if not c.get("in_safe_bounds", True))
                retry_evaluated = self._update_retry_state(
                    last_target, fresh_caps, retry_records, quarantined_zones, occluded_y_mm=bin_y
                )
                candidate_next = self.select_next_cap(
                    fresh_caps, (bin_x, bin_y), retry_records, quarantined_zones
                )

            if candidate_next is not None:
                next_cap = candidate_next
                continue

            # 0 graspable caps visible outside bridge band -> clear-FOV verification park at (0,0)
            self.uart.write(bytes([CMD_PARK_FOR_SCAN]))
            self.uart.flush()
            park_res = self._await_opcode_with_heartbeat(
                CMD_AWAIT_NEXT, timeout_s=MAX_SINGLE_CAP_S, clock=clock, sleep_fn=sleep_fn
            )
            if park_res != "OK":
                return {
                    "status": park_res,
                    "picked": picked,
                    "scans": scans,
                    "quarantined": quarantined_zones,
                    "out_of_bounds_seen": out_of_bounds_seen,
                }

            verify_frame = cap_source()
            scans += 1
            self.uart.write(bytes([CMD_HEARTBEAT_PING]))
            self.uart.flush()
            verify_caps = self.detect_caps(verify_frame, occluded_y_mm=None)
            out_of_bounds_seen += sum(1 for c in verify_caps if not c.get("in_safe_bounds", True))
            if not retry_evaluated:
                self._update_retry_state(
                    last_target, verify_caps, retry_records, quarantined_zones, occluded_y_mm=None
                )
            revealed = self.select_next_cap(
                verify_caps, (0.0, 0.0), retry_records, quarantined_zones
            )
            if revealed is not None:
                next_cap = revealed
                continue

            # Confirmed empty (or only quarantined/out-of-bounds caps remain) -> finish cleanly
            self.uart.write(bytes([CMD_FINISH_SESSION]))
            self.uart.flush()
            done_res = self._await_opcode_with_heartbeat(
                CMD_BATCH_COMPLETE, timeout_s=MAX_SINGLE_CAP_S, clock=clock, sleep_fn=sleep_fn
            )
            return {
                "status": "COMPLETE" if done_res == "OK" else done_res,
                "picked": picked,
                "scans": scans,
                "quarantined": quarantined_zones,
                "out_of_bounds_seen": out_of_bounds_seen,
            }

        # Reached hard cycle limit -> park safely
        self.uart.write(bytes([CMD_FINISH_SESSION]))
        self.uart.flush()
        self._await_opcode_with_heartbeat(
            CMD_BATCH_COMPLETE, timeout_s=MAX_SINGLE_CAP_S, clock=clock, sleep_fn=sleep_fn
        )
        return {
            "status": "MAX_CYCLES",
            "picked": picked,
            "scans": scans,
            "quarantined": quarantined_zones,
            "out_of_bounds_seen": out_of_bounds_seen,
        }

    def transmit_batch_and_supervise(
        self,
        ordered_caps: list,
        clock=time.monotonic,
        sleep_fn=time.sleep,
    ) -> str:
        """
        Legacy open-loop batch transmitter (retained for offline tools & protocol tests).
        Transmits a CRC16-verified batch and maintains 250 ms heartbeat pings
        for the entire batch duration.

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
    parser = argparse.ArgumentParser(description="CapSorter V2 Raspberry Pi Zero 2 W Closed-Loop Vision Controller")
    parser.add_argument("--calib", help="Path to validated camera calibration .npz file")
    parser.add_argument("--camera-type", choices=("picamera2_imx219", "usb_v4l2"))
    parser.add_argument("--serial-port", default="/dev/serial0")
    parser.add_argument("--bridge-occlusion-mm", type=float, default=BRIDGE_OCCLUSION_HALF_MM)
    parser.add_argument("--jaw-axis", choices=("X", "Y"), default=GRIPPER_JAW_AXIS)
    args = parser.parse_args()

    try:
        controller = PiZeroVisionController(
            calib_file=args.calib,
            serial_port=args.serial_port,
            camera_type=args.camera_type,
            init_hardware=True,
            bridge_occlusion_half_mm=args.bridge_occlusion_mm,
            jaw_axis=args.jaw_axis,
        )
    except CalibrationError as exc:
        print(f"[FAIL-CLOSED CALIBRATION ERROR] {exc}", file=sys.stderr)
        sys.exit(2)

    print("[V2] Waiting for Arduino Mega CMD_STATUS_READY handshake...")
    while not controller.wait_for_mega_ready(timeout_s=5.0):
        print("[V2] Mega not READY (homing or fault latched); retrying status query...")
        time.sleep(1.0)
    print("[V2] Mega READY. Starting closed-loop bin-side re-imaging vision loop.")

    while True:
        if not controller.wait_for_mega_ready(timeout_s=2.0):
            print("[V2] Mega unavailable or faulted; waiting for operator re-arm & READY...")
            time.sleep(1.0)
            continue

        result = controller.run_closed_loop_sort_session()
        if result["status"] != "COMPLETE":
            print(
                f"[V2] Session ended with status={result['status']} "
                f"(picked={result['picked']}, scans={result['scans']}). "
                "Inhibiting new scans until Mega is READY.",
                file=sys.stderr,
            )
            time.sleep(1.0)
        elif result["picked"] > 0:
            print(
                f"[V2] Session COMPLETE: picked={result['picked']}, scans={result['scans']}, "
                f"quarantined={len(result['quarantined'])}."
            )
        time.sleep(0.5)


if __name__ == "__main__":
    main()
