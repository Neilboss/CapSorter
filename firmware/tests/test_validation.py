#!/usr/bin/env python3
"""
Automated verification suite for CapSorter V1 & V2 firmware, calibration,
protocol parity, fail-closed safety invariants, HTML code-vault sync,
and documentation consistency.

Run via:
    python -m unittest discover -s firmware/tests -v
"""
import html
import math
import os
import re
import struct
import sys
import tempfile
import unittest
from collections import deque

import cv2
import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
V2_DIR = os.path.join(REPO_ROOT, "firmware", "v2_pizero2w")
if V2_DIR not in sys.path:
    sys.path.insert(0, V2_DIR)

import fisheye_calibrate as fc
import pi_zero_vision as pzv


class FakeUART:
    """Deterministic in-memory UART mock for handshake & heartbeat supervision tests."""

    def __init__(self, script_fn=None):
        self.tx_log = []
        self.rx_queue = deque()
        self.script_fn = script_fn

    @property
    def in_waiting(self):
        return len(self.rx_queue)

    def write(self, data: bytes):
        raw = bytes(data)
        self.tx_log.append(raw)
        if self.script_fn is not None:
            self.script_fn(raw, self.rx_queue)
        return len(raw)

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.rx_queue.clear()

    def read(self, n: int = 1) -> bytes:
        out = bytearray()
        for _ in range(n):
            if not self.rx_queue:
                break
            out.append(self.rx_queue.popleft())
        return bytes(out)


def _render_synthetic_checkerboards(out_dir: str, lens_model: str, count: int = 10):
    """
    Renders sharp synthetic checkerboard images at multiple 3D poses through a known
    camera model (pinhole or fisheye) so real OpenCV calibration can be tested offline.
    """
    w, h = 640, 480
    cols, rows = 9, 6
    sq_px = 40
    # Build high-contrast planar board texture: (cols+1) x (rows+1) squares + white border
    pad = sq_px
    board_w = (cols + 1) * sq_px + 2 * pad
    board_h = (rows + 1) * sq_px + 2 * pad
    board_img = np.full((board_h, board_w, 3), 245, dtype=np.uint8)
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:
                y0 = pad + r * sq_px
                x0 = pad + c * sq_px
                board_img[y0:y0 + sq_px, x0:x0 + sq_px] = 15

    # Known ground-truth camera intrinsics & distortion
    if lens_model == "fisheye":
        K_true = np.array([[320.0, 0.0, 320.0], [0.0, 320.0, 240.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        D_true = np.array([[-0.035], [0.008], [-0.002], [0.0004]], dtype=np.float64)
    else:
        K_true = np.array([[560.0, 0.0, 320.0], [0.0, 560.0, 240.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        D_true = np.array([[-0.08], [0.02], [0.0005], [-0.0005], [0.0]], dtype=np.float64)

    # 3D corners of the full board texture in board mm coordinates
    sq_mm = 25.0
    mm_per_px = sq_mm / sq_px
    src_tex_pts = np.array([
        [0, 0],
        [board_w - 1, 0],
        [board_w - 1, board_h - 1],
        [0, board_h - 1],
    ], dtype=np.float32)

    obj_corners_3d = np.array([
        [(-pad) * mm_per_px, (-pad) * mm_per_px, 0.0],
        [(board_w - 1 - pad) * mm_per_px, (-pad) * mm_per_px, 0.0],
        [(board_w - 1 - pad) * mm_per_px, (board_h - 1 - pad) * mm_per_px, 0.0],
        [(-pad) * mm_per_px, (board_h - 1 - pad) * mm_per_px, 0.0],
    ], dtype=np.float64)
    # Center the board around (0, 0, 0)
    cx_mm = ((cols + 1) * sq_mm) / 2.0
    cy_mm = ((rows + 1) * sq_mm) / 2.0
    obj_corners_3d[:, 0] -= cx_mm
    obj_corners_3d[:, 1] -= cy_mm

    poses = [
        ((0.0, 0.0, 0.0), (0.0, 0.0, 420.0)),
        ((0.22, 0.10, 0.05), (-25.0, -15.0, 440.0)),
        ((-0.20, 0.14, -0.08), (25.0, -10.0, 430.0)),
        ((0.15, -0.22, 0.10), (-15.0, 20.0, 450.0)),
        ((-0.18, -0.16, -0.12), (20.0, 18.0, 410.0)),
        ((0.28, -0.06, 0.18), (0.0, -25.0, 460.0)),
        ((-0.25, 0.08, -0.16), (-30.0, 10.0, 435.0)),
        ((0.08, 0.25, 0.22), (30.0, -20.0, 445.0)),
        ((-0.10, -0.26, -0.20), (10.0, 25.0, 455.0)),
        ((0.20, 0.20, 0.30), (-20.0, -20.0, 470.0)),
    ][:count]

    # Precompute inverse distortion map (distorted pixel -> ideal undistorted pixel)
    grid_x, grid_y = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    pts_dist = np.stack([grid_x, grid_y], axis=-1).reshape(-1, 1, 2)
    if lens_model == "fisheye":
        pts_undist = cv2.fisheye.undistortPoints(pts_dist, K_true, D_true, P=K_true)
    else:
        pts_undist = cv2.undistortPoints(pts_dist, K_true, D_true, P=K_true)
    inv_map_x = pts_undist[:, 0, 0].reshape(h, w).astype(np.float32)
    inv_map_y = pts_undist[:, 0, 1].reshape(h, w).astype(np.float32)

    paths = []
    for idx, (rvec_t, tvec_t) in enumerate(poses):
        rvec = np.array(rvec_t, dtype=np.float64).reshape(3, 1)
        tvec = np.array(tvec_t, dtype=np.float64).reshape(3, 1)
        proj, _ = cv2.projectPoints(obj_corners_3d, rvec, tvec, K_true, np.zeros((5, 1)))
        dst_pts = proj.reshape(4, 2).astype(np.float32)
        M = cv2.getPerspectiveTransform(src_tex_pts, dst_pts)
        pinhole_view = cv2.warpPerspective(
            board_img, M, (w, h), borderMode=cv2.BORDER_CONSTANT, borderValue=(180, 180, 180)
        )
        # Apply ground-truth lens distortion by sampling pinhole_view at undistorted coords
        distorted = cv2.remap(
            pinhole_view, inv_map_x, inv_map_y,
            interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(180, 180, 180)
        )
        img_path = os.path.join(out_dir, f"cb_{lens_model}_{idx:02d}.png")
        cv2.imwrite(img_path, distorted)
        paths.append(img_path)

    return paths, K_true, D_true


def _read_text(path: str) -> str:
    with open(path, "r", encoding="utf-8") as fh:
        return fh.read()


class TestProtocolParity(unittest.TestCase):
    def test_opcodes_and_limits_match_header(self):
        hdr_path = os.path.join(REPO_ROOT, "firmware", "common", "packet_protocol.h")
        text = _read_text(hdr_path)
        defines = {
            m.group(1): int(m.group(2), 0)
            for m in re.finditer(
                r"#define\s+(MAX_BATCH_CAPS|PROTOCOL_MAGIC_BYTE|PROTOCOL_SINGLE_MAGIC|CMD_\w+)\s+(0x[0-9A-Fa-f]+|\d+)",
                text,
            )
        }
        expected_shared = [
            "MAX_BATCH_CAPS", "PROTOCOL_MAGIC_BYTE", "PROTOCOL_SINGLE_MAGIC",
            "CMD_BATCH_ACK", "CMD_BATCH_NACK", "CMD_BATCH_COMPLETE", "CMD_CAP_PROGRESS",
            "CMD_SCAN_WINDOW_OPEN", "CMD_AWAIT_NEXT",
            "CMD_QUERY_STATUS", "CMD_STATUS_READY", "CMD_STATUS_BUSY",
            "CMD_STATUS_FAULT", "CMD_REARM_HOME", "CMD_PARK_FOR_SCAN", "CMD_FINISH_SESSION",
            "CMD_HEARTBEAT_PING", "CMD_RECOVERY_HALT", "CMD_HARD_FAULT",
        ]
        for name in expected_shared:
            self.assertIn(name, defines, f"{name} missing from packet_protocol.h")
            self.assertEqual(getattr(pzv, name), defines[name], f"Mismatch for {name}")

    def test_crc16_ccitt_and_packet_layout(self):
        # Standard CRC16-CCITT-FALSE check value for "123456789" is 0x29B1
        self.assertEqual(pzv.crc16_ccitt(b"123456789"), 0x29B1)
        self.assertEqual(struct.calcsize("<BHBH"), 6)
        self.assertEqual(struct.calcsize("<BBHHffB"), 15)

        with tempfile.TemporaryDirectory() as td:
            cal_path = os.path.join(td, "cal.npz")
            H = fc.compute_homography_from_fiducials([[92, 34], [548, 34], [548, 446], [92, 446]])
            fc.save_validated_calibration(
                cal_path,
                np.array([[320.0, 0, 320.0], [0, 320.0, 240.0], [0, 0, 1.0]]),
                np.zeros((4, 1)), H, "picamera2_imx219", "fisheye", (640, 480), 0.1
            )
            ctrl = pzv.PiZeroVisionController(
                calib_file=cal_path, camera_type="picamera2_imx219", init_hardware=False
            )
            caps = [
                {"capId": 1, "color": 1, "pixelX": 120, "pixelY": 200, "boardX_mm": 45.5, "boardY_mm": 120.25, "targetBinId": 1},
                {"capId": 2, "color": 4, "pixelX": 420, "pixelY": 310, "boardX_mm": 215.0, "boardY_mm": 240.75, "targetBinId": 4},
            ]
            pkt = ctrl.pack_batch_packet(caps)
            self.assertEqual(len(pkt), 6 + 2 * 15)
            magic, batch_id, total_caps, crc = struct.unpack("<BHBH", pkt[:6])
            self.assertEqual(magic, pzv.PROTOCOL_MAGIC_BYTE)
            self.assertEqual(batch_id, 1)
            self.assertEqual(total_caps, 2)
            self.assertEqual(crc, pzv.crc16_ccitt(pkt[6:]))
            with self.assertRaises(ValueError):
                ctrl.pack_batch_packet([])
            with self.assertRaises(ValueError):
                ctrl.pack_batch_packet(caps * 51)  # 102 > 100

            # Verify 18-byte closed-loop single-target packet layout & CRC16
            single_pkt = ctrl.pack_single_target_packet(caps[0])
            self.assertEqual(len(single_pkt), 18)
            self.assertEqual(single_pkt[0], pzv.PROTOCOL_SINGLE_MAGIC)
            crc_rx = struct.unpack("<H", single_pkt[16:18])[0]
            self.assertEqual(crc_rx, pzv.crc16_ccitt(single_pkt[1:16]))


class TestV2CalibrationAndFailClosed(unittest.TestCase):
    def test_shipped_placeholder_fails_closed(self):
        placeholder = os.path.join(V2_DIR, "calibration.npz")
        with self.assertRaises(pzv.CalibrationError):
            pzv.PiZeroVisionController(
                calib_file=placeholder, camera_type="picamera2_imx219", init_hardware=False
            )

    def test_missing_and_mismatched_calibrations_fail_closed(self):
        with tempfile.TemporaryDirectory() as td:
            # 1. Missing file
            with self.assertRaises(pzv.CalibrationError):
                pzv.PiZeroVisionController(
                    calib_file=os.path.join(td, "does_not_exist.npz"),
                    camera_type="usb_v4l2", init_hardware=False,
                )

            # 2. Legacy archive missing metadata keys
            legacy_path = os.path.join(td, "legacy.npz")
            np.savez(legacy_path, K=np.eye(3), D=np.zeros((4, 1)), H=np.eye(3))
            with self.assertRaises(pzv.CalibrationError):
                pzv.PiZeroVisionController(
                    calib_file=legacy_path, camera_type="picamera2_imx219", init_hardware=False
                )

            # 3. Valid CSI fisheye file rejected when USB camera mode is active!
            csi_path = os.path.join(td, "csi.npz")
            H = fc.compute_homography_from_fiducials([[92, 34], [548, 34], [548, 446], [92, 446]])
            fc.save_validated_calibration(
                csi_path,
                np.array([[320.0, 0, 320.0], [0, 320.0, 240.0], [0, 0, 1.0]]),
                np.zeros((4, 1)), H, "picamera2_imx219", "fisheye", (640, 480), 0.15
            )
            with self.assertRaises(pzv.CalibrationError):
                pzv.PiZeroVisionController(
                    calib_file=csi_path, camera_type="usb_v4l2", init_hardware=False
                )

            # 4. Resolution mismatch rejected
            with self.assertRaises(pzv.CalibrationError):
                pzv.PiZeroVisionController(
                    calib_file=csi_path, camera_type="picamera2_imx219",
                    width=1280, height=720, init_hardware=False
                )

    def test_real_checkerboard_calibration_both_models_and_detection(self):
        corners_px = [[92.0, 34.0], [548.0, 34.0], [548.0, 446.0], [92.0, 446.0]]
        for camera_type, lens_model in [
            ("picamera2_imx219", "fisheye"),
            ("usb_v4l2", "pinhole"),
        ]:
            with tempfile.TemporaryDirectory() as td:
                img_paths, _, _ = _render_synthetic_checkerboards(td, lens_model=lens_model, count=10)
                K, D, img_size, rms = fc.calibrate_intrinsics_from_checkerboards(
                    img_paths, pattern_size=(9, 6), square_size_mm=25.0,
                    lens_model=lens_model, min_images=8, max_reproj_err=1.0
                )
                self.assertEqual(img_size, (640, 480))
                self.assertLess(rms, 0.8)

                H = fc.compute_homography_from_fiducials(corners_px)
                cal_path = os.path.join(td, f"cal_{lens_model}.npz")
                fc.save_validated_calibration(
                    cal_path, K, D, H, camera_type, lens_model, img_size, rms
                )

                ctrl = pzv.PiZeroVisionController(
                    calib_file=cal_path, camera_type=camera_type, init_hardware=False
                )
                self.assertEqual(ctrl.lens_model, lens_model)

                # Draw 4 synthetic colored caps on a rectified 640x480 canvas
                canvas = np.full((480, 640, 3), 230, dtype=np.uint8)
                H_inv = np.linalg.inv(H)
                test_caps = [
                    (45.0, 45.0, (20, 20, 230), pzv.COLOR_RED),
                    (255.0, 45.0, (15, 225, 235), pzv.COLOR_YELLOW),
                    (45.0, 255.0, (30, 190, 30), pzv.COLOR_GREEN),
                    (255.0, 255.0, (230, 60, 20), pzv.COLOR_BLUE),
                ]
                for x_mm, y_mm, bgr, _ in test_caps:
                    uv = cv2.perspectiveTransform(
                        np.array([[[x_mm, y_mm]]], dtype=np.float32), H_inv
                    )[0][0]
                    cv2.circle(canvas, (int(round(uv[0])), int(round(uv[1]))), 22, bgr, -1)

                detected = ctrl.detect_caps(canvas)
                self.assertEqual(len(detected), 4, f"Expected 4 caps for {lens_model}, got {detected}")
                by_color = {c["color"]: c for c in detected}
                for x_mm, y_mm, _, col in test_caps:
                    self.assertIn(col, by_color)
                    self.assertAlmostEqual(by_color[col]["boardX_mm"], x_mm, delta=2.0)
                    self.assertAlmostEqual(by_color[col]["boardY_mm"], y_mm, delta=2.0)


class TestV2HandshakeAndSupervision(unittest.TestCase):
    def _make_controller(self, td):
        cal_path = os.path.join(td, "cal.npz")
        H = fc.compute_homography_from_fiducials([[92, 34], [548, 34], [548, 446], [92, 446]])
        fc.save_validated_calibration(
            cal_path,
            np.array([[320.0, 0, 320.0], [0, 320.0, 240.0], [0, 0, 1.0]]),
            np.zeros((4, 1)), H, "picamera2_imx219", "fisheye", (640, 480), 0.1
        )
        return pzv.PiZeroVisionController(
            calib_file=cal_path, camera_type="picamera2_imx219", init_hardware=False
        )

    def test_wait_for_mega_ready_waits_through_busy_and_inhibits_on_fault(self):
        with tempfile.TemporaryDirectory() as td:
            ctrl = self._make_controller(td)
            now = [0.0]

            def clock():
                return now[0]

            def sleep_fn(dt):
                now[0] += dt

            queries = [0]

            def busy_then_ready(raw, rx):
                if raw == bytes([pzv.CMD_QUERY_STATUS]):
                    queries[0] += 1
                    rx.append(pzv.CMD_STATUS_BUSY if queries[0] < 4 else pzv.CMD_STATUS_READY)

            ctrl.uart = FakeUART(busy_then_ready)
            self.assertTrue(ctrl.wait_for_mega_ready(timeout_s=10.0, clock=clock, sleep_fn=sleep_fn))
            self.assertEqual(queries[0], 4)

            def fault_reply(raw, rx):
                if raw == bytes([pzv.CMD_QUERY_STATUS]):
                    rx.append(pzv.CMD_STATUS_FAULT)

            ctrl.uart = FakeUART(fault_reply)
            self.assertFalse(ctrl.wait_for_mega_ready(timeout_s=5.0, clock=clock, sleep_fn=sleep_fn))

    def test_100_cap_batch_exceeding_180s_completes_with_progress_heartbeats(self):
        with tempfile.TemporaryDirectory() as td:
            ctrl = self._make_controller(td)
            now = [0.0]
            caps_done = [0]
            pings_sent = [0]

            def clock():
                return now[0]

            def sleep_fn(dt):
                now[0] += dt

            # Simulate 100 caps taking 9.0 seconds each (900 s total >> old 180 s cutoff!)
            next_cap_finish = [9.0]
            acked = [False]

            def mega_sim(raw, rx):
                if not acked[0] and raw[0] == pzv.PROTOCOL_MAGIC_BYTE:
                    acked[0] = True
                    rx.append(pzv.CMD_BATCH_ACK)
                elif raw == bytes([pzv.CMD_HEARTBEAT_PING]):
                    pings_sent[0] += 1
                    if now[0] >= next_cap_finish[0] and caps_done[0] < 100:
                        caps_done[0] += 1
                        next_cap_finish[0] += 9.0
                        rx.append(pzv.CMD_CAP_PROGRESS)
                        if caps_done[0] == 100:
                            rx.append(pzv.CMD_BATCH_COMPLETE)

            ctrl.uart = FakeUART(mega_sim)
            caps = [
                {"capId": i + 1, "color": (i % 4) + 1, "pixelX": 200, "pixelY": 200,
                 "boardX_mm": 100.0, "boardY_mm": 100.0, "targetBinId": (i % 4) + 1}
                for i in range(100)
            ]
            status = ctrl.transmit_batch_and_supervise(caps, clock=clock, sleep_fn=sleep_fn)
            self.assertEqual(status, "COMPLETE")
            self.assertGreater(now[0], 850.0, "Simulated 100-cap run should exceed 180 s cutoff")
            self.assertGreater(pings_sent[0], 3000)

    def test_stalled_mega_triggers_timeout_and_fault_halts_immediately(self):
        with tempfile.TemporaryDirectory() as td:
            ctrl = self._make_controller(td)
            now = [0.0]
            acked = [False]

            def stall_sim(raw, rx):
                if not acked[0] and raw[0] == pzv.PROTOCOL_MAGIC_BYTE:
                    acked[0] = True
                    rx.append(pzv.CMD_BATCH_ACK)

            ctrl.uart = FakeUART(stall_sim)
            caps = [{"capId": 1, "color": 1, "pixelX": 100, "pixelY": 100, "boardX_mm": 50.0, "boardY_mm": 50.0, "targetBinId": 1}]
            status = ctrl.transmit_batch_and_supervise(
                caps, clock=lambda: now[0], sleep_fn=lambda dt: now.__setitem__(0, now[0] + dt)
            )
            self.assertEqual(status, "TIMEOUT")
            self.assertGreaterEqual(now[0], pzv.MAX_SINGLE_CAP_S)

            acked2 = [False]

            def hard_fault_sim(raw, rx):
                if not acked2[0] and raw[0] == pzv.PROTOCOL_MAGIC_BYTE:
                    acked2[0] = True
                    rx.append(pzv.CMD_BATCH_ACK)
                elif raw == bytes([pzv.CMD_HEARTBEAT_PING]):
                    rx.append(pzv.CMD_HARD_FAULT)

            ctrl.uart = FakeUART(hard_fault_sim)
            status = ctrl.transmit_batch_and_supervise(
                caps, clock=lambda: now[0], sleep_fn=lambda dt: now.__setitem__(0, now[0] + dt)
            )
            self.assertEqual(status, "FAULT")


class TestV1AlgorithmsAndFirmwareInvariants(unittest.TestCase):
    def test_v1_nms_triple_overlap_keeps_largest_area(self):
        # Faithful mirror of updated teensy_vision.cpp deduplicateTargetsNMS
        caps = [
            {"id": "A", "x": 100.0, "y": 100.0},
            {"id": "B", "x": 105.0, "y": 100.0},
            {"id": "C", "x": 102.0, "y": 103.0},
            {"id": "D", "x": 200.0, "y": 200.0},
        ]
        areas = [500, 1200, 800, 600]

        # Insertion sort descending by area (identical to C++ implementation)
        for i in range(1, len(caps)):
            kc, ka = caps[i], areas[i]
            j = i - 1
            while j >= 0 and areas[j] < ka:
                caps[j + 1], areas[j + 1] = caps[j], areas[j]
                j -= 1
            caps[j + 1], areas[j + 1] = kc, ka

        kept = []
        for i in range(len(caps)):
            if any(math.hypot(caps[i]["x"] - k["x"], caps[i]["y"] - k["y"]) < 24.0 for k in kept):
                continue
            kept.append(caps[i])

        self.assertEqual([k["id"] for k in kept], ["B", "D"])

    def test_v1_dlt_4pt_homography_accuracy(self):
        # Faithful mirror of teensy_vision.cpp computeHomography4Pt
        src = [(18.0, 22.0), (298.0, 14.0), (306.0, 228.0), (12.0, 220.0)]
        dst = [(0.0, 0.0), (300.0, 0.0), (300.0, 300.0), (0.0, 300.0)]
        A = np.zeros((8, 9), dtype=np.float64)
        for i in range(4):
            u, v = src[i]
            x, y = dst[i]
            A[2 * i] = [u, v, 1, 0, 0, 0, -u * x, -v * x, x]
            A[2 * i + 1] = [0, 0, 0, u, v, 1, -u * y, -v * y, y]
        for col in range(8):
            pivot = col + int(np.argmax(np.abs(A[col:, col])))
            A[[col, pivot]] = A[[pivot, col]]
            A[col] /= A[col, col]
            for row in range(8):
                if row != col:
                    A[row] -= A[row, col] * A[col]
        H = list(A[:, 8]) + [1.0]
        for (u, v), (x, y) in zip(src, dst):
            w = H[6] * u + H[7] * v + H[8]
            mx = (H[0] * u + H[1] * v + H[2]) / w
            my = (H[3] * u + H[4] * v + H[5]) / w
            self.assertAlmostEqual(mx, x, places=3)
            self.assertAlmostEqual(my, y, places=3)

    def test_mega_esp32_teensy_safety_invariants(self):
        mega = _read_text(os.path.join(REPO_ROOT, "firmware", "mega_motion", "mega_motion.ino"))
        self.assertIn("PIN_ESTOP", mega)
        self.assertIn("isAxisLimitOpen(PIN_Z_HOME)", mega)
        self.assertIn("HOMING_TIMEOUT_MS  30000UL", mega)
        self.assertNotIn("recoveryPingCount", mega)
        self.assertIn("CMD_REARM_HOME", mega)
        self.assertIn("CMD_STATUS_READY", mega)
        self.assertIn("CMD_CAP_PROGRESS", mega)
        self.assertIn("CMD_SCAN_WINDOW_OPEN", mega)
        self.assertIn("CMD_AWAIT_NEXT", mega)
        self.assertIn("STATE_BIN_SETTLE", mega)
        self.assertIn("STATE_AWAIT_NEXT_TARGET", mega)
        self.assertIn("STATE_PARKING_FOR_SCAN", mega)
        self.assertIn("isValidSingleTarget", mega)
        self.assertIn("digitalWrite(PIN_SERVO_OE, HIGH);", mega)
        self.assertIn("#define HAS_GRIP_SENSOR        0", mega)

        esp = _read_text(os.path.join(REPO_ROOT, "firmware", "v1_esp32cam", "esp32cam_uart.ino"))
        self.assertIn("camera_config_t config = {};", esp)
        self.assertIn("psramFound()", esp)
        self.assertIn("config.fb_location   = CAMERA_FB_IN_PSRAM;", esp)
        self.assertIn("config.jpeg_quality  = 12;", esp)
        self.assertIn("config.sccb_i2c_port = 0;", esp)
        self.assertIn("staleFb", esp)

        teensy = _read_text(os.path.join(REPO_ROOT, "firmware", "v1_teensy41", "teensy_vision.cpp"))
        self.assertIn("#define V1_CALIBRATION_VERIFIED  0", teensy)
        self.assertIn("computeHomography4Pt", teensy)
        self.assertIn("waitForMegaReady", teensy)
        self.assertIn("receiveFrameWithHeartbeat", teensy)
        self.assertIn("splitMergedSameColorBlob", teensy)
        self.assertIn("selectNextCapClearanceAware", teensy)
        self.assertIn("runClosedLoopSortSession", teensy)
        self.assertNotIn("delay(1500)", teensy)
        self.assertNotIn("180000UL", teensy)


def _draw_synthetic_board(H_inv: np.ndarray, caps_spec: list) -> np.ndarray:
    """Renders synthetic colored circular caps (radius ~15 mm) on a 640x480 rectified canvas."""
    bgr_map = {
        pzv.COLOR_RED:    (20, 20, 230),
        pzv.COLOR_YELLOW: (15, 225, 235),
        pzv.COLOR_GREEN:  (30, 190, 30),
        pzv.COLOR_BLUE:   (230, 60, 20),
    }
    canvas = np.full((480, 640, 3), 230, dtype=np.uint8)
    # Estimate px radius for 15 mm cap
    p0 = cv2.perspectiveTransform(np.array([[[150.0, 150.0]]], dtype=np.float32), H_inv)[0][0]
    p1 = cv2.perspectiveTransform(np.array([[[165.0, 150.0]]], dtype=np.float32), H_inv)[0][0]
    rad_px = max(10, int(round(float(np.linalg.norm(p1 - p0)))))
    for x_mm, y_mm, col in caps_spec:
        uv = cv2.perspectiveTransform(np.array([[[x_mm, y_mm]]], dtype=np.float32), H_inv)[0][0]
        cv2.circle(canvas, (int(round(uv[0])), int(round(uv[1]))), rad_px, bgr_map[col], -1)
    return canvas


class TestContinuousVisionAndEdgeCases(unittest.TestCase):
    def _make_controller(self, td, camera_height_mm=0.0, cap_height_mm=13.0, bridge_mm=35.0):
        cal_path = os.path.join(td, "cal.npz")
        H = fc.compute_homography_from_fiducials([[92, 34], [548, 34], [548, 446], [92, 446]])
        fc.save_validated_calibration(
            cal_path,
            np.array([[320.0, 0, 320.0], [0, 320.0, 240.0], [0, 0, 1.0]]),
            np.zeros((4, 1)), H, "picamera2_imx219", "fisheye", (640, 480), 0.1,
            camera_height_mm=camera_height_mm, cap_height_mm=cap_height_mm,
        )
        return pzv.PiZeroVisionController(
            calib_file=cal_path, camera_type="picamera2_imx219", init_hardware=False,
            bridge_occlusion_half_mm=bridge_mm,
        )

    def test_continuous_arbitrary_offgrid_and_edge_bounds(self):
        with tempfile.TemporaryDirectory() as td:
            ctrl = self._make_controller(td)
            board = _draw_synthetic_board(ctrl.H_inv, [
                (23.4, 178.9, pzv.COLOR_RED),
                (269.2, 41.6, pzv.COLOR_BLUE),
                (10.5, 150.0, pzv.COLOR_GREEN),  # Outside SAFE_PICK_MIN_X_MM (18.0)
            ])
            detected = ctrl.detect_caps(board)
            self.assertEqual(len(detected), 3)
            by_col = {c["color"]: c for c in detected}
            self.assertAlmostEqual(by_col[pzv.COLOR_RED]["boardX_mm"], 23.4, delta=2.0)
            self.assertAlmostEqual(by_col[pzv.COLOR_RED]["boardY_mm"], 178.9, delta=2.0)
            self.assertTrue(by_col[pzv.COLOR_RED]["in_safe_bounds"])
            self.assertTrue(by_col[pzv.COLOR_BLUE]["in_safe_bounds"])
            self.assertFalse(by_col[pzv.COLOR_GREEN]["in_safe_bounds"])

    def test_touching_same_color_caps_split_via_distance_transform(self):
        with tempfile.TemporaryDirectory() as td:
            ctrl = self._make_controller(td)
            # Two touching red caps (30.5 mm apart) + three touching blue caps in a triangle
            specs = [
                (90.0, 90.0, pzv.COLOR_RED),
                (120.5, 90.0, pzv.COLOR_RED),
                (200.0, 180.0, pzv.COLOR_BLUE),
                (231.0, 180.0, pzv.COLOR_BLUE),
                (215.5, 207.0, pzv.COLOR_BLUE),
            ]
            board = _draw_synthetic_board(ctrl.H_inv, specs)
            detected = ctrl.detect_caps(board)
            reds = [c for c in detected if c["color"] == pzv.COLOR_RED]
            blues = [c for c in detected if c["color"] == pzv.COLOR_BLUE]
            self.assertEqual(len(reds), 2, f"Expected 2 split red caps, got {reds}")
            self.assertEqual(len(blues), 3, f"Expected 3 split blue caps, got {blues}")
            for tx, ty, col in specs:
                self.assertTrue(
                    any(c["color"] == col and math.hypot(c["boardX_mm"] - tx, c["boardY_mm"] - ty) < 3.0
                        for c in detected),
                    f"Missing split cap near ({tx}, {ty}) color={col} in {detected}",
                )

    def test_radial_cap_height_parallax_compensation(self):
        with tempfile.TemporaryDirectory() as td:
            H_cam, h_cap = 260.0, 13.0
            ctrl = self._make_controller(td, camera_height_mm=H_cam, cap_height_mm=h_cap)
            true_x, true_y = 35.0, 35.0
            cx, cy = ctrl.cam_center_board_mm
            # Apparent projected position on Z=0 plane is shifted radially outward by 1 / (1 - h/H)
            inv_scale = 1.0 / (1.0 - h_cap / H_cam)
            apparent_x = cx + (true_x - cx) * inv_scale
            apparent_y = cy + (true_y - cy) * inv_scale
            board = _draw_synthetic_board(ctrl.H_inv, [(apparent_x, apparent_y, pzv.COLOR_YELLOW)])
            detected = ctrl.detect_caps(board)
            self.assertEqual(len(detected), 1)
            self.assertAlmostEqual(detected[0]["boardX_mm"], true_x, delta=1.5)
            self.assertAlmostEqual(detected[0]["boardY_mm"], true_y, delta=1.5)

    def test_clearance_first_peels_outer_caps_before_sandwiched_middle(self):
        with tempfile.TemporaryDirectory() as td:
            ctrl = self._make_controller(td)
            caps = [
                {"capId": 1, "color": pzv.COLOR_RED, "pixelX": 100, "pixelY": 100,
                 "boardX_mm": 100.0, "boardY_mm": 150.0, "targetBinId": 1, "in_safe_bounds": True},
                {"capId": 2, "color": pzv.COLOR_YELLOW, "pixelX": 150, "pixelY": 100,
                 "boardX_mm": 131.0, "boardY_mm": 150.0, "targetBinId": 2, "in_safe_bounds": True},
                {"capId": 3, "color": pzv.COLOR_GREEN, "pixelX": 200, "pixelY": 100,
                 "boardX_mm": 162.0, "boardY_mm": 150.0, "targetBinId": 3, "in_safe_bounds": True},
            ]
            # Even starting closest to sandwiched middle cap (131, 150), must peel outer cap first!
            chosen = ctrl.select_next_cap(caps, cur_xy=(131.0, 125.0))
            self.assertIsNotNone(chosen)
            self.assertIn(chosen["capId"], (1, 3))
            self.assertEqual(chosen["blocked_sides"], 1)


class TestClosedLoopBinReimagingAndRecovery(unittest.TestCase):
    def _make_controller(self, td, bridge_mm=35.0):
        cal_path = os.path.join(td, "cal.npz")
        H = fc.compute_homography_from_fiducials([[92, 34], [548, 34], [548, 446], [92, 446]])
        fc.save_validated_calibration(
            cal_path,
            np.array([[320.0, 0, 320.0], [0, 320.0, 240.0], [0, 0, 1.0]]),
            np.zeros((4, 1)), H, "picamera2_imx219", "fisheye", (640, 480), 0.1,
        )
        return pzv.PiZeroVisionController(
            calib_file=cal_path, camera_type="picamera2_imx219", init_hardware=False,
            bridge_occlusion_half_mm=bridge_mm,
        )

    def _run_virtual_mega_session(self, ctrl, world_caps, on_pick_hook=None):
        """
        Simulates the Arduino Mega closed-loop FSM over FakeUART:
          - Accepts 18B CMD_PICK_SINGLE (0x5B), emits ACK -> SCAN_WINDOW_OPEN -> PROGRESS -> AWAIT_NEXT.
          - Executes on_pick_hook(x, y, color, world_caps) when gripper lowers & picks.
          - Supports CMD_PARK_FOR_SCAN (0xB5) -> AWAIT_NEXT and CMD_FINISH_SESSION (0xB6) -> BATCH_COMPLETE.
        """
        now = [0.0]
        picked_commands = []
        park_cmds = [0]
        pending_events = []  # list of (trigger_time, byte)

        def clock():
            return now[0]

        def sleep_fn(dt):
            now[0] += dt
            while pending_events and pending_events[0][0] <= now[0]:
                _, b = pending_events.pop(0)
                ctrl.uart.rx_queue.append(b)

        def mega_script(raw, rx):
            if raw[0] == pzv.PROTOCOL_SINGLE_MAGIC and len(raw) == 18:
                _, color, _, _, bx, by, bin_id = struct.unpack("<BBHHffB", raw[1:16])
                picked_commands.append((bx, by, color, bin_id))
                rx.append(pzv.CMD_BATCH_ACK)
                if on_pick_hook is not None:
                    on_pick_hook(bx, by, color, world_caps)
                # Mega arrives at bin after 2.0 s, emits SCAN_WINDOW_OPEN, then finishes drop at 2.8 s
                pending_events.append((now[0] + 2.0, pzv.CMD_SCAN_WINDOW_OPEN))
                pending_events.append((now[0] + 2.8, pzv.CMD_CAP_PROGRESS))
                pending_events.append((now[0] + 2.8, pzv.CMD_AWAIT_NEXT))
            elif raw == bytes([pzv.CMD_PARK_FOR_SCAN]):
                park_cmds[0] += 1
                pending_events.append((now[0] + 1.5, pzv.CMD_AWAIT_NEXT))
            elif raw == bytes([pzv.CMD_FINISH_SESSION]):
                pending_events.append((now[0] + 0.5, pzv.CMD_BATCH_COMPLETE))

        ctrl.uart = FakeUART(mega_script)
        capture_fn = lambda: _draw_synthetic_board(ctrl.H_inv, [(c[0], c[1], c[2]) for c in world_caps])
        res = ctrl.run_closed_loop_sort_session(capture_fn=capture_fn, clock=clock, sleep_fn=sleep_fn)
        return res, picked_commands, park_cmds[0]

    def test_bin_side_reimaging_reacquires_bumped_neighbor(self):
        with tempfile.TemporaryDirectory() as td:
            ctrl = self._make_controller(td, bridge_mm=35.0)
            # Cap 1 (Green) picked first; when picked, Cap 2 (Blue) is bumped by (+32, -24) mm!
            world_caps = [
                [80.0, 120.0, pzv.COLOR_GREEN],
                [150.0, 220.0, pzv.COLOR_BLUE],
            ]

            def bump_on_first_pick(bx, by, color, w_caps):
                # Remove picked cap
                for i, c in enumerate(w_caps):
                    if math.hypot(c[0] - bx, c[1] - by) < 10.0:
                        w_caps.pop(i)
                        break
                # Physically bump the remaining blue cap to a brand-new location!
                if color == pzv.COLOR_GREEN and w_caps:
                    w_caps[0][0] += 32.0
                    w_caps[0][1] -= 24.0

            res, cmds, parks = self._run_virtual_mega_session(ctrl, world_caps, bump_on_first_pick)
            self.assertEqual(res["status"], "COMPLETE")
            self.assertEqual(res["picked"], 2)
            self.assertEqual(len(world_caps), 0)
            self.assertEqual(len(cmds), 2)
            # Second pick MUST target the NEW bumped coordinates (182, 196), NOT stale (150, 220)!
            self.assertAlmostEqual(cmds[1][0], 182.0, delta=2.5)
            self.assertAlmostEqual(cmds[1][1], 196.0, delta=2.5)
            self.assertGreaterEqual(parks, 1)

    def test_bridge_occlusion_triggers_clear_fov_park_and_reveals_hidden_cap(self):
        with tempfile.TemporaryDirectory() as td:
            ctrl = self._make_controller(td, bridge_mm=35.0)
            # Cap 1 (Red) at (60, 140) is closer to (0,0) (152 mm) so picked first -> Bin 1 (Y=55).
            # Cap 2 (Yellow) at (220, 56) (227 mm from 0,0) sits inside Bin 1's [20..90] bridge occlusion strip!
            world_caps = [
                [60.0, 140.0, pzv.COLOR_RED],
                [220.0, 56.0, pzv.COLOR_YELLOW],
            ]

            def normal_pick(bx, by, color, w_caps):
                for i, c in enumerate(w_caps):
                    if math.hypot(c[0] - bx, c[1] - by) < 10.0:
                        w_caps.pop(i)
                        break

            res, cmds, parks = self._run_virtual_mega_session(ctrl, world_caps, normal_pick)
            self.assertEqual(res["status"], "COMPLETE")
            self.assertEqual(res["picked"], 2)
            self.assertEqual(len(world_caps), 0)
            # First park reveals the Yellow cap hidden under Bin 1's bridge; second park confirms empty!
            self.assertGreaterEqual(parks, 2)
            self.assertTrue(any(c[2] == pzv.COLOR_YELLOW for c in cmds))

    def test_missed_pick_retries_and_quarantines_stuck_cap_without_infinite_loop(self):
        with tempfile.TemporaryDirectory() as td:
            ctrl = self._make_controller(td, bridge_mm=35.0)
            # Red cap at (140, 140) is glued/stuck and never lifts; Blue cap at (220, 220) lifts normally
            world_caps = [
                [140.0, 140.0, pzv.COLOR_RED],
                [220.0, 220.0, pzv.COLOR_BLUE],
            ]

            def stuck_red_pick(bx, by, color, w_caps):
                if color == pzv.COLOR_RED:
                    return  # Simulate missed pick: Red cap stays stuck on the board!
                for i, c in enumerate(w_caps):
                    if c[2] == color and math.hypot(c[0] - bx, c[1] - by) < 10.0:
                        w_caps.pop(i)
                        break

            res, cmds, _ = self._run_virtual_mega_session(ctrl, world_caps, stuck_red_pick)
            self.assertEqual(res["status"], "COMPLETE")
            self.assertEqual(len(res["quarantined"]), 1)
            red_attempts = [c for c in cmds if c[2] == pzv.COLOR_RED]
            blue_attempts = [c for c in cmds if c[2] == pzv.COLOR_BLUE]
            self.assertEqual(len(red_attempts), pzv.MAX_PICK_RETRIES)
            self.assertEqual(len(blue_attempts), 1)
            # Blue cap was sorted cleanly even though Red was stuck!
            self.assertEqual(len(world_caps), 1)
            self.assertEqual(world_caps[0][2], pzv.COLOR_RED)


class TestHTMLCodeVaultSyncAndDocsAudit(unittest.TestCase):
    def test_html_code_vaults_match_standalone_firmware_verbatim(self):
        checks = [
            ("v1.html", "code-teensy",    "firmware/v1_teensy41/teensy_vision.cpp"),
            ("v1.html", "code-mega",      "firmware/mega_motion/mega_motion.ino"),
            ("v1.html", "code-esp32",     "firmware/v1_esp32cam/esp32cam_uart.ino"),
            ("v1.html", "code-protocol",  "firmware/common/packet_protocol.h"),
            ("v2.html", "code-pizero",    "firmware/v2_pizero2w/pi_zero_vision.py"),
            ("v2.html", "code-calibrate", "firmware/v2_pizero2w/fisheye_calibrate.py"),
            ("v2.html", "code-mega",      "firmware/mega_motion/mega_motion.ino"),
            ("v2.html", "code-protocol",  "firmware/common/packet_protocol.h"),
        ]
        cache = {}
        for html_file, block_id, src_file in checks:
            if html_file not in cache:
                text = _read_text(os.path.join(REPO_ROOT, html_file))
                cache[html_file] = dict(
                    re.findall(r'<div id="(code-[^"]+)"[^>]*>\s*<pre><code[^>]*>(.*?)</code></pre>', text, re.S)
                )
            self.assertIn(block_id, cache[html_file], f"Missing {block_id} in {html_file}")
            embedded = html.unescape(cache[html_file][block_id]).replace("\r\n", "\n").strip()
            standalone = (
                _read_text(os.path.join(REPO_ROOT, src_file))
                .replace("\r\n", "\n")
                .replace('#include "../common/packet_protocol.h"', '#include "packet_protocol.h"')
                .strip()
            )
            self.assertEqual(embedded, standalone, f"Code vault mismatch: {html_file}#{block_id} vs {src_file}")

    def test_zero_banned_claims_in_documentation(self):
        banned = [
            (r"limit switch ISR|homing ISR|e-stop interrupt|stop interrupt|limit switch interrupts", "unsubstantiated ISR claim"),
            (r"Sensorless Homing", "Sensorless Homing misnomer"),
            (r"\bCRC-?8\b", "wrong CRC8 label"),
            (r"5V\s*/\s*VIN", "ambiguous 5V/VIN"),
            (r"V_ref\s*/\s*\(1\.414\s*\*\s*R_sense\)", "broken TMC2208 equation"),
            (r"Approved for Production|Production Release|Production-ready|Production Firmware Vault", "premature production sign-off"),
            (r"3 isolated buck rails|electrically isolated branches|Isolated high-current|Clean, isolated 5V|strictly isolated on|3-branch isolated|Fully Isolated Rails|servo rail isolation", "false galvanic isolation claim"),
            (r"2400 steps/s|60 mm/s on 16 MHz|Up to 20 kHz pulse rate", "stale speed/frequency claim"),
            (r"5V Tolerant on select pins", "false Teensy 5V tolerance"),
            (r"5\s*ms inter-chunk", "stale 5ms pacing claim"),
            (r"completes depositing|in-flight pickup", "unverified-cap comms recovery claim"),
        ]
        doc_files = [
            "README.md", "firmware/README.md", "index.html",
            "v1.html", "v2.html", "pdf_dossier.html", "pdf_dossier_v2.html",
        ]
        for fn in doc_files:
            text = _read_text(os.path.join(REPO_ROOT, fn))
            for pat, label in banned:
                self.assertIsNone(
                    re.search(pat, text, re.I),
                    f"Banned claim ({label}) still present in {fn}"
                )
            # Check <10 us claim is absent everywhere except explicit prohibition in firmware/README.md
            if fn != "firmware/README.md":
                self.assertIsNone(
                    re.search(r"10\s*[uµμ]s|10\s*microseconds", text, re.I),
                    f"Unsubstantiated 10us claim still present in {fn}"
                )

        # Ensure V2 dossier has no leftover V1 Serial2/OV2640/QVGA/921,600 baud architecture prose
        v2_dossier = _read_text(os.path.join(REPO_ROOT, "pdf_dossier_v2.html"))
        for v1_leftover in [r"OV2640", r"\bQVGA\b", r"921,600\s*Baud", r"Serial2", r"BGR888"]:
            self.assertIsNone(
                re.search(v1_leftover, v2_dossier, re.I),
                f"V1 leftover ({v1_leftover}) still present in pdf_dossier_v2.html"
            )


if __name__ == "__main__":
    unittest.main()
