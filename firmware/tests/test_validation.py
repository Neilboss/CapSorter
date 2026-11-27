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
import lcd_hud as hud_mod
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
        self.assertIn("HOMING_TIMEOUT_MS    23000UL", mega)
        self.assertIn("MAX_HOMING_SEEK_X_STEPS", mega)
        self.assertIn("MAX_HOMING_SEEK_Y_STEPS", mega)
        self.assertNotIn("recoveryPingCount", mega)
        self.assertIn("CMD_REARM_HOME", mega)
        self.assertIn("CMD_STATUS_READY", mega)
        self.assertIn("CMD_CAP_PROGRESS", mega)
        self.assertIn("CMD_SCAN_WINDOW_OPEN", mega)
        self.assertIn("CMD_AWAIT_NEXT", mega)
        self.assertIn("STATE_BIN_SETTLE", mega)
        self.assertIn("STATE_AWAIT_NEXT_TARGET", mega)
        self.assertIn("STATE_PARKING_FOR_SCAN", mega)
        self.assertIn("STATE_FAILSAFE_PARKING", mega)
        self.assertIn("inActiveCapCycle", mega)
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


class TestV2USBSerialAndLCDHUD(unittest.TestCase):
    def _make_controller(self, td, state_callback=None):
        cal_path = os.path.join(td, "cal.npz")
        H = fc.compute_homography_from_fiducials([[92, 34], [548, 34], [548, 446], [92, 446]])
        fc.save_validated_calibration(
            cal_path,
            np.array([[320.0, 0, 320.0], [0, 320.0, 240.0], [0, 0, 1.0]]),
            np.zeros((4, 1)), H, "picamera2_imx219", "fisheye", (640, 480), 0.1,
        )
        return pzv.PiZeroVisionController(
            calib_file=cal_path, camera_type="picamera2_imx219",
            init_hardware=False, state_callback=state_callback,
        )

    def test_resolve_serial_port_priority_and_udev_rules(self):
        # 1. Explicit override always wins
        self.assertEqual(pzv.resolve_serial_port("/dev/ttyUSB9"), "/dev/ttyUSB9")

        # 2. Persistent udev symlink /dev/capsorter_mega has highest auto priority
        self.assertEqual(
            pzv.resolve_serial_port("auto", exists_fn=lambda p: p == "/dev/capsorter_mega", glob_fn=lambda _: []),
            "/dev/capsorter_mega",
        )

        # 3. CH9102 / CP2102 by-id match wins over generic ttyUSB/ttyACM
        by_id_ch9102 = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_54321-if00"
        self.assertEqual(
            pzv.resolve_serial_port(
                "auto",
                exists_fn=lambda _: False,
                glob_fn=lambda pat: [by_id_ch9102] if "by-id" in pat else ["/dev/ttyUSB0"],
            ),
            by_id_ch9102,
        )

        # 4. Falls through to /dev/ttyUSB0, then /dev/ttyACM0, then /dev/serial0
        self.assertEqual(
            pzv.resolve_serial_port(
                "auto",
                exists_fn=lambda _: False,
                glob_fn=lambda pat: ["/dev/ttyUSB0"] if "ttyUSB" in pat else [],
            ),
            "/dev/ttyUSB0",
        )
        self.assertEqual(
            pzv.resolve_serial_port(
                "auto",
                exists_fn=lambda _: False,
                glob_fn=lambda pat: ["/dev/ttyACM0"] if "ttyACM" in pat else [],
            ),
            "/dev/ttyACM0",
        )
        self.assertEqual(
            pzv.resolve_serial_port("auto", exists_fn=lambda _: False, glob_fn=lambda _: []),
            "/dev/serial0",
        )

        # 5. Verify shipped udev rules cover both WCH CH9102 (1a86) and Silicon Labs CP2102 (10c4)
        udev = _read_text(os.path.join(V2_DIR, "99-capsorter-mega.rules"))
        self.assertIn('idVendor}=="1a86"', udev)
        self.assertIn('idVendor}=="10c4"', udev)
        self.assertIn('SYMLINK+="capsorter_mega"', udev)

    def test_usb_serial_disconnect_mid_cycle_returns_link_lost_cleanly(self):
        with tempfile.TemporaryDirectory() as td:
            events = []
            ctrl = self._make_controller(td, state_callback=lambda ev, pl: events.append((ev, pl)))

            class FailingUSBSerial(FakeUART):
                def __init__(self):
                    super().__init__()
                    self.writes = 0

                def write(self, data: bytes):
                    self.writes += 1
                    if self.writes == 1:
                        self.rx_queue.append(pzv.CMD_BATCH_ACK)
                        return len(data)
                    raise OSError("Simulated USB-TTL unplug (Errno 5)")

            ctrl.uart = FailingUSBSerial()
            board = _draw_synthetic_board(ctrl.H_inv, [(120.0, 140.0, pzv.COLOR_RED)])
            res = ctrl.run_closed_loop_sort_session(
                capture_fn=lambda: board, clock=lambda: 0.0, sleep_fn=lambda _: None
            )
            self.assertEqual(res["status"], "LINK_LOST")
            self.assertTrue(any(ev == "LINK_LOST" for ev, _ in events))

            # Verify send_rearm_home sends 0xB4 cleanly on healthy link
            ctrl.uart = FakeUART()
            self.assertTrue(ctrl.send_rearm_home())
            self.assertEqual(ctrl.uart.tx_log[-1], bytes([pzv.CMD_REARM_HOME]))

    def test_lcd_hud_kinematics_rendering_rgb565_and_touch_controls(self):
        # Verify trapezoidal & triangular move duration matches Mega kinematics
        self.assertAlmostEqual(hud_mod.compute_axis_move_time(0.0), 0.0, places=5)
        self.assertAlmostEqual(hud_mod.compute_axis_move_time(140.0), (140.0 / 70.0) + (70.0 / 150.0), places=4)
        self.assertAlmostEqual(hud_mod.compute_axis_move_time(15.0), 2.0 * math.sqrt(15.0 / 150.0), places=4)

        paused_calls = []
        rearm_calls = []
        hud = hud_mod.CapSorterLCDHUD(
            fb_device="",
            on_pause_toggle=lambda p: paused_calls.append(p),
            on_rearm=lambda: rearm_calls.append(True),
        )
        caps = [
            {"capId": 1, "color": pzv.COLOR_RED, "boardX_mm": 80.0, "boardY_mm": 120.0, "in_safe_bounds": True},
            {"capId": 2, "color": pzv.COLOR_BLUE, "boardX_mm": 210.0, "boardY_mm": 220.0, "in_safe_bounds": True},
        ]
        hud.update_state("READY", {"caps": caps, "serial_port": "/dev/capsorter_mega", "scans": 1}, now=1.0)
        hud.update_state("TARGET_DISPATCHED", {"target": caps[0], "caps": caps}, now=1.0)

        # Mid-travel frame should advance gantry along (0,0) -> (80,120)
        frame_mid = hud.render_frame_bgr(now=2.0)
        self.assertEqual(frame_mid.shape, (hud_mod.LCD_HEIGHT, hud_mod.LCD_WIDTH, 3))
        self.assertEqual(frame_mid.dtype, np.uint8)
        self.assertGreater(hud.predictor.x, 5.0)
        self.assertGreater(hud.predictor.y, 5.0)

        # Bin scan window anchors gantry at bin (X=355) & enables bridge occlusion overlay
        hud.update_state("SCAN_WINDOW_OPEN", {"bin_y": 55.0, "caps": [caps[1]], "scans": 2}, now=5.0)
        hud.update_state("CAP_SORTED", {"color": pzv.COLOR_RED}, now=5.8)
        frame_bin = hud.render_frame_bgr(now=5.2)
        rgb565 = hud_mod.bgr_to_rgb565_bytes(frame_bin)
        self.assertEqual(len(rgb565), hud_mod.LCD_WIDTH * hud_mod.LCD_HEIGHT * 2)
        self.assertEqual(hud.state["bin_counts"][pzv.COLOR_RED], 1)

        # Verify touch hit-testing for PAUSE, RE-ARM, and VIEW buttons
        for key in ("pause", "rearm", "view"):
            x0, y0, x1, y1 = hud_mod.TOUCH_BUTTONS[key]
            cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
            self.assertEqual(hud.handle_touch_px(cx, cy), key)
        self.assertEqual(paused_calls, [True])
        self.assertEqual(rearm_calls, [True])
        self.assertEqual(hud.view_mode, "CAM")
        bgra32 = hud_mod.bgr_to_bgra32_bytes(frame_bin)
        self.assertEqual(len(bgra32), hud_mod.LCD_WIDTH * hud_mod.LCD_HEIGHT * 4)


class SimulatedMega2560:
    """
    Faithful discrete-time mirror of firmware/mega_motion/mega_motion.ino for
    end-to-end closed-loop UART, coordinate-frame, UART-cut finish-and-park,
    E-stop, and NC limit switch simulation against PiZeroVisionController.
    """

    BIN_X = {1: 355.0, 2: 355.0, 3: 355.0, 4: 355.0}
    BIN_Y = {1: 55.0, 2: 130.0, 3: 205.0, 4: 280.0}

    def __init__(self, tool_offset_xy=(0.0, 0.0), max_travel_xy=(370.0, 300.0)):
        self.tool_offset_x, self.tool_offset_y = float(tool_offset_xy[0]), float(tool_offset_xy[1])
        self.max_travel_x, self.max_travel_y = float(max_travel_xy[0]), float(max_travel_xy[1])
        self.state = "IDLE"
        self.is_homed = True
        self.closed_loop = False
        self.comms_fault = False
        self.steppers_enabled = True
        self.servos_enabled = True
        self.estop_active = False
        self.limit_open = False
        self.estop_count = 0
        self.limit_count = 0
        self.mach_x = 0.0
        self.mach_y = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.current_cap = None
        self.state_until = 0.0
        self.last_heartbeat = 0.0
        self.rx_buf = bytearray()
        self.tx_out = deque()
        self.pick_log = []      # (mach_x, mach_y, board_x, board_y, color)
        self.deposit_log = []   # (mach_x, mach_y, bin_id, color)
        self.park_log = []      # (mach_x, mach_y, reason_state)
        self.on_pick_world = None

    def _move_machine(self, mx, my, now):
        self.target_x = min(self.max_travel_x, max(0.0, float(mx)))
        self.target_y = min(self.max_travel_y, max(0.0, float(my)))
        dt = max(0.05, hud_mod.compute_xy_move_time(self.target_x - self.mach_x, self.target_y - self.mach_y))
        self.state_until = now + dt

    def _is_valid_target(self, bx, by, bin_id):
        if not (math.isfinite(bx) and math.isfinite(by)):
            return False
        if not (pzv.SAFE_PICK_MIN_X_MM <= bx <= pzv.SAFE_PICK_MAX_X_MM):
            return False
        if not (pzv.SAFE_PICK_MIN_Y_MM <= by <= pzv.SAFE_PICK_MAX_Y_MM):
            return False
        mx = bx + self.tool_offset_x
        my = by + self.tool_offset_y
        if not (0.0 <= mx <= self.max_travel_x and 0.0 <= my <= self.max_travel_y):
            return False
        return 1 <= bin_id <= 4

    def _trigger_hard_fault(self):
        self.steppers_enabled = False
        self.servos_enabled = False
        self.closed_loop = False
        self.is_homed = False
        self.state = "HARD_FAULT"
        self.tx_out.append(pzv.CMD_HARD_FAULT)

    def feed_host_bytes(self, data: bytes, now: float):
        self.rx_buf.extend(data)
        while self.rx_buf:
            b = self.rx_buf[0]
            if b == pzv.CMD_QUERY_STATUS:
                self.rx_buf.pop(0)
                if self.state == "IDLE" and self.is_homed and not self.comms_fault and not self.estop_active:
                    self.tx_out.append(pzv.CMD_STATUS_READY)
                elif self.state in ("FAILSAFE_HALT", "HARD_FAULT"):
                    self.tx_out.append(pzv.CMD_STATUS_FAULT)
                else:
                    self.tx_out.append(pzv.CMD_STATUS_BUSY)
            elif b == pzv.CMD_HEARTBEAT_PING:
                self.rx_buf.pop(0)
                self.last_heartbeat = now
            elif b == pzv.CMD_REARM_HOME:
                self.rx_buf.pop(0)
                if self.state in ("FAILSAFE_HALT", "HARD_FAULT"):
                    if self.estop_active:
                        self.tx_out.append(pzv.CMD_STATUS_FAULT)
                    else:
                        self.mach_x = 0.0
                        self.mach_y = 0.0
                        self.steppers_enabled = True
                        self.servos_enabled = True
                        self.is_homed = True
                        self.comms_fault = False
                        self.closed_loop = False
                        self.estop_count = 0
                        self.limit_count = 0
                        self.last_heartbeat = now
                        self.state = "IDLE"
                        self.tx_out.append(pzv.CMD_STATUS_READY)
                else:
                    self.tx_out.append(pzv.CMD_STATUS_READY if self.state == "IDLE" else pzv.CMD_STATUS_BUSY)
            elif b == pzv.CMD_PARK_FOR_SCAN:
                self.rx_buf.pop(0)
                if self.state not in ("AWAIT_NEXT_TARGET", "IDLE") or not self.is_homed or self.comms_fault:
                    self.tx_out.append(pzv.CMD_BATCH_NACK)
                else:
                    self.closed_loop = True
                    self.last_heartbeat = now
                    self._move_machine(0.0, 0.0, now)
                    self.state = "PARKING_FOR_SCAN"
            elif b == pzv.CMD_FINISH_SESSION:
                self.rx_buf.pop(0)
                if self.state not in ("AWAIT_NEXT_TARGET", "IDLE") or not self.is_homed or self.comms_fault:
                    self.tx_out.append(pzv.CMD_BATCH_NACK)
                else:
                    self.closed_loop = False
                    self.last_heartbeat = now
                    self._move_machine(0.0, 0.0, now)
                    self.state = "PARKING_HOME"
            elif b == pzv.PROTOCOL_SINGLE_MAGIC:
                if len(self.rx_buf) < 18:
                    break
                pkt = bytes(self.rx_buf[:18])
                del self.rx_buf[:18]
                if self.state not in ("IDLE", "AWAIT_NEXT_TARGET") or not self.is_homed or self.comms_fault:
                    self.tx_out.append(pzv.CMD_BATCH_NACK)
                    continue
                payload = pkt[1:16]
                rx_crc = struct.unpack("<H", pkt[16:18])[0]
                if pzv.crc16_ccitt(payload) != rx_crc:
                    self.tx_out.append(pzv.CMD_BATCH_NACK)
                    continue
                cap_id, color, px, py, bx, by, bin_id = struct.unpack("<BBHHffB", payload)
                if not self._is_valid_target(bx, by, bin_id):
                    self.tx_out.append(pzv.CMD_BATCH_NACK)
                    continue
                self.closed_loop = True
                self.current_cap = {
                    "capId": cap_id, "color": color, "boardX_mm": bx, "boardY_mm": by, "targetBinId": bin_id
                }
                self.last_heartbeat = now
                self.tx_out.append(pzv.CMD_BATCH_ACK)
                self._move_machine(bx + self.tool_offset_x, by + self.tool_offset_y, now)
                self.state = "MOVING_TO_CAP"
            else:
                self.rx_buf.pop(0)

    def step(self, now: float):
        if self.state != "HARD_FAULT":
            if self.estop_active:
                self.estop_count += 1
                if self.estop_count >= 2:
                    self._trigger_hard_fault()
                    return
            else:
                self.estop_count = 0
            if self.limit_open:
                self.limit_count += 1
                if self.limit_count >= 3:
                    self._trigger_hard_fault()
                    return
            else:
                self.limit_count = 0

        # Watchdog: 1.2 s heartbeat timeout (mirrors checkCommunicationLoss + executeSafeCommsLossPolicy)
        if (
            self.state not in ("IDLE", "FAILSAFE_PARKING", "FAILSAFE_HALT", "HARD_FAULT")
            and not self.comms_fault
            and (now - self.last_heartbeat > 1.2)
        ):
            self.comms_fault = True
            self.tx_out.append(pzv.CMD_RECOVERY_HALT)
            in_active = self.state in (
                "MOVING_TO_CAP", "LOWER_Z", "GRIP_CAP", "LIFT_Z",
                "TRAVERSE_BIN", "BIN_SETTLE", "LOWER_BIN", "RELEASE_CAP", "RETRACT_Z",
            )
            if not in_active:
                self.closed_loop = False
                self._move_machine(0.0, 0.0, now)
                self.state = "FAILSAFE_PARKING"

        if now < self.state_until:
            return

        if self.state == "MOVING_TO_CAP":
            self.mach_x, self.mach_y = self.target_x, self.target_y
            self.state_until = now + 0.35
            self.state = "LOWER_Z"
        elif self.state == "LOWER_Z":
            self.state_until = now + 0.30
            self.state = "GRIP_CAP"
        elif self.state == "GRIP_CAP":
            cap = self.current_cap
            self.pick_log.append((self.mach_x, self.mach_y, cap["boardX_mm"], cap["boardY_mm"], cap["color"]))
            if self.on_pick_world is not None:
                self.on_pick_world(cap["boardX_mm"], cap["boardY_mm"], cap["color"])
            self.state_until = now + 0.35
            self.state = "LIFT_Z"
        elif self.state == "LIFT_Z":
            b = self.current_cap["targetBinId"]
            self._move_machine(self.BIN_X[b], self.BIN_Y[b], now)
            self.state = "TRAVERSE_BIN"
        elif self.state == "TRAVERSE_BIN":
            self.mach_x, self.mach_y = self.target_x, self.target_y
            if self.closed_loop and not self.comms_fault:
                self.state_until = now + 0.12
                self.state = "BIN_SETTLE"
            else:
                self.state_until = now + 0.25
                self.state = "LOWER_BIN"
        elif self.state == "BIN_SETTLE":
            if not self.comms_fault:
                self.tx_out.append(pzv.CMD_SCAN_WINDOW_OPEN)
            self.state_until = now + 0.25
            self.state = "LOWER_BIN"
        elif self.state == "LOWER_BIN":
            self.state_until = now + 0.25
            self.state = "RELEASE_CAP"
        elif self.state == "RELEASE_CAP":
            cap = self.current_cap
            self.deposit_log.append((self.mach_x, self.mach_y, cap["targetBinId"], cap["color"]))
            self.state_until = now + 0.30
            self.state = "RETRACT_Z"
        elif self.state == "RETRACT_Z":
            self.tx_out.append(pzv.CMD_CAP_PROGRESS)
            if self.comms_fault:
                self.tx_out.append(pzv.CMD_RECOVERY_HALT)
                self.closed_loop = False
                self._move_machine(0.0, 0.0, now)
                self.state = "FAILSAFE_PARKING"
            else:
                self.tx_out.append(pzv.CMD_AWAIT_NEXT)
                self.state = "AWAIT_NEXT_TARGET"
        elif self.state == "PARKING_FOR_SCAN":
            self.mach_x, self.mach_y = self.target_x, self.target_y
            self.park_log.append((self.mach_x, self.mach_y, "PARKING_FOR_SCAN"))
            self.tx_out.append(pzv.CMD_AWAIT_NEXT)
            self.state = "AWAIT_NEXT_TARGET"
        elif self.state == "PARKING_HOME":
            self.mach_x, self.mach_y = self.target_x, self.target_y
            self.park_log.append((self.mach_x, self.mach_y, "PARKING_HOME"))
            self.closed_loop = False
            self.tx_out.append(pzv.CMD_BATCH_COMPLETE)
            self.state = "IDLE"
        elif self.state == "FAILSAFE_PARKING":
            self.mach_x, self.mach_y = self.target_x, self.target_y
            self.park_log.append((self.mach_x, self.mach_y, "FAILSAFE_PARKING"))
            self.steppers_enabled = False
            self.is_homed = False
            self.closed_loop = False
            self.state = "FAILSAFE_HALT"
            self.tx_out.append(pzv.CMD_RECOVERY_HALT)


class TestEndToEndFirmwareSimulation(unittest.TestCase):
    def _make_controller(self, td, state_callback=None):
        cal_path = os.path.join(td, "cal.npz")
        H = fc.compute_homography_from_fiducials([[92, 34], [548, 34], [548, 446], [92, 446]])
        fc.save_validated_calibration(
            cal_path,
            np.array([[320.0, 0, 320.0], [0, 320.0, 240.0], [0, 0, 1.0]]),
            np.zeros((4, 1)), H, "picamera2_imx219", "fisheye", (640, 480), 0.1,
        )
        return pzv.PiZeroVisionController(
            calib_file=cal_path, camera_type="picamera2_imx219",
            init_hardware=False, state_callback=state_callback,
        )

    def _wire_mega_uart(self, ctrl, mega, now_ref, cut_at_time=None):
        """Connects PiZeroVisionController <-> SimulatedMega2560 via paired virtual UART."""

        class PairedUART:
            def __init__(self):
                self.closed = False

            @property
            def in_waiting(self):
                if cut_at_time is not None and now_ref[0] >= cut_at_time[0]:
                    return 0
                return len(mega.tx_out)

            def write(self, data: bytes):
                if cut_at_time is not None and now_ref[0] >= cut_at_time[0]:
                    raise OSError("Simulated UART cut")
                mega.feed_host_bytes(bytes(data), now_ref[0])
                mega.step(now_ref[0])
                return len(data)

            def flush(self):
                pass

            def reset_input_buffer(self):
                mega.tx_out.clear()

            def read(self, n: int = 1) -> bytes:
                if cut_at_time is not None and now_ref[0] >= cut_at_time[0]:
                    raise OSError("Simulated UART cut")
                out = bytearray()
                for _ in range(n):
                    if not mega.tx_out:
                        break
                    out.append(mega.tx_out.popleft())
                return bytes(out)

            def close(self):
                self.closed = True

        def sleep_fn(dt):
            steps = max(1, int(math.ceil(dt / 0.05)))
            sub_dt = dt / steps
            for _ in range(steps):
                now_ref[0] += sub_dt
                mega.step(now_ref[0])

        ctrl.uart = PairedUART()
        return sleep_fn

    def test_nonzero_tool_offset_separates_board_picks_from_bins_and_home_park(self):
        with tempfile.TemporaryDirectory() as td:
            hud = hud_mod.CapSorterLCDHUD(fb_device="")
            ctrl = self._make_controller(td, state_callback=hud.update_state)
            mega = SimulatedMega2560(tool_offset_xy=(16.5, 11.0), max_travel_xy=(370.0, 300.0))
            now = [0.0]
            sleep_fn = self._wire_mega_uart(ctrl, mega, now)

            world_caps = [
                [80.0, 95.0, pzv.COLOR_RED],
                [210.0, 190.0, pzv.COLOR_BLUE],
            ]

            def on_pick(bx, by, col):
                for i, c in enumerate(world_caps):
                    if c[2] == col and math.hypot(c[0] - bx, c[1] - by) < 10.0:
                        world_caps.pop(i)
                        break

            mega.on_pick_world = on_pick
            self.assertTrue(ctrl.wait_for_mega_ready(timeout_s=5.0, clock=lambda: now[0], sleep_fn=sleep_fn))
            res = ctrl.run_closed_loop_sort_session(
                capture_fn=lambda: _draw_synthetic_board(ctrl.H_inv, [(c[0], c[1], c[2]) for c in world_caps]),
                clock=lambda: now[0],
                sleep_fn=sleep_fn,
            )
            self.assertEqual(res["status"], "COMPLETE")
            self.assertEqual(res["picked"], 2)
            self.assertEqual(len(world_caps), 0)
            self.assertEqual(mega.state, "IDLE")

            # Verify board pick positions added TOOL_OFFSET (+16.5, +11.0)
            for mx, my, bx, by, _ in mega.pick_log:
                self.assertAlmostEqual(mx, bx + 16.5, delta=0.1)
                self.assertAlmostEqual(my, by + 11.0, delta=0.1)

            # Verify external bins stayed in exact machine space (NO double tool-offset!)
            for mx, my, bin_id, _ in mega.deposit_log:
                self.assertAlmostEqual(mx, mega.BIN_X[bin_id], delta=0.1)
                self.assertAlmostEqual(my, mega.BIN_Y[bin_id], delta=0.1)

            # Verify all parks returned to true machine (0.0, 0.0) (NOT 16.5, 11.0!)
            self.assertGreaterEqual(len(mega.park_log), 2)
            for mx, my, _ in mega.park_log:
                self.assertAlmostEqual(mx, 0.0, delta=0.05)
                self.assertAlmostEqual(my, 0.0, delta=0.05)

            self.assertEqual(hud.state["bin_counts"][pzv.COLOR_RED], 1)
            self.assertEqual(hud.state["bin_counts"][pzv.COLOR_BLUE], 1)

    def test_uart_cut_mid_cap_finishes_active_cap_parks_zero_halts_and_rearms(self):
        with tempfile.TemporaryDirectory() as td:
            ctrl = self._make_controller(td)
            mega = SimulatedMega2560(tool_offset_xy=(12.0, 8.0))
            now = [0.0]
            cut_time = [1.0]  # Cut UART 1.0 s into Cap #1 while gantry is moving to Cap #1
            sleep_fn = self._wire_mega_uart(ctrl, mega, now, cut_at_time=cut_time)

            world_caps = [
                [85.0, 110.0, pzv.COLOR_RED],
                [220.0, 210.0, pzv.COLOR_GREEN],
            ]

            def on_pick(bx, by, col):
                for i, c in enumerate(world_caps):
                    if c[2] == col and math.hypot(c[0] - bx, c[1] - by) < 10.0:
                        world_caps.pop(i)
                        break

            mega.on_pick_world = on_pick
            res = ctrl.run_closed_loop_sort_session(
                capture_fn=lambda: _draw_synthetic_board(ctrl.H_inv, [(c[0], c[1], c[2]) for c in world_caps]),
                clock=lambda: now[0],
                sleep_fn=sleep_fn,
            )
            # Pi cleanly detects LINK_LOST and closes stale fd so main() can reconnect
            self.assertEqual(res["status"], "LINK_LOST")
            self.assertIsNone(ctrl.uart)

            # Advance Mega independently for 20 s after UART cut
            for _ in range(400):
                now[0] += 0.05
                mega.step(now[0])

            # Mega MUST finish ONLY Cap #1 into Bin 1, park at true machine (0,0), disable steppers & halt!
            self.assertEqual(len(mega.deposit_log), 1)
            self.assertEqual(mega.deposit_log[0][2], 1)  # Red -> Bin 1
            self.assertEqual(len(world_caps), 1)         # Green Cap #2 untouched!
            self.assertEqual(world_caps[0][2], pzv.COLOR_GREEN)
            self.assertEqual(mega.state, "FAILSAFE_HALT")
            self.assertFalse(mega.steppers_enabled)
            self.assertAlmostEqual(mega.mach_x, 0.0, delta=0.05)
            self.assertAlmostEqual(mega.mach_y, 0.0, delta=0.05)

            # Reconnect link: heartbeats alone must NOT clear halt; explicit CMD_REARM_HOME must recover!
            cut_time[0] = 1e9
            sleep_fn = self._wire_mega_uart(ctrl, mega, now, cut_at_time=cut_time)
            self.assertFalse(ctrl.wait_for_mega_ready(timeout_s=1.0, clock=lambda: now[0], sleep_fn=sleep_fn))
            self.assertTrue(ctrl.send_rearm_home())
            self.assertTrue(ctrl.wait_for_mega_ready(timeout_s=2.0, clock=lambda: now[0], sleep_fn=sleep_fn))
            self.assertEqual(mega.state, "IDLE")

    def test_mid_session_lcd_pause_parks_cleanly_without_fault_and_missed_pick_not_counted(self):
        with tempfile.TemporaryDirectory() as td:
            hud = hud_mod.CapSorterLCDHUD(fb_device="")
            ctrl = self._make_controller(td, state_callback=hud.update_state)
            mega = SimulatedMega2560()
            now = [0.0]
            sleep_fn = self._wire_mega_uart(ctrl, mega, now)

            world_caps = [
                [90.0, 90.0, pzv.COLOR_YELLOW],
                [210.0, 210.0, pzv.COLOR_BLUE],
            ]

            def pick_and_pause(bx, by, col):
                for i, c in enumerate(world_caps):
                    if c[2] == col and math.hypot(c[0] - bx, c[1] - by) < 10.0:
                        world_caps.pop(i)
                        break
                # Operator taps PAUSE on 3.5" LCD during Cap #1!
                ctrl.paused = True

            mega.on_pick_world = pick_and_pause
            res = ctrl.run_closed_loop_sort_session(
                capture_fn=lambda: _draw_synthetic_board(ctrl.H_inv, [(c[0], c[1], c[2]) for c in world_caps]),
                clock=lambda: now[0],
                sleep_fn=sleep_fn,
            )
            self.assertEqual(res["status"], "PAUSED")
            self.assertEqual(res["picked"], 1)
            self.assertEqual(len(world_caps), 1)
            self.assertEqual(mega.state, "IDLE")
            self.assertAlmostEqual(mega.mach_x, 0.0, delta=0.05)

            # Also verify stuck cap does NOT falsely increment picked or HUD bin count
            ctrl.paused = False
            mega2 = SimulatedMega2560()
            now2 = [0.0]
            sleep_fn2 = self._wire_mega_uart(ctrl, mega2, now2)
            stuck_world = [[140.0, 140.0, pzv.COLOR_RED]]
            mega2.on_pick_world = lambda bx, by, col: None  # Glued down!
            hud2 = hud_mod.CapSorterLCDHUD(fb_device="")
            ctrl.state_callback = hud2.update_state
            res2 = ctrl.run_closed_loop_sort_session(
                capture_fn=lambda: _draw_synthetic_board(ctrl.H_inv, [(c[0], c[1], c[2]) for c in stuck_world]),
                clock=lambda: now2[0],
                sleep_fn=sleep_fn2,
            )
            self.assertEqual(res2["status"], "COMPLETE")
            self.assertEqual(res2["picked"], 0)
            self.assertEqual(len(res2["quarantined"]), 1)
            self.assertEqual(hud2.state["bin_counts"][pzv.COLOR_RED], 0)

    def test_cpp_and_ino_structural_and_safety_checks(self):
        files = [
            "firmware/common/packet_protocol.h",
            "firmware/mega_motion/mega_motion.ino",
            "firmware/v1_teensy41/teensy_vision.cpp",
            "firmware/v1_esp32cam/esp32cam_uart.ino",
        ]
        for rel in files:
            src = _read_text(os.path.join(REPO_ROOT, rel))
            # Strip comments & string literals to verify exact brace/paren balance
            cleaned = re.sub(r"//.*?$|/\*.*?\*/|\"(?:\\.|[^\"\\])*\"|'(?:\\.|[^'\\])*'", "", src, flags=re.S | re.M)
            self.assertEqual(cleaned.count("{"), cleaned.count("}"), f"Unbalanced braces in {rel}")
            self.assertEqual(cleaned.count("("), cleaned.count(")"), f"Unbalanced parens in {rel}")

        mega = _read_text(os.path.join(REPO_ROOT, "firmware/mega_motion/mega_motion.ino"))
        self.assertIn("moveMachineToMm", mega)
        self.assertIn("moveBoardToMm", mega)
        self.assertIn("Wire.setWireTimeout(3000, true);", mega)
        esp = _read_text(os.path.join(REPO_ROOT, "firmware/v1_esp32cam/esp32cam_uart.ino"))
        self.assertIn("yield();", esp)
        teensy = _read_text(os.path.join(REPO_ROOT, "firmware/v1_teensy41/teensy_vision.cpp"))
        self.assertIn("#define SPLIT_ROI_MAX             160", teensy)

    def test_bringup_sketches_and_scripts_compile_and_execute(self):
        import py_compile
        import subprocess

        # 1. Verify all 3 Mega bring-up Arduino sketches have balanced syntax & safety guards
        bringup_inos = [
            "firmware/bringup/mega_01_switches_estop/mega_01_switches_estop.ino",
            "firmware/bringup/mega_02_servo_calibration/mega_02_servo_calibration.ino",
            "firmware/bringup/mega_03_stepper_homing_test/mega_03_stepper_homing_test.ino",
        ]
        for rel in bringup_inos:
            src = _read_text(os.path.join(REPO_ROOT, rel))
            cleaned = re.sub(r"//.*?$|/\*.*?\*/|\"(?:\\.|[^\"\\])*\"|'(?:\\.|[^'\\])*'", "", src, flags=re.S | re.M)
            self.assertEqual(cleaned.count("{"), cleaned.count("}"), f"Unbalanced braces in {rel}")
            self.assertEqual(cleaned.count("("), cleaned.count(")"), f"Unbalanced parens in {rel}")
            self.assertIn("PIN_ESTOP", src)

        # 2. Syntax-compile every Python file in v2_pizero2w and bringup
        py_files = [
            "firmware/v2_pizero2w/pi_zero_vision.py",
            "firmware/v2_pizero2w/lcd_hud.py",
            "firmware/v2_pizero2w/fisheye_calibrate.py",
            "firmware/bringup/pi_01_camera_detect_preview.py",
            "firmware/bringup/pi_02_capture_and_mark_corners.py",
            "firmware/bringup/pi_03_verify_mm_coordinates.py",
            "firmware/bringup/pi_04_uart_link_test.py",
            "firmware/bringup/pi_05_lcd_touch_test.py",
        ]
        for rel in py_files:
            py_compile.compile(os.path.join(REPO_ROOT, rel), doraise=True)

        # 3. Smoke-execute Stage 4, Stage 5, Stage 6, and Stage 7B bring-up tools on synthetic data!
        with tempfile.TemporaryDirectory() as td:
            # Generate 10 synthetic fisheye checkerboards + 1 synthetic 4-cap board image
            cb_dir = os.path.join(td, "cbs")
            os.makedirs(cb_dir)
            _render_synthetic_checkerboards(cb_dir, lens_model="fisheye", count=10)
            H_ref = fc.compute_homography_from_fiducials([[92, 34], [548, 34], [548, 446], [92, 446]])
            board_img = _draw_synthetic_board(np.linalg.inv(H_ref), [
                (70.0, 80.0, pzv.COLOR_RED),
                (101.0, 80.0, pzv.COLOR_RED),
                (210.0, 140.0, pzv.COLOR_YELLOW),
                (120.0, 230.0, pzv.COLOR_GREEN),
                (240.0, 240.0, pzv.COLOR_BLUE),
            ])
            board_png = os.path.join(td, "board.png")
            cv2.imwrite(board_png, board_img)

            # Stage 4: pi_01_camera_detect_preview.py --image --once
            s4_dir = os.path.join(td, "s4")
            r4 = subprocess.run(
                [sys.executable, os.path.join(REPO_ROOT, "firmware/bringup/pi_01_camera_detect_preview.py"),
                 "--image", board_png, "--once", "--out-dir", s4_dir],
                capture_output=True, text=True, check=True,
            )
            self.assertIn("Detected 4 cap region", r4.stdout)  # 2 touching reds = 1 raw cluster + 3 others
            self.assertTrue(os.path.isfile(os.path.join(s4_dir, "stage4_boxes.png")))

            # Stage 5: pi_02_capture_and_mark_corners.py --mode corners -> produces validated .npz!
            s5_dir = os.path.join(td, "s5")
            out_npz = os.path.join(td, "cal_bench.npz")
            subprocess.run(
                [sys.executable, os.path.join(REPO_ROOT, "firmware/bringup/pi_02_capture_and_mark_corners.py"),
                 "--mode", "corners", "--camera-type", "picamera2_imx219", "--lens-model", "fisheye",
                 "--checkerboard-glob", os.path.join(cb_dir, "*.png"),
                 "--image", board_png, "--corners", "92,34;548,34;548,446;92,446",
                 "--camera-height-mm", "350", "--out-dir", s5_dir, "--out", out_npz],
                capture_output=True, text=True, check=True,
            )
            self.assertTrue(os.path.isfile(out_npz))
            self.assertTrue(os.path.isfile(os.path.join(s5_dir, "stage5_rectified_ruler_grid.png")))
            self.assertTrue(os.path.isfile(os.path.join(s5_dir, "stage5_corners_verified.png")))

            # Stage 6: pi_03_verify_mm_coordinates.py --calib --image --already-rectified --once
            s6_dir = os.path.join(td, "s6")
            r6 = subprocess.run(
                [sys.executable, os.path.join(REPO_ROOT, "firmware/bringup/pi_03_verify_mm_coordinates.py"),
                 "--calib", out_npz, "--camera-type", "picamera2_imx219",
                 "--image", board_png, "--already-rectified", "--once", "--out-dir", s6_dir],
                capture_output=True, text=True, check=True,
            )
            self.assertIn("Detected 5 calibrated cap(s)", r6.stdout)  # Touching red pair split into 2!
            self.assertTrue(os.path.isfile(os.path.join(s6_dir, "stage6_mm_verified.png")))

            # Stage 7B: pi_05_lcd_touch_test.py --snapshot
            s7_png = os.path.join(td, "stage7_lcd.png")
            subprocess.run(
                [sys.executable, os.path.join(REPO_ROOT, "firmware/bringup/pi_05_lcd_touch_test.py"),
                 "--snapshot", s7_png],
                capture_output=True, text=True, check=True,
            )
            self.assertTrue(os.path.isfile(s7_png))

        # 3b. Smoke-test Stage 7A (pi_04_uart_link_test.py) packet packing, ping & rearm
        bringup_dir = os.path.join(REPO_ROOT, "firmware", "bringup")
        if bringup_dir not in sys.path:
            sys.path.insert(0, bringup_dir)
        import pi_04_uart_link_test as u4
        pkt = u4.pack_single_target(150.0, 150.0, 1)
        self.assertEqual(len(pkt), 18)
        self.assertEqual(pkt[0], pzv.PROTOCOL_SINGLE_MAGIC)
        self.assertEqual(struct.unpack("<H", pkt[16:18])[0], pzv.crc16_ccitt(pkt[1:16]))
        ping_uart = FakeUART(lambda raw, rx: rx.append(pzv.CMD_STATUS_READY) if raw == bytes([pzv.CMD_QUERY_STATUS]) else None)
        self.assertEqual(u4.run_ping(ping_uart, timeout_s=1.0), 0)
        rearm_uart = FakeUART(lambda raw, rx: rx.append(pzv.CMD_STATUS_READY) if raw == bytes([pzv.CMD_REARM_HOME]) else None)
        self.assertEqual(u4.run_rearm(rearm_uart, timeout_s=1.0), 0)

        # 4. Verify HARDWARE_BRINGUP_GUIDE.md exists and has zero banned claims
        guide = _read_text(os.path.join(REPO_ROOT, "firmware/HARDWARE_BRINGUP_GUIDE.md"))
        self.assertIn("Stage 0", guide)
        self.assertIn("Stage 8", guide)
        self.assertIn("pi_01_camera_detect_preview.py", guide)
        self.assertIn("pi_02_capture_and_mark_corners.py", guide)
        self.assertIn("pi_03_verify_mm_coordinates.py", guide)
        self.assertIn("pi_04_uart_link_test.py", guide)
        self.assertIn("pi_05_lcd_touch_test.py", guide)


class TestHTMLCodeVaultSyncAndDocsAudit(unittest.TestCase):
    def test_html_code_vaults_match_standalone_firmware_verbatim(self):
        checks = [
            ("website/v1.html", "code-teensy",    "firmware/v1_teensy41/teensy_vision.cpp"),
            ("website/v1.html", "code-mega",      "firmware/mega_motion/mega_motion.ino"),
            ("website/v1.html", "code-esp32",     "firmware/v1_esp32cam/esp32cam_uart.ino"),
            ("website/v1.html", "code-protocol",  "firmware/common/packet_protocol.h"),
            ("website/v2.html", "code-pizero",    "firmware/v2_pizero2w/pi_zero_vision.py"),
            ("website/v2.html", "code-lcd",       "firmware/v2_pizero2w/lcd_hud.py"),
            ("website/v2.html", "code-calibrate", "firmware/v2_pizero2w/fisheye_calibrate.py"),
            ("website/v2.html", "code-mega",      "firmware/mega_motion/mega_motion.ino"),
            ("website/v2.html", "code-protocol",  "firmware/common/packet_protocol.h"),
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
            "README.md", "firmware/README.md", "firmware/HARDWARE_BRINGUP_GUIDE.md",
            "website/index.html", "website/v1.html", "website/v2.html",
            "website/pdf_dossier.html", "website/pdf_dossier_v2.html",
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
        v2_dossier = _read_text(os.path.join(REPO_ROOT, "website", "pdf_dossier_v2.html"))
        for v1_leftover in [r"OV2640", r"\bQVGA\b", r"921,600\s*Baud", r"Serial2", r"BGR888"]:
            self.assertIsNone(
                re.search(v1_leftover, v2_dossier, re.I),
                f"V1 leftover ({v1_leftover}) still present in pdf_dossier_v2.html"
            )


class TestRemediationHardening(unittest.TestCase):
    """Dedicated regression suite verifying all 12 audited firmware/safety/tooling remediations."""

    def _make_controller(self, td):
        cal_path = os.path.join(td, "cal.npz")
        H = fc.compute_homography_from_fiducials(
            [[92, 34], [548, 34], [548, 446], [92, 446]], image_size=(640, 480)
        )
        fc.save_validated_calibration(
            cal_path,
            np.array([[320.0, 0, 320.0], [0, 320.0, 240.0], [0, 0, 1.0]]),
            np.zeros((4, 1)), H, "picamera2_imx219", "fisheye", (640, 480), 0.1,
        )
        return pzv.PiZeroVisionController(
            calib_file=cal_path, camera_type="picamera2_imx219", init_hardware=False
        )

    def test_mega_bounded_homing_timed_debounce_z_and_grip_guards(self):
        mega = _read_text(os.path.join(REPO_ROOT, "firmware", "mega_motion", "mega_motion.ino"))
        # Issue #1: Step- & distance-bounded homing seek + derived 23 s timeout
        self.assertIn("#define MAX_HOMING_SEEK_X_MM    (MAX_TRAVEL_X_MM + HOMING_SEEK_MARGIN_MM)", mega)
        self.assertIn("#define MAX_HOMING_SEEK_Y_MM    (MAX_TRAVEL_Y_MM + HOMING_SEEK_MARGIN_MM)", mega)
        self.assertIn("labs(stepperX.currentPosition()) > MAX_HOMING_SEEK_X_STEPS", mega)
        self.assertIn("labs(stepperY.currentPosition()) > MAX_HOMING_SEEK_Y_STEPS", mega)
        self.assertIn("HOMING_TIMEOUT_MS    23000UL", mega)
        # Issue #2: Deterministic millis()-timed persistence debounce
        self.assertIn("#define ESTOP_DEBOUNCE_MS        4UL", mega)
        self.assertIn("#define LIMIT_DEBOUNCE_MS        8UL", mega)
        self.assertIn("nowMs - estopHighSinceMs >= ESTOP_DEBOUNCE_MS", mega)
        self.assertIn("nowMs - limitOpenSinceMs >= LIMIT_DEBOUNCE_MS", mega)
        # Issue #3: Honest Z-safety check naming & mode-specific fault reasons
        self.assertIn("bool isZSafeForXYMotion()", mega)
        self.assertIn("zUnsafeReason()", mega)
        self.assertNotIn("isZRetractedSafeForXY", mega)
        # Issue #4: Active isCapVerifiedHeld() wiring in STATE_GRIP_CAP & STATE_LIFT_Z
        self.assertIn("bool          capPossessionVerified = false;", mega)
        self.assertGreaterEqual(mega.count("isCapVerifiedHeld()"), 3)
        self.assertIn("[GRIP_MISS]", mega)
        self.assertIn("[GRIP_SLIP]", mega)

    def test_stage3_jog_safety_clamps_and_disable_clears_targets(self):
        s3 = _read_text(os.path.join(
            REPO_ROOT, "firmware", "bringup", "mega_03_stepper_homing_test", "mega_03_stepper_homing_test.ino"
        ))
        self.assertIn("bool canAcceptMotionCmd()", s3)
        self.assertIn("#define MAX_UNHOMED_JOG_MM      50.0f", s3)
        self.assertIn("MAX_HOMING_SEEK_X_STEPS", s3)
        self.assertIn("MAX_HOMING_SEEK_Y_STEPS", s3)
        self.assertIn("HOMING_TIMEOUT_MS      23000UL", s3)
        # Ensure dis clears pending targets & disables servo OE, and en syncs targets
        dis_match = re.search(r'else if \(line == "dis"\)\s*\{(.*?)\}\s*else if \(line == "en"\)', s3, re.S)
        self.assertIsNotNone(dis_match)
        dis_body = dis_match.group(1)
        self.assertIn("stepperX.moveTo(stepperX.currentPosition());", dis_body)
        self.assertIn("stepperY.moveTo(stepperY.currentPosition());", dis_body)
        self.assertIn("digitalWrite(PIN_SERVO_OE, HIGH);", dis_body)
        en_match = re.search(r'else if \(line == "en"\)\s*\{(.*?)\}\s*else if \(line == "home"\)', s3, re.S)
        self.assertIsNotNone(en_match)
        self.assertIn("stepperX.moveTo(stepperX.currentPosition());", en_match.group(1))
        # Ensure x, y, go, and servo commands all call canAcceptMotionCmd()
        self.assertGreaterEqual(s3.count("if (!canAcceptMotionCmd()) return;"), 8)

    def test_v1_bfs_overflow_capture_fault_retry_limit_and_rearm(self):
        teensy = _read_text(os.path.join(REPO_ROOT, "firmware", "v1_teensy41", "teensy_vision.cpp"))
        # Issue #6: Expanded 8192 BFS queue + capacity check before marking visited + overflow rejection
        self.assertIn("#define BFS_QUEUE_SIZE            8192", teensy)
        self.assertIn("bool componentOverflow = false;", teensy)
        self.assertIn("if (componentOverflow)", teensy)
        # Ensure visited flag is set inside the else (non-overflow) branch, not before checking nextTail
        bfs_block = teensy[teensy.index("if (nextTail == qHead)"):teensy.index("if (componentOverflow)")]
        self.assertIn("componentOverflow = true;", bfs_block)
        self.assertIn("colorMask[nIdx] |= 0x80;", bfs_block)
        # Issue #7: Distinct EXEC_CAPTURE_FAULT on Scan #0 and clear-FOV verification failure
        self.assertIn("EXEC_CAPTURE_FAULT", teensy)
        self.assertIn("captureAndSegmentWithRetry", teensy)
        self.assertNotIn("if (!captureAndSegmentBoard(false, 0.0f, nullptr)) {\n    return EXEC_COMPLETE;", teensy)
        # Issue #8: Expanded retry tracking table (MAX_BATCH_CAPS = 100) + bounded exhaustion abort
        self.assertIn("#define MAX_RETRY_RECORDS         MAX_BATCH_CAPS", teensy)
        self.assertIn("retryTableExhausted", teensy)
        # Issue #9: Explicit operator re-arm via USB Serial "REARM" + optional Pin 2 button
        self.assertIn("#define ENABLE_V1_REARM_BTN       0", teensy)
        self.assertIn("#define PIN_V1_REARM_BTN          2", teensy)
        self.assertIn("bool sendRearmToMega()", teensy)
        self.assertIn("bool pollOperatorRearmInput()", teensy)
        self.assertIn("Serial1.write(CMD_REARM_HOME);", teensy)

    def test_v1_bfs_overflow_and_retry_table_algorithmic_mirrors(self):
        # Mirror Issue #6: tiny queue triggers overflow flag and drops corrupted blob without false emit
        def bfs_with_cap(mask_2d, queue_cap):
            h, w = mask_2d.shape
            visited = np.zeros_like(mask_2d, dtype=bool)
            q = [0] * queue_cap
            emitted = []
            overflows = 0
            for y in range(h):
                for x in range(w):
                    if not mask_2d[y, x] or visited[y, x]:
                        continue
                    qh = qt = 0
                    visited[y, x] = True
                    q[qt] = (x, y)
                    qt = (qt + 1) % queue_cap
                    overflow = False
                    area = 0
                    while qh != qt:
                        cx, cy = q[qh]
                        qh = (qh + 1) % queue_cap
                        area += 1
                        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                            nx, ny = cx + dx, cy + dy
                            if 0 <= nx < w and 0 <= ny < h and mask_2d[ny, nx] and not visited[ny, nx]:
                                next_t = (qt + 1) % queue_cap
                                if next_t == qh:
                                    overflow = True
                                else:
                                    visited[ny, nx] = True
                                    q[qt] = (nx, ny)
                                    qt = next_t
                    if overflow:
                        overflows += 1
                    else:
                        emitted.append(area)
            return emitted, overflows

        grid = np.zeros((20, 20), dtype=bool)
        grid[2:14, 2:14] = True  # 144-pixel square
        emitted_small, ovf_small = bfs_with_cap(grid, queue_cap=8)
        self.assertGreater(ovf_small, 0)
        self.assertEqual(len(emitted_small), 0, "Overflowed component must be rejected, not silently emitted")
        emitted_ok, ovf_ok = bfs_with_cap(grid, queue_cap=256)
        self.assertEqual(ovf_ok, 0)
        self.assertEqual(emitted_ok, [144])

    def test_v2_guarded_heartbeat_survives_slow_capture_and_halts_on_stall(self):
        import time as _time

        with tempfile.TemporaryDirectory() as td:
            ctrl = self._make_controller(td)
            world_caps = [[120.0, 140.0, pzv.COLOR_RED]]
            board_with_cap = _draw_synthetic_board(ctrl.H_inv, [(120.0, 140.0, pzv.COLOR_RED)])
            board_empty = _draw_synthetic_board(ctrl.H_inv, [])

            # Part A: Slow bin-side capture (0.35 s) with 0.05 s bg heartbeat MUST keep emitting 0xF0!
            capture_calls = [0]
            ping_times = []

            def slow_capture():
                capture_calls[0] += 1
                if capture_calls[0] == 1:
                    return board_with_cap
                _time.sleep(0.35)
                return board_empty

            def mega_script(raw, rx):
                if raw[0] == pzv.PROTOCOL_SINGLE_MAGIC:
                    rx.append(pzv.CMD_BATCH_ACK)
                    rx.append(pzv.CMD_SCAN_WINDOW_OPEN)
                elif raw == bytes([pzv.CMD_HEARTBEAT_PING]):
                    ping_times.append(_time.monotonic())
                    if capture_calls[0] >= 2 and len(ping_times) >= 4:
                        rx.append(pzv.CMD_CAP_PROGRESS)
                        rx.append(pzv.CMD_AWAIT_NEXT)
                elif raw == bytes([pzv.CMD_PARK_FOR_SCAN]):
                    rx.append(pzv.CMD_AWAIT_NEXT)
                elif raw == bytes([pzv.CMD_FINISH_SESSION]):
                    rx.append(pzv.CMD_BATCH_COMPLETE)

            ctrl.uart = FakeUART(mega_script)
            res = ctrl.run_closed_loop_sort_session(
                capture_fn=slow_capture,
                enable_bg_heartbeat=True,
                heartbeat_interval_s=0.05,
                max_vision_stall_s=1.5,
            )
            self.assertEqual(res["status"], "COMPLETE")
            self.assertGreaterEqual(res["bg_heartbeats_sent"], 4)
            self.assertGreater(res["max_vision_ms"], 300.0)

            # Part B: True vision deadlock (> max_vision_stall_s) MUST stop heartbeating & return VISION_STALL!
            stall_calls = [0]
            stall_pings = []

            def deadlocked_capture():
                stall_calls[0] += 1
                if stall_calls[0] == 1:
                    return board_with_cap
                _time.sleep(0.55)  # Exceeds max_vision_stall_s=0.18 s
                return board_empty

            def stall_mega(raw, rx):
                if raw[0] == pzv.PROTOCOL_SINGLE_MAGIC:
                    rx.append(pzv.CMD_BATCH_ACK)
                    rx.append(pzv.CMD_SCAN_WINDOW_OPEN)
                elif raw == bytes([pzv.CMD_HEARTBEAT_PING]):
                    stall_pings.append(_time.monotonic())

            ctrl.uart = FakeUART(stall_mega)
            res_stall = ctrl.run_closed_loop_sort_session(
                capture_fn=deadlocked_capture,
                enable_bg_heartbeat=True,
                heartbeat_interval_s=0.04,
                max_vision_stall_s=0.18,
            )
            self.assertEqual(res_stall["status"], "VISION_STALL")
            # Heartbeats must have stopped once the 0.18 s liveness budget expired (not ran for full 0.55 s)
            self.assertLess(len(stall_pings), 8)

    def test_lcd_framebuffer_stride_resize_and_channel_packing(self):
        # Construct solid pure-red BGR image (B=0, G=0, R=255) at 480x320
        red_bgr = np.zeros((320, 480, 3), dtype=np.uint8)
        red_bgr[:, :, 2] = 255

        # 1. Padded 16-bpp RGB565 stride (line_length = 480*2 + 64 = 1024 B)
        fb_rgb565_pad = hud_mod.FramebufferInfo(
            width=480, height=320, bpp=16, line_length=1024,
            red_offset=11, red_len=5, green_offset=5, green_len=6, blue_offset=0, blue_len=5,
        )
        raw_pad = hud_mod.pack_bgr_for_framebuffer(red_bgr, fb_rgb565_pad)
        self.assertEqual(len(raw_pad), 320 * 1024)
        # First pixel = 0xF800 (little-endian b"\x00\xf8"); trailing 64 pad bytes are zero
        self.assertEqual(raw_pad[0:2], b"\x00\xf8")
        self.assertEqual(raw_pad[960:1024], b"\x00" * 64)

        # 2. BGR565 channel order (red_offset=0, blue_offset=11) + non-default resize (320x240)
        fb_bgr565 = hud_mod.FramebufferInfo(
            width=320, height=240, bpp=16, line_length=640,
            red_offset=0, red_len=5, green_offset=5, green_len=6, blue_offset=11, blue_len=5,
        )
        raw_bgr565 = hud_mod.pack_bgr_for_framebuffer(red_bgr, fb_bgr565)
        self.assertEqual(len(raw_bgr565), 240 * 640)
        self.assertEqual(raw_bgr565[0:2], b"\x1f\x00")  # Red in low 5 bits = 0x001F

        # 3. 32-bpp RGBA8888 (red_offset=0, green_offset=8, blue_offset=16) with padded stride
        fb_rgba32 = hud_mod.FramebufferInfo(
            width=480, height=320, bpp=32, line_length=2048,
            red_offset=0, red_len=8, green_offset=8, green_len=8, blue_offset=16, blue_len=8,
        )
        raw_rgba = hud_mod.pack_bgr_for_framebuffer(red_bgr, fb_rgba32)
        self.assertEqual(len(raw_rgba), 320 * 2048)
        self.assertEqual(raw_rgba[0:4], b"\xff\x00\x00\xff")  # R=255, G=0, B=0, A=255

        # 4. Verify _blit_framebuffer writes exact stride-padded bytes to a target file
        with tempfile.TemporaryDirectory() as td:
            fb_file = os.path.join(td, "mock_fb0")
            hud = hud_mod.CapSorterLCDHUD(fb_device=fb_file)
            hud._fb_info = fb_rgb565_pad
            hud._blit_framebuffer(red_bgr)
            with open(fb_file, "rb") as fh:
                written = fh.read()
            self.assertEqual(len(written), 320 * 1024)
            self.assertEqual(written[0:2], b"\x00\xf8")

    def test_homography_rejects_bowtie_mirrored_rotated_oob_sliver_and_bad_interior_points(self):
        valid = [[92, 34], [548, 34], [548, 446], [92, 446]]
        H = fc.compute_homography_from_fiducials(
            valid,
            image_size=(640, 480),
            check_points_px=[[320.0, 240.0]],
            check_points_mm=[[150.0, 150.0]],
            max_check_err_mm=1.0,
        )
        self.assertEqual(H.shape, (3, 3))

        # 1. Bow-tie / self-intersecting (swapped BR & BL)
        with self.assertRaises(ValueError):
            fc.compute_homography_from_fiducials([[92, 34], [548, 34], [92, 446], [548, 446]], image_size=(640, 480))
        # 2. Counter-clockwise / mirrored winding (TL -> BL -> BR -> TR)
        with self.assertRaises(ValueError):
            fc.compute_homography_from_fiducials([[92, 34], [92, 446], [548, 446], [548, 34]], image_size=(640, 480))
        # 3. 180-degree rotated corner sequence (BR -> BL -> TL -> TR)
        with self.assertRaises(ValueError):
            fc.compute_homography_from_fiducials([[548, 446], [92, 446], [92, 34], [548, 34]], image_size=(640, 480))
        # 4. Out-of-bounds pixel coordinate
        with self.assertRaises(ValueError):
            fc.compute_homography_from_fiducials([[-5, 34], [548, 34], [548, 446], [92, 446]], image_size=(640, 480))
        # 5. Degenerate narrow sliver (extreme angle / small height)
        with self.assertRaises(ValueError):
            fc.compute_homography_from_fiducials([[92, 200], [548, 200], [548, 215], [92, 215]], image_size=(640, 480))
        # 6. Bad interior check point exceeds tolerance
        with self.assertRaises(RuntimeError):
            fc.compute_homography_from_fiducials(
                valid,
                image_size=(640, 480),
                check_points_px=[[320.0, 240.0]],
                check_points_mm=[[190.0, 150.0]],
                max_check_err_mm=2.5,
            )

    def test_v2_poster_and_remediation_plan_verified(self):
        poster_path = os.path.join(REPO_ROOT, "website", "assets", "system_poster_v2.jpg")
        img = cv2.imread(poster_path)
        self.assertIsNotNone(img, "Regenerated website/assets/system_poster_v2.jpg must be readable")
        h, w = img.shape[:2]
        self.assertGreaterEqual(w, 2000)
        self.assertGreaterEqual(h, 1200)

        plan_path = os.path.join(REPO_ROOT, "firmware", "REMEDIATION_PLAN.md")
        plan_txt = _read_text(plan_path)
        self.assertIn("MAX_HOMING_SEEK_X_MM", plan_txt)
        self.assertIn("_GuardedHeartbeatSupervisor", plan_txt)
        self.assertIn("FBIOGET_VSCREENINFO", plan_txt)
        self.assertIn("EXEC_CAPTURE_FAULT", plan_txt)


if __name__ == "__main__":
    unittest.main()
