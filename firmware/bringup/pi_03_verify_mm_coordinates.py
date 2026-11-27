#!/usr/bin/env python3
# =========================================================================
# Stage 6 Bench Test: Millimeter Coordinate, Parallax & Clearance Verifier
# Purpose:
#   Loads your validated Stage 5 .npz calibration through the EXACT
#   PiZeroVisionController pipeline (with ZERO motor motion!) and overlays:
#     - 300x300 mm board perimeter + 30 mm reference grid
#     - Cyan safe-pick zone (18..282 mm)
#     - Bounding square + circle around every detected cap (including split
#       touching same-color caps!)
#     - Exact (X, Y) mm coordinates, color -> bin, jaw clearance tier (BLK),
#       and pick order (#1, #2, ...)
#     - Open X-axis gripper jaw footprint (+/-18..26 mm) around pick #1
#     - Optional bridge occlusion band preview (--preview-occlusion-bin 1..4)
#
# Usage Examples:
#   # Capture once & print mm table to compare against a physical ruler:
#   python3 pi_03_verify_mm_coordinates.py --calib calibration_csi_imx219.npz \
#       --camera-type picamera2_imx219 --once
#
#   # Live browser stream while you slide caps around the board by hand:
#   python3 pi_03_verify_mm_coordinates.py --calib calibration_csi_imx219.npz \
#       --camera-type picamera2_imx219 --web-port 8080
# =========================================================================
import argparse
import os
import sys
import time

import cv2
import numpy as np

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
V2_DIR = os.path.abspath(os.path.join(THIS_DIR, "..", "v2_pizero2w"))
for p in (THIS_DIR, V2_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)

import pi_zero_vision as pzv
from pi_01_camera_detect_preview import (
    COLOR_BGR,
    COLOR_NAMES,
    CameraSource,
    blit_to_lcd,
    start_mjpeg_server,
)


def annotate_mm_frame(
    ctrl: pzv.PiZeroVisionController,
    rect_bgr: np.ndarray,
    caps: list,
    occluded_y_mm: float = None,
) -> tuple:
    """Draws board geometry, cap boxes, (X,Y) mm coordinates, and pick-order plan."""
    vis = rect_bgr.copy()

    def mm_to_px(x_mm, y_mm):
        pt = cv2.perspectiveTransform(np.array([[[float(x_mm), float(y_mm)]]], dtype=np.float32), ctrl.H_inv)[0][0]
        return (int(round(pt[0])), int(round(pt[1])))

    # Board outline & 30 mm grid
    for mm in range(30, 300, 30):
        cv2.line(vis, mm_to_px(mm, 0), mm_to_px(mm, 300), (60, 95, 60), 1)
        cv2.line(vis, mm_to_px(0, mm), mm_to_px(300, mm), (60, 95, 60), 1)
    board_poly = np.array([mm_to_px(0, 0), mm_to_px(300, 0), mm_to_px(300, 300), mm_to_px(0, 300)], dtype=np.int32)
    cv2.polylines(vis, [board_poly], True, (0, 230, 0), 2)
    safe_poly = np.array([
        mm_to_px(pzv.SAFE_PICK_MIN_X_MM, pzv.SAFE_PICK_MIN_Y_MM),
        mm_to_px(pzv.SAFE_PICK_MAX_X_MM, pzv.SAFE_PICK_MIN_Y_MM),
        mm_to_px(pzv.SAFE_PICK_MAX_X_MM, pzv.SAFE_PICK_MAX_Y_MM),
        mm_to_px(pzv.SAFE_PICK_MIN_X_MM, pzv.SAFE_PICK_MAX_Y_MM),
    ], dtype=np.int32)
    cv2.polylines(vis, [safe_poly], True, (255, 220, 0), 1)

    # Optional bridge-occlusion band visualization
    if occluded_y_mm is not None and ctrl.bridge_occlusion_half_mm > 0:
        y0 = max(0.0, occluded_y_mm - ctrl.bridge_occlusion_half_mm)
        y1 = min(300.0, occluded_y_mm + ctrl.bridge_occlusion_half_mm)
        band = np.array([mm_to_px(0, y0), mm_to_px(300, y0), mm_to_px(300, y1), mm_to_px(0, y1)], dtype=np.int32)
        overlay = vis.copy()
        cv2.fillPoly(overlay, [band], (20, 90, 180))
        cv2.addWeighted(overlay, 0.35, vis, 0.65, 0.0, vis)

    # Simulate clearance-first pick order from (0,0)
    plan_order = {}
    remaining = [dict(c) for c in caps]
    cur_xy = (0.0, 0.0)
    step_num = 1
    first_pick = None
    while True:
        nxt = ctrl.select_next_cap(remaining, cur_xy=cur_xy)
        if nxt is None:
            break
        plan_order[nxt["capId"]] = (step_num, nxt["blocked_sides"])
        if first_pick is None:
            first_pick = nxt
        remaining = [c for c in remaining if c["capId"] != nxt["capId"]]
        bx, by, _ = pzv.BIN_COORDS[nxt["color"]]
        cur_xy = (bx, by)
        step_num += 1

    rad_px = max(12, int(round(ctrl.expected_cap_radius_px)))
    for cap in caps:
        u, v = cap["pixelX"], cap["pixelY"]
        col = COLOR_BGR[cap["color"]]
        safe = cap.get("in_safe_bounds", True)
        order_info = plan_order.get(cap["capId"])

        # Bounding square + circle + centroid
        box_col = col if safe else (0, 0, 255)
        cv2.rectangle(vis, (u - rad_px, v - rad_px), (u + rad_px, v + rad_px), box_col, 2)
        cv2.circle(vis, (u, v), rad_px, (255, 255, 255) if safe else (0, 0, 255), 1)
        cv2.drawMarker(vis, (u, v), (255, 255, 255), cv2.MARKER_CROSS, 10, 1)

        rank_str = f"#{order_info[0]} BLK:{order_info[1]}" if order_info else "OUT-OF-SAFE"
        lbl1 = f"{COLOR_NAMES[cap['color']][0]}->B{cap['targetBinId']} {rank_str}"
        lbl2 = f"({cap['boardX_mm']:.1f},{cap['boardY_mm']:.1f})mm"
        cv2.putText(vis, lbl1, (u - rad_px, max(16, v - rad_px - 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(vis, lbl1, (u - rad_px, max(16, v - rad_px - 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, box_col, 1, cv2.LINE_AA)
        cv2.putText(vis, lbl2, (u - rad_px, max(28, v - rad_px - 2)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(vis, lbl2, (u - rad_px, max(28, v - rad_px - 2)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)

    # Draw open X-jaw descent rectangles around Pick #1
    if first_pick is not None:
        tx, ty = first_pick["boardX_mm"], first_pick["boardY_mm"]
        for x0, x1 in [
            (tx - pzv.JAW_OUTER_HALF_MM, tx - pzv.JAW_INNER_HALF_MM),
            (tx + pzv.JAW_INNER_HALF_MM, tx + pzv.JAW_OUTER_HALF_MM),
        ]:
            jaw_poly = np.array([
                mm_to_px(x0, ty - pzv.JAW_WIDTH_HALF_MM),
                mm_to_px(x1, ty - pzv.JAW_WIDTH_HALF_MM),
                mm_to_px(x1, ty + pzv.JAW_WIDTH_HALF_MM),
                mm_to_px(x0, ty + pzv.JAW_WIDTH_HALF_MM),
            ], dtype=np.int32)
            cv2.polylines(vis, [jaw_poly], True, (255, 255, 0), 2)

    cv2.rectangle(vis, (0, 0), (vis.shape[1], 26), (18, 18, 18), -1)
    hdr = (
        f"STAGE 6 MM VERIFY | Caps:{len(caps)} | Parallax H={ctrl.camera_height_mm:.0f}mm "
        f"| Jaw={ctrl.jaw_axis} | Safe:18..282mm"
    )
    cv2.putText(vis, hdr, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (240, 240, 240), 1, cv2.LINE_AA)
    return vis, plan_order


def main():
    parser = argparse.ArgumentParser(description="Stage 6: Calibrated (X, Y) mm Coordinate & Clearance Verifier")
    parser.add_argument("--calib", required=True, help="Path to validated Stage 5 .npz calibration file")
    parser.add_argument("--camera-type", choices=("picamera2_imx219", "usb_v4l2"),
                        default="picamera2_imx219" if pzv.HAS_PICAMERA2 else "usb_v4l2")
    parser.add_argument("--image", help="Optional rectified or raw PNG for offline testing")
    parser.add_argument("--already-rectified", action="store_true",
                        help="Skip undistort_frame() when --image is already rectified")
    parser.add_argument("--preview-occlusion-bin", type=int, choices=(1, 2, 3, 4), default=0,
                        help="Optionally preview the +/-35 mm bridge occlusion band for Bin 1..4")
    parser.add_argument("--out-dir", default="stage6_verify")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--web-port", type=int, default=0)
    parser.add_argument("--lcd", default="")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    ctrl = pzv.PiZeroVisionController(
        calib_file=args.calib,
        camera_type=args.camera_type,
        init_hardware=False,
    )
    cam = CameraSource(args.camera_type, ctrl.width, ctrl.height, args.image)
    occluded_y = pzv.BIN_COORDS[args.preview_occlusion_bin][1] if args.preview_occlusion_bin else None

    frame_holder = {"jpg": None}
    srv = start_mjpeg_server(args.web_port, frame_holder) if args.web_port > 0 else None
    if srv:
        print(f"[WEB] Live mm verifier at http://<pi-ip>:{args.web_port}/  (Ctrl+C to stop)")

    try:
        while True:
            raw = cam.read()
            rect = raw if (args.image and args.already_rectified) else ctrl.undistort_frame(raw)
            caps = ctrl.detect_caps(rect, occluded_y_mm=occluded_y)
            vis, order = annotate_mm_frame(ctrl, rect, caps, occluded_y_mm=occluded_y)

            if args.once or (args.web_port == 0 and not args.show and not args.lcd):
                out_path = os.path.join(args.out_dir, "stage6_mm_verified.png")
                cv2.imwrite(out_path, vis)
                print(f"[SAVED] Calibrated mm verification image -> {out_path}")
                print(f"\nDetected {len(caps)} calibrated cap(s) [px/mm={ctrl.px_per_mm:.2f}]:")
                print(f"{'PICK':>4}  {'COLOR':<7} {'BIN':>3}  {'BOARD X (mm)':>12}  {'BOARD Y (mm)':>12}  {'SAFE':<5}  {'JAW_BLK'}")
                for c in caps:
                    info = order.get(c["capId"])
                    pick_s = f"#{info[0]}" if info else "SKIP"
                    blk_s = str(info[1]) if info else "-"
                    print(
                        f"{pick_s:>4}  {COLOR_NAMES[c['color']]:<7} B{c['targetBinId']:<2}  "
                        f"{c['boardX_mm']:>12.2f}  {c['boardY_mm']:>12.2f}  "
                        f"{'YES' if c['in_safe_bounds'] else 'NO':<5}  {blk_s}"
                    )
                break

            if args.web_port > 0:
                ok, enc = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if ok:
                    frame_holder["jpg"] = enc.tobytes()
            if args.lcd:
                blit_to_lcd(args.lcd, vis)
            if args.show:
                cv2.imshow("CapSorter Stage 6 - Calibrated (X, Y) mm (Q to quit)", vis)
                if (cv2.waitKey(30) & 0xFF) in (ord("q"), 27):
                    break
            else:
                time.sleep(0.12)
    except KeyboardInterrupt:
        print("\n[EXIT] Stopped Stage 6 verifier.")
    finally:
        cam.close()
        if srv:
            srv.shutdown()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
