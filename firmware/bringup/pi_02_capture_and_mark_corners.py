#!/usr/bin/env python3
# =========================================================================
# Stage 5 Bench Test: Checkerboard Capture & 4-Corner Coordinate Marker
# Purpose:
#   Step 5A (--mode checkerboard):
#     Captures 12-15 sharp checkerboard photos while verifying OpenCV can
#     detect all inner corners (default 9x6).
#   Step 5B (--mode corners):
#     1. Calibrates lens intrinsics (K, D) from your Step 5A photos.
#     2. Captures ONE undistorted/rectified photo of the empty 300x300 mm
#        plywood board and overlays a 20 px numbered ruler grid so you can
#        read or click the 4 corners: Top-Left (0,0), Top-Right (300,0),
#        Bottom-Right (300,300), Bottom-Left (0,300).
#     3. Computes homography H, draws the verified 300x300 mm grid + safe
#        pick box (18..282 mm), and saves your validated .npz calibration!
#
# Usage Examples:
#   # Step 5A: Capture 12 checkerboard photos automatically every 2.5 s
#   python3 pi_02_capture_and_mark_corners.py --mode checkerboard \
#       --camera-type picamera2_imx219 --out-dir ./calib_imgs --count 12 --auto-interval 2.5
#
#   # Step 5B (Step 1 - generate rectified ruler image to read 4 corner pixels):
#   python3 pi_02_capture_and_mark_corners.py --mode corners \
#       --camera-type picamera2_imx219 --lens-model fisheye \
#       --checkerboard-glob "./calib_imgs/*.png" --out-dir ./stage5_out
#
#   # Step 5B (Step 2 - build final validated .npz from your 4 measured corners):
#   python3 pi_02_capture_and_mark_corners.py --mode corners \
#       --camera-type picamera2_imx219 --lens-model fisheye \
#       --checkerboard-glob "./calib_imgs/*.png" --out-dir ./stage5_out \
#       --corners "92,34;548,34;548,446;92,446" \
#       --camera-height-mm 350 --cap-height-mm 13 \
#       --out /home/pi/capsorter/calibration_csi_imx219.npz
# =========================================================================
import argparse
import glob
import json
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

import fisheye_calibrate as fc
import pi_zero_vision as pzv
from pi_01_camera_detect_preview import CameraSource, start_mjpeg_server


def draw_pixel_ruler_grid(img_bgr: np.ndarray, step_px: int = 20) -> np.ndarray:
    """Draws a crisp numbered pixel coordinate grid so headless users can read (u, v) accurately."""
    vis = img_bgr.copy()
    h, w = vis.shape[:2]
    for u in range(0, w, step_px):
        major = (u % 100 == 0)
        col = (0, 220, 255) if major else (90, 90, 90)
        cv2.line(vis, (u, 0), (u, h), col, 1)
        if u % 40 == 0:
            cv2.putText(vis, str(u), (u + 2, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(vis, str(u), (u + 2, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 255, 255), 1, cv2.LINE_AA)
    for v in range(0, h, step_px):
        major = (v % 100 == 0)
        col = (0, 220, 255) if major else (90, 90, 90)
        cv2.line(vis, (0, v), (w, v), col, 1)
        if v % 40 == 0 and v > 0:
            cv2.putText(vis, str(v), (3, v - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(vis, str(v), (3, v - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 255, 255), 1, cv2.LINE_AA)
    return vis


def draw_verified_board_overlay(rect_bgr: np.ndarray, H: np.ndarray) -> np.ndarray:
    """Projects the 300x300 mm board perimeter, 30 mm grid, and 18..282 mm safe box back onto image."""
    vis = rect_bgr.copy()
    H_inv = np.linalg.inv(H)

    def mm_to_px(x_mm, y_mm):
        pt = cv2.perspectiveTransform(np.array([[[float(x_mm), float(y_mm)]]], dtype=np.float32), H_inv)[0][0]
        return (int(round(pt[0])), int(round(pt[1])))

    # 30 mm internal grid
    for mm in range(30, 300, 30):
        cv2.line(vis, mm_to_px(mm, 0), mm_to_px(mm, 300), (80, 140, 80), 1)
        cv2.line(vis, mm_to_px(0, mm), mm_to_px(300, mm), (80, 140, 80), 1)

    # Outer 300x300 mm plywood boundary
    board_poly = np.array([mm_to_px(0, 0), mm_to_px(300, 0), mm_to_px(300, 300), mm_to_px(0, 300)], dtype=np.int32)
    cv2.polylines(vis, [board_poly], True, (0, 255, 0), 2)

    # Safe mechanical reach box (18..282 mm)
    safe_poly = np.array([
        mm_to_px(pzv.SAFE_PICK_MIN_X_MM, pzv.SAFE_PICK_MIN_Y_MM),
        mm_to_px(pzv.SAFE_PICK_MAX_X_MM, pzv.SAFE_PICK_MIN_Y_MM),
        mm_to_px(pzv.SAFE_PICK_MAX_X_MM, pzv.SAFE_PICK_MAX_Y_MM),
        mm_to_px(pzv.SAFE_PICK_MIN_X_MM, pzv.SAFE_PICK_MAX_Y_MM),
    ], dtype=np.int32)
    cv2.polylines(vis, [safe_poly], True, (255, 220, 0), 2)

    labels = [("TL (0,0)", (0, 0)), ("TR (300,0)", (300, 0)),
              ("BR (300,300)", (300, 300)), ("BL (0,300)", (0, 300))]
    for text, (xm, ym) in labels:
        px, py = mm_to_px(xm, ym)
        cv2.circle(vis, (px, py), 6, (0, 0, 255), -1)
        cv2.putText(vis, text, (px - 28, max(16, py - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)
    return vis


def run_checkerboard_capture(args):
    os.makedirs(args.out_dir, exist_ok=True)
    cols, rows = (int(x) for x in args.pattern.lower().split("x"))
    cam = CameraSource(args.camera_type, args.width, args.height, args.image)
    frame_holder = {"jpg": None}
    srv = start_mjpeg_server(args.web_port, frame_holder) if args.web_port > 0 else None
    if srv:
        print(f"[WEB] Checkerboard preview at http://<pi-ip>:{args.web_port}/")

    saved = 0
    last_save = 0.0
    print(f"[STEP 5A] Looking for {cols}x{rows} inner checkerboard corners (target: {args.count} photos).")
    print("          Move & tilt the checkerboard across center, corners, and edges of the board.")
    try:
        while saved < args.count:
            raw = cam.read()
            gray = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(
                gray, (cols, rows),
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
            vis = raw.copy()
            if found:
                cv2.drawChessboardCorners(vis, (cols, rows), corners, found)
                status = f"DETECTED! Saved {saved}/{args.count}"
                col = (0, 255, 0)
            else:
                status = f"No {cols}x{rows} pattern seen | Saved {saved}/{args.count}"
                col = (0, 0, 255)
            cv2.rectangle(vis, (0, 0), (vis.shape[1], 28), (15, 15, 15), -1)
            cv2.putText(vis, status, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.52, col, 1, cv2.LINE_AA)

            if args.web_port > 0:
                ok, enc = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if ok:
                    frame_holder["jpg"] = enc.tobytes()

            should_save = False
            if args.show:
                cv2.imshow("Stage 5A - Checkerboard (SPACE to save, Q to quit)", vis)
                key = cv2.waitKey(30) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord(" ") and found:
                    should_save = True
            if found and args.auto_interval > 0 and (time.monotonic() - last_save >= args.auto_interval):
                should_save = True

            if should_save:
                out_path = os.path.join(args.out_dir, f"cb_{saved:02d}.png")
                cv2.imwrite(out_path, raw)
                saved += 1
                last_save = time.monotonic()
                print(f"[CAPTURED {saved:02d}/{args.count}] -> {out_path}")
                if args.image:
                    break
            elif not args.show:
                time.sleep(0.15)
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()
        if srv:
            srv.shutdown()
        if args.show:
            cv2.destroyAllWindows()
    print(f"[DONE] Saved {saved} valid checkerboard image(s) in {args.out_dir}")


def interactive_click_corners(rect_bgr: np.ndarray) -> np.ndarray:
    """Opens an OpenCV window so user clicks TL -> TR -> BR -> BL board corners."""
    clicked = []
    names = ["1: Top-Left (0,0)", "2: Top-Right (300,0)", "3: Bottom-Right (300,300)", "4: Bottom-Left (0,300)"]

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(clicked) < 4:
            clicked.append([float(x), float(y)])
            print(f"[CLICK] {names[len(clicked) - 1]} -> ({x}, {y})")

    win = "Click 4 Board Corners: TL -> TR -> BR -> BL (R=reset, Q=cancel)"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    while len(clicked) < 4:
        vis = rect_bgr.copy()
        prompt = f"Click {names[len(clicked)]}"
        cv2.rectangle(vis, (0, 0), (vis.shape[1], 28), (15, 15, 15), -1)
        cv2.putText(vis, prompt, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
        for idx, (cx, cy) in enumerate(clicked):
            cv2.circle(vis, (int(cx), int(cy)), 6, (0, 255, 0), -1)
            cv2.putText(vis, str(idx + 1), (int(cx) + 8, int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow(win, vis)
        key = cv2.waitKey(30) & 0xFF
        if key == ord("r"):
            clicked.clear()
        elif key in (ord("q"), 27):
            cv2.destroyAllWindows()
            raise RuntimeError("Corner selection cancelled by user.")
    cv2.destroyAllWindows()
    return np.array(clicked, dtype=np.float32)


def run_corners_and_calibration(args):
    os.makedirs(args.out_dir, exist_ok=True)
    cols, rows = (int(x) for x in args.pattern.lower().split("x"))
    img_paths = sorted(glob.glob(args.checkerboard_glob))
    print(f"[STEP 5B] Calibrating {args.lens_model} intrinsics from {len(img_paths)} image(s)...")
    K, D, image_size, rms = fc.calibrate_intrinsics_from_checkerboards(
        image_paths=img_paths,
        pattern_size=(cols, rows),
        square_size_mm=args.square_mm,
        lens_model=args.lens_model,
        min_images=args.min_images,
        max_reproj_err=args.max_reproj_err,
    )
    print(f"[OK] Intrinsics solved: size={image_size}, RMS reprojection error = {rms:.4f} px")

    # Build undistortion maps & rectify a fresh board image
    if args.lens_model == "fisheye":
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            K, D, np.eye(3, dtype=np.float64), K, image_size, cv2.CV_16SC2
        )
    else:
        map1, map2 = cv2.initUndistortRectifyMap(K, D, None, K, image_size, cv2.CV_16SC2)

    cam = CameraSource(args.camera_type, image_size[0], image_size[1], args.image)
    try:
        raw_board = cam.read()
    finally:
        cam.close()

    rect_board = cv2.remap(raw_board, map1, map2, interpolation=cv2.INTER_LINEAR)
    rect_path = os.path.join(args.out_dir, "stage5_rectified_board.png")
    grid_path = os.path.join(args.out_dir, "stage5_rectified_ruler_grid.png")
    cv2.imwrite(rect_path, rect_board)
    cv2.imwrite(grid_path, draw_pixel_ruler_grid(rect_board, step_px=20))
    print(f"[SAVED] Rectified board      -> {rect_path}")
    print(f"[SAVED] Pixel ruler overlay  -> {grid_path}")

    corners_px = None
    if args.corners:
        corners_px = fc._parse_corners_arg(args.corners)
    elif args.corners_json:
        with open(args.corners_json, "r", encoding="utf-8") as fh:
            corners_px = np.array(json.load(fh), dtype=np.float32)
    elif args.show:
        corners_px = interactive_click_corners(rect_board)
    else:
        print("\n[NEXT STEP] Open 'stage5_rectified_ruler_grid.png' and read the (u, v) pixel coordinates")
        print("            of your 4 board corners in order: Top-Left; Top-Right; Bottom-Right; Bottom-Left.")
        print("            Then re-run this exact command adding:")
        print('              --corners "uTL,vTL;uTR,vTR;uBR,vBR;uBL,vBL" --camera-height-mm <H> --out <file.npz>')
        return

    H = fc.compute_homography_from_fiducials(corners_px, image_size=image_size)
    corners_json_path = os.path.join(args.out_dir, "corners_px.json")
    with open(corners_json_path, "w", encoding="utf-8") as fh:
        json.dump(corners_px.tolist(), fh, indent=2)

    verified_img = draw_verified_board_overlay(rect_board, H)
    verified_path = os.path.join(args.out_dir, "stage5_corners_verified.png")
    cv2.imwrite(verified_path, verified_img)
    print(f"[SAVED] Corner & 300x300 mm verification overlay -> {verified_path}")

    default_out = (
        "calibration_csi_imx219.npz"
        if args.camera_type == "picamera2_imx219"
        else "calibration_usb.npz"
    )
    out_npz = args.out or os.path.join(args.out_dir, default_out)
    fc.save_validated_calibration(
        out_path=out_npz,
        K=K, D=D, H=H,
        camera_type=args.camera_type,
        lens_model=args.lens_model,
        image_size=image_size,
        reprojection_error_px=rms,
        camera_height_mm=args.camera_height_mm,
        cap_height_mm=args.cap_height_mm,
    )


def main():
    parser = argparse.ArgumentParser(description="Stage 5: Checkerboard Capture & 4-Corner Coordinate Marker")
    parser.add_argument("--mode", required=True, choices=("checkerboard", "corners"))
    parser.add_argument("--camera-type", choices=("picamera2_imx219", "usb_v4l2"),
                        default="picamera2_imx219" if pzv.HAS_PICAMERA2 else "usb_v4l2")
    parser.add_argument("--lens-model", choices=("fisheye", "pinhole"), default="fisheye")
    parser.add_argument("--image", help="Optional static image path for offline testing")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--out-dir", default="stage5_calib")
    parser.add_argument("--pattern", default="9x6", help="Inner checkerboard corners COLSxROWS (default: 9x6)")
    parser.add_argument("--square-mm", type=float, default=25.0)
    parser.add_argument("--count", type=int, default=12, help="Checkerboard images to capture in Step 5A")
    parser.add_argument("--auto-interval", type=float, default=2.5,
                        help="Auto-save interval (s) whenever pattern is detected (0=manual SPACE)")
    parser.add_argument("--checkerboard-glob", default="stage5_calib/cb_*.png")
    parser.add_argument("--min-images", type=int, default=8)
    parser.add_argument("--max-reproj-err", type=float, default=1.0)
    parser.add_argument("--corners", help='4 rectified corners "uTL,vTL;uTR,vTR;uBR,vBR;uBL,vBL"')
    parser.add_argument("--corners-json", help="JSON file with 4x2 corner pixels")
    parser.add_argument("--camera-height-mm", type=float, default=0.0,
                        help="Measured lens height above plywood plate in mm for radial parallax compensation")
    parser.add_argument("--cap-height-mm", type=float, default=13.0)
    parser.add_argument("--out", help="Output validated .npz path")
    parser.add_argument("--web-port", type=int, default=0)
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    if args.mode == "checkerboard":
        run_checkerboard_capture(args)
    else:
        run_corners_and_calibration(args)


if __name__ == "__main__":
    main()
