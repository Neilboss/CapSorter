#!/usr/bin/env python3
# =========================================================================
# fisheye_calibrate.py - Camera Intrinsic & 4-Corner Homography Utility (V2)
# Status: Pre-Validation Engineering Utility
#
# Calibrates EACH exact camera, lens, resolution, and mounting setup:
#   - 8MP IMX219 160-deg CSI camera: --camera-type picamera2_imx219 --lens-model fisheye
#   - Standard USB webcam:           --camera-type usb_v4l2 --lens-model pinhole
#
# Derives real K and D from checkerboard images (no hard-coded defaults),
# validates RMS reprojection error, computes 3x3 board homography H from
# measured rectified fiducial corners, and saves validated metadata to .npz.
# =========================================================================
import argparse
import datetime
import glob
import json
import os
import cv2
import numpy as np

VALID_CAMERA_TYPES = ("picamera2_imx219", "usb_v4l2")
VALID_LENS_MODELS = ("fisheye", "pinhole")

# Physical 300x300 mm Sorting Board Fiducial Corners (in millimeters)
BOARD_CORNERS_MM = np.array([
    [0.0,   0.0],     # Top-Left Marker (Origin)
    [300.0, 0.0],     # Top-Right Marker
    [300.0, 300.0],   # Bottom-Right Marker
    [0.0,   300.0],   # Bottom-Left Marker
], dtype=np.float32)


def calibrate_intrinsics_from_checkerboards(
    image_paths: list,
    pattern_size: tuple = (9, 6),
    square_size_mm: float = 25.0,
    lens_model: str = "fisheye",
    min_images: int = 8,
    max_reproj_err: float = 1.0,
):
    """
    Computes camera intrinsic matrix K and distortion coefficients D from
    real checkerboard images using either OpenCV's fisheye or pinhole model.
    Returns (K, D, image_size, rms_error_px).
    """
    if lens_model not in VALID_LENS_MODELS:
        raise ValueError(f"Unsupported lens_model '{lens_model}'; expected one of {VALID_LENS_MODELS}")
    if not image_paths:
        raise ValueError("No checkerboard images provided for intrinsic calibration.")

    cols, rows = int(pattern_size[0]), int(pattern_size[1])
    objp = np.zeros((1, cols * rows, 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * float(square_size_mm)

    objpoints = []
    imgpoints = []
    image_size = None
    subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

    for path in sorted(image_paths):
        img = cv2.imread(path)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]
        if image_size is None:
            image_size = (w, h)
        elif image_size != (w, h):
            raise ValueError(
                f"Inconsistent image resolution in {path}: got {(w, h)}, expected {image_size}"
            )

        found, corners = cv2.findChessboardCorners(
            gray, (cols, rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if not found:
            continue

        refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), subpix_criteria)
        if lens_model == "fisheye":
            objpoints.append(objp.astype(np.float64))
            imgpoints.append(refined.reshape(1, -1, 2).astype(np.float64))
        else:
            objpoints.append(objp.reshape(-1, 3).astype(np.float32))
            imgpoints.append(refined.astype(np.float32))

    if len(objpoints) < min_images:
        raise RuntimeError(
            f"Only {len(objpoints)} valid checkerboard images found; need at least {min_images}."
        )

    if lens_model == "fisheye":
        K = np.zeros((3, 3), dtype=np.float64)
        D = np.zeros((4, 1), dtype=np.float64)
        rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in objpoints]
        tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in objpoints]
        flags = (
            cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
            + cv2.fisheye.CALIB_FIX_SKEW
        )
        rms, K, D, _, _ = cv2.fisheye.calibrate(
            objpoints, imgpoints, image_size, K, D, rvecs, tvecs,
            flags, (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
        )
        D = np.asarray(D, dtype=np.float64).reshape((4, 1))
    else:
        rms, K, D, _, _ = cv2.calibrateCamera(
            objpoints, imgpoints, image_size, None, None
        )
        K = np.asarray(K, dtype=np.float64).reshape((3, 3))
        D = np.asarray(D, dtype=np.float64).reshape((-1, 1))

    if not np.isfinite(rms) or rms > max_reproj_err:
        raise RuntimeError(
            f"Intrinsic calibration RMS reprojection error {rms:.4f} px exceeds limit {max_reproj_err:.4f} px."
        )

    return K, D, image_size, float(rms)


def compute_homography_from_fiducials(
    rectified_pixel_corners: np.ndarray,
    max_corner_err_mm: float = 1.0,
    image_size: tuple = None,
    check_points_px: np.ndarray = None,
    check_points_mm: np.ndarray = None,
    max_check_err_mm: float = 2.5,
) -> np.ndarray:
    """
    Computes 3x3 planar homography matrix H mapping rectified image pixels
    (u, v) to physical sorting board coordinates (X_mm, Y_mm) after validating
    strict quadrilateral geometry (image bounds, convexity, clockwise winding,
    minimum area, side ratios, interior angles, TL->TR->BR->BL orientation),
    corner round-trip accuracy, and optional interior check-point accuracy.
    """
    rect_pts = np.asarray(rectified_pixel_corners, dtype=np.float32).reshape((4, 2))
    if not np.all(np.isfinite(rect_pts)):
        raise ValueError("Rectified fiducial pixel corners contain non-finite values.")

    # 1. Image bounds check (if image dimensions are known)
    if image_size is not None:
        img_w, img_h = float(image_size[0]), float(image_size[1])
        if img_w <= 0 or img_h <= 0:
            raise ValueError(f"Invalid image_size={image_size}")
        if (
            np.any(rect_pts[:, 0] < 0.0)
            or np.any(rect_pts[:, 0] >= img_w)
            or np.any(rect_pts[:, 1] < 0.0)
            or np.any(rect_pts[:, 1] >= img_h)
        ):
            raise ValueError(
                f"Fiducial corners fall outside image bounds {image_size}: {rect_pts.tolist()}"
            )

    # 2. Edge lengths, strict convexity, and clockwise winding (in image coords: +u right, +v down)
    edges = np.roll(rect_pts, -1, axis=0) - rect_pts
    edge_lens = np.linalg.norm(edges, axis=1)
    if float(np.min(edge_lens)) < 40.0:
        raise ValueError(
            f"Degenerate fiducial quadrilateral: shortest edge is {float(np.min(edge_lens)):.1f} px (<40 px)."
        )
    horiz_ratio = float(edge_lens[0] / edge_lens[2])
    vert_ratio = float(edge_lens[1] / edge_lens[3])
    if not (0.4 <= horiz_ratio <= 2.5 and 0.4 <= vert_ratio <= 2.5):
        raise ValueError(
            f"Fiducial quadrilateral has extreme perspective/edge-length distortion "
            f"(top/bottom={horiz_ratio:.2f}, right/left={vert_ratio:.2f})."
        )

    cross_z = []
    for i in range(4):
        e1 = edges[i]
        e2 = edges[(i + 1) % 4]
        cross_z.append(float(e1[0] * e2[1] - e1[1] * e2[0]))

    if not cv2.isContourConvex(rect_pts) or any(c == 0.0 for c in cross_z):
        raise ValueError(
            "Fiducial quadrilateral is non-convex or self-intersecting (bow-tie/crossed clicks)."
        )
    if any(c < 0.0 for c in cross_z):
        if all(c < 0.0 for c in cross_z):
            raise ValueError(
                "Fiducial corners have counter-clockwise/mirrored winding; expected clockwise TL->TR->BR->BL."
            )
        raise ValueError(
            "Fiducial quadrilateral is concave or self-intersecting (bow-tie/crossed clicks)."
        )

    # 3. Signed area & interior angle sanity (reject slivers / near-collinear quads)
    u = rect_pts[:, 0].astype(np.float64)
    v = rect_pts[:, 1].astype(np.float64)
    area_px2 = 0.5 * float(np.sum(u * np.roll(v, -1) - np.roll(u, -1) * v))
    min_area = 2500.0
    if image_size is not None:
        min_area = max(min_area, 0.05 * float(image_size[0]) * float(image_size[1]))
    if area_px2 < min_area:
        raise ValueError(
            f"Fiducial quadrilateral area {area_px2:.1f} px^2 is too small (min {min_area:.1f} px^2)."
        )

    for i in range(4):
        v_prev = rect_pts[(i - 1) % 4] - rect_pts[i]
        v_next = rect_pts[(i + 1) % 4] - rect_pts[i]
        cos_ang = float(np.dot(v_prev, v_next) / (np.linalg.norm(v_prev) * np.linalg.norm(v_next)))
        ang_deg = float(np.degrees(np.arccos(np.clip(cos_ang, -1.0, 1.0))))
        if not (40.0 <= ang_deg <= 140.0):
            raise ValueError(
                f"Fiducial corner #{i + 1} interior angle {ang_deg:.1f} deg outside [40, 140] deg."
            )

    # 4. Spatial TL/TR/BR/BL ordering consistency (rejects 90/180/270-degree rotated corner sequences)
    v_top_mid = 0.5 * (v[0] + v[1])
    v_bot_mid = 0.5 * (v[2] + v[3])
    u_left_mid = 0.5 * (u[0] + u[3])
    u_right_mid = 0.5 * (u[1] + u[2])
    if not (
        v_top_mid < v_bot_mid - 10.0
        and u_left_mid < u_right_mid - 10.0
        and u[0] < u[1]
        and u[3] < u[2]
        and v[0] < v[3]
        and v[1] < v[2]
    ):
        raise ValueError(
            "Fiducial corner order is rotated or inconsistent; expected TL -> TR -> BR -> BL."
        )

    H, _ = cv2.findHomography(rect_pts, BOARD_CORNERS_MM, method=0)
    if H is None or not np.all(np.isfinite(H)) or abs(np.linalg.det(H)) < 1e-12:
        raise ValueError("Degenerate fiducial corners produced a singular homography matrix.")

    H = H.astype(np.float64)
    mapped = cv2.perspectiveTransform(rect_pts.reshape(1, 4, 2), H).reshape(4, 2)
    max_err = float(np.max(np.linalg.norm(mapped - BOARD_CORNERS_MM, axis=1)))
    if max_err > max_corner_err_mm:
        raise RuntimeError(
            f"Homography corner mapping error {max_err:.4f} mm exceeds limit {max_corner_err_mm:.4f} mm."
        )

    # 5. Optional independent interior check-point verification (e.g., board center or test grid)
    if check_points_px is not None or check_points_mm is not None:
        if check_points_px is None or check_points_mm is None:
            raise ValueError("Provide both check_points_px and check_points_mm together.")
        chk_px = np.asarray(check_points_px, dtype=np.float32).reshape((-1, 2))
        chk_mm = np.asarray(check_points_mm, dtype=np.float64).reshape((-1, 2))
        if chk_px.shape != chk_mm.shape or chk_px.shape[0] == 0:
            raise ValueError("check_points_px and check_points_mm must be non-empty Nx2 arrays of matching shape.")
        if not (np.all(np.isfinite(chk_px)) and np.all(np.isfinite(chk_mm))):
            raise ValueError("Interior check points contain non-finite values.")
        mapped_chk = cv2.perspectiveTransform(chk_px.reshape(1, -1, 2), H).reshape(-1, 2)
        max_chk_err = float(np.max(np.linalg.norm(mapped_chk - chk_mm, axis=1)))
        if max_chk_err > max_check_err_mm:
            raise RuntimeError(
                f"Interior check-point mapping error {max_chk_err:.3f} mm exceeds limit {max_check_err_mm:.3f} mm."
            )

    return H


def save_validated_calibration(
    out_path: str,
    K: np.ndarray,
    D: np.ndarray,
    H: np.ndarray,
    camera_type: str,
    lens_model: str,
    image_size: tuple,
    reprojection_error_px: float,
    camera_height_mm: float = 0.0,
    cap_height_mm: float = 13.0,
):
    if camera_type not in VALID_CAMERA_TYPES:
        raise ValueError(f"Unsupported camera_type '{camera_type}'")
    if lens_model not in VALID_LENS_MODELS:
        raise ValueError(f"Unsupported lens_model '{lens_model}'")

    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    timestamp = datetime.datetime.now(datetime.timezone.utc).isoformat()
    np.savez(
        out_path,
        K=np.asarray(K, dtype=np.float64).reshape((3, 3)),
        D=np.asarray(D, dtype=np.float64).reshape((-1, 1)),
        H=np.asarray(H, dtype=np.float64).reshape((3, 3)),
        camera_type=np.array(camera_type),
        lens_model=np.array(lens_model),
        image_size=np.array([int(image_size[0]), int(image_size[1])], dtype=np.int32),
        reprojection_error_px=np.array(float(reprojection_error_px), dtype=np.float64),
        camera_height_mm=np.array(float(camera_height_mm), dtype=np.float64),
        cap_height_mm=np.array(float(cap_height_mm), dtype=np.float64),
        calibrated=np.array(True, dtype=bool),
        calibrated_utc=np.array(timestamp),
    )
    print(
        f"[OK] Saved validated calibration ({camera_type}, {lens_model}, "
        f"{image_size[0]}x{image_size[1]}, RMS={reprojection_error_px:.4f} px, "
        f"H_cam={camera_height_mm:.1f} mm, h_cap={cap_height_mm:.1f} mm) -> {out_path}"
    )


def save_uncalibrated_placeholder(out_path: str):
    """Writes an explicitly uncalibrated placeholder so runtime fails closed until bench calibrated."""
    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    np.savez(
        out_path,
        K=np.eye(3, dtype=np.float64),
        D=np.zeros((4, 1), dtype=np.float64),
        H=np.eye(3, dtype=np.float64),
        camera_type=np.array("uncalibrated_placeholder"),
        lens_model=np.array("uncalibrated"),
        image_size=np.array([0, 0], dtype=np.int32),
        reprojection_error_px=np.array(float("nan"), dtype=np.float64),
        calibrated=np.array(False, dtype=bool),
        calibrated_utc=np.array(""),
    )


def _parse_points_list_arg(points_str: str) -> np.ndarray:
    pairs = [p.strip() for p in points_str.split(";") if p.strip()]
    if not pairs:
        raise ValueError("Expected one or more semicolon-separated 'x,y' point pairs.")
    pts = []
    for p in pairs:
        x_str, y_str = p.split(",")
        pts.append([float(x_str.strip()), float(y_str.strip())])
    return np.array(pts, dtype=np.float32)


def _parse_corners_arg(corners_str: str) -> np.ndarray:
    pts = _parse_points_list_arg(corners_str)
    if pts.shape != (4, 2):
        raise ValueError("Expected 4 semicolon-separated 'u,v' corners (TL;TR;BR;BL).")
    return pts


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate V2 camera intrinsics (checkerboard) and 4-corner board homography."
    )
    parser.add_argument("--camera-type", required=True, choices=VALID_CAMERA_TYPES)
    parser.add_argument("--lens-model", required=True, choices=VALID_LENS_MODELS)
    parser.add_argument(
        "--checkerboard-glob", required=True,
        help="Glob pattern of checkerboard calibration images (e.g. 'calib_imgs/*.png')",
    )
    parser.add_argument(
        "--pattern", default="9x6",
        help="Inner checkerboard corner counts as COLSxROWS (default: 9x6)",
    )
    parser.add_argument(
        "--square-mm", type=float, default=25.0,
        help="Physical checkerboard square edge length in mm (default: 25.0)",
    )
    parser.add_argument(
        "--corners",
        help="Rectified board fiducial corners as 'u0,v0;u1,v1;u2,v2;u3,v3' (TL;TR;BR;BL)",
    )
    parser.add_argument(
        "--corners-json",
        help="Path to JSON file containing a 4x2 list of rectified fiducial pixel corners",
    )
    parser.add_argument(
        "--min-images", type=int, default=8,
        help="Minimum valid checkerboard images required (default: 8)",
    )
    parser.add_argument(
        "--max-reproj-err", type=float, default=1.0,
        help="Maximum allowed RMS reprojection error in pixels (default: 1.0)",
    )
    parser.add_argument(
        "--camera-height-mm", type=float, default=0.0,
        help="Measured overhead camera lens height above board plane in mm for radial parallax correction (0 disables)",
    )
    parser.add_argument(
        "--cap-height-mm", type=float, default=13.0,
        help="Nominal bottle-cap height in mm used with --camera-height-mm parallax compensation (default: 13.0)",
    )
    parser.add_argument(
        "--check-points-px",
        help="Optional interior rectified check points as 'u0,v0;u1,v1;...' to verify homography accuracy",
    )
    parser.add_argument(
        "--check-points-mm",
        help="Known physical board coordinates 'x0,y0;x1,y1;...' matching --check-points-px",
    )
    parser.add_argument(
        "--max-check-err-mm", type=float, default=2.5,
        help="Maximum allowed interior check-point mapping error in mm (default: 2.5)",
    )
    parser.add_argument(
        "--out",
        help="Output .npz path (defaults to calibration_csi_imx219.npz or calibration_usb.npz)",
    )
    args = parser.parse_args()

    if bool(args.corners) == bool(args.corners_json):
        parser.error("Provide exactly one of --corners or --corners-json with measured fiducials.")

    cols, rows = (int(x) for x in args.pattern.lower().split("x"))
    image_paths = sorted(glob.glob(args.checkerboard_glob))
    K, D, image_size, rms = calibrate_intrinsics_from_checkerboards(
        image_paths=image_paths,
        pattern_size=(cols, rows),
        square_size_mm=args.square_mm,
        lens_model=args.lens_model,
        min_images=args.min_images,
        max_reproj_err=args.max_reproj_err,
    )

    if args.corners:
        corners_px = _parse_corners_arg(args.corners)
    else:
        with open(args.corners_json, "r", encoding="utf-8") as fh:
            corners_px = np.array(json.load(fh), dtype=np.float32)

    check_px = _parse_points_list_arg(args.check_points_px) if args.check_points_px else None
    check_mm = _parse_points_list_arg(args.check_points_mm) if args.check_points_mm else None

    H = compute_homography_from_fiducials(
        corners_px,
        image_size=image_size,
        check_points_px=check_px,
        check_points_mm=check_mm,
        max_check_err_mm=args.max_check_err_mm,
    )
    default_out = (
        "calibration_csi_imx219.npz"
        if args.camera_type == "picamera2_imx219"
        else "calibration_usb.npz"
    )
    save_validated_calibration(
        out_path=args.out or default_out,
        K=K, D=D, H=H,
        camera_type=args.camera_type,
        lens_model=args.lens_model,
        image_size=image_size,
        reprojection_error_px=rms,
        camera_height_mm=args.camera_height_mm,
        cap_height_mm=args.cap_height_mm,
    )


if __name__ == "__main__":
    main()
