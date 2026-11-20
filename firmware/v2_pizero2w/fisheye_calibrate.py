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
) -> np.ndarray:
    """
    Computes 3x3 planar homography matrix H mapping rectified image pixels
    (u, v) to physical sorting board coordinates (X_mm, Y_mm) and validates
    corner round-trip accuracy.
    """
    rect_pts = np.asarray(rectified_pixel_corners, dtype=np.float32).reshape((4, 2))
    if not np.all(np.isfinite(rect_pts)):
        raise ValueError("Rectified fiducial pixel corners contain non-finite values.")

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
        calibrated=np.array(True, dtype=bool),
        calibrated_utc=np.array(timestamp),
    )
    print(
        f"[OK] Saved validated calibration ({camera_type}, {lens_model}, "
        f"{image_size[0]}x{image_size[1]}, RMS={reprojection_error_px:.4f} px) -> {out_path}"
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


def _parse_corners_arg(corners_str: str) -> np.ndarray:
    pairs = [p.strip() for p in corners_str.split(";") if p.strip()]
    if len(pairs) != 4:
        raise ValueError("Expected 4 semicolon-separated 'u,v' corners (TL;TR;BR;BL).")
    pts = []
    for p in pairs:
        u_str, v_str = p.split(",")
        pts.append([float(u_str.strip()), float(v_str.strip())])
    return np.array(pts, dtype=np.float32)


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

    H = compute_homography_from_fiducials(corners_px)
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
    )


if __name__ == "__main__":
    main()
