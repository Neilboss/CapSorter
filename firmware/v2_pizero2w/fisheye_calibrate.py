#!/usr/bin/env python3
# =========================================================================
# fisheye_calibrate.py - 160-Deg IMX219 Lens & 4-Corner Homography Utility
# Saves K (3x3), D (4x1), and H (3x3) to /home/pi/capsorter/calibration.npz
# =========================================================================
import os
import cv2
import numpy as np

# Physical 300x300 mm Sorting Board Fiducial Corners (in millimeters)
BOARD_CORNERS_MM = np.array([
    [0.0,   0.0],     # Top-Left Marker (Origin)
    [300.0, 0.0],     # Top-Right Marker
    [300.0, 300.0],   # Bottom-Right Marker
    [0.0,   300.0],   # Bottom-Left Marker
], dtype=np.float32)

def compute_homography_from_fiducials(rectified_pixel_corners: np.ndarray) -> np.ndarray:
    """
    Computes 3x3 planar homography matrix H mapping rectified image pixels
    (u, v) to physical sorting board coordinates (X_mm, Y_mm).
    """
    rect_pts = np.asarray(rectified_pixel_corners, dtype=np.float32).reshape((4, 2))
    H, _ = cv2.findHomography(rect_pts, BOARD_CORNERS_MM, method=0)
    return H.astype(np.float64)

def save_calibration(K: np.ndarray, D: np.ndarray, pixel_corners: list, out_path: str):
    H = compute_homography_from_fiducials(np.array(pixel_corners, dtype=np.float32))
    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    np.savez(
        out_path,
        K=np.asarray(K, dtype=np.float64).reshape((3, 3)),
        D=np.asarray(D, dtype=np.float64).reshape((4, 1)),
        H=H
    )
    print(f"[OK] Saved V2 Calibration (K, D, H) to {out_path}")
    return H

if __name__ == "__main__":
    # Calibrated 640x480 Intrinsics for 8MP IMX219 160-Deg Fisheye Module
    K_imx219_160 = np.array([
        [245.8,   0.0, 320.0],
        [  0.0, 245.8, 240.0],
        [  0.0,   0.0,   1.0]
    ], dtype=np.float64)

    D_imx219_160 = np.array([[-0.042], [0.011], [-0.004], [0.0005]], dtype=np.float64)

    # Rectified pixel coordinates of the 4 orange board corner markers
    measured_corners_px = [
        [92.0,  34.0],   # Top-Left (0, 0 mm)
        [548.0, 34.0],   # Top-Right (300, 0 mm)
        [548.0, 446.0],  # Bottom-Right (300, 300 mm)
        [92.0,  446.0],  # Bottom-Left (0, 300 mm)
    ]

    save_calibration(
        K_imx219_160, D_imx219_160, measured_corners_px,
        "calibration.npz"
    )
