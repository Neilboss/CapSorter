#!/usr/bin/env python3
# =========================================================================
# Stage 4 Bench Test: Raspberry Pi Zero 2 W — Live Camera & Cap Box Preview
# Purpose:
#   Verify your camera (8MP IMX219 CSI or USB webcam), lighting, and HSV
#   color thresholds BEFORE calibration or connecting the Arduino Mega.
#   Draws colored bounding SQUARES, circles, centroids (u, v), area, and a
#   live center-pixel HSV inspector onto every frame.
#
# Usage Examples (on Raspberry Pi Zero 2 W):
#   1. Capture one annotated image (headless SSH):
#        python3 pi_01_camera_detect_preview.py --once --out-dir ./stage4_out
#   2. Stream LIVE bounding squares to your phone/laptop browser:
#        python3 pi_01_camera_detect_preview.py --web-port 8080
#      (Then open http://<pi-ip>:8080 in any browser!)
#   3. Mirror live bounding squares onto the 3.5" SPI LCD (/dev/fb0):
#        python3 pi_01_camera_detect_preview.py --lcd /dev/fb0
#   4. Test offline on a saved PNG photo:
#        python3 pi_01_camera_detect_preview.py --image sample.png --once
# =========================================================================
import argparse
import http.server
import math
import os
import socketserver
import sys
import threading
import time

import cv2
import numpy as np

V2_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v2_pizero2w"))
if V2_DIR not in sys.path:
    sys.path.insert(0, V2_DIR)

import lcd_hud as hud_mod
import pi_zero_vision as pzv

COLOR_NAMES = {
    pzv.COLOR_RED:    "RED",
    pzv.COLOR_YELLOW: "YELLOW",
    pzv.COLOR_GREEN:  "GREEN",
    pzv.COLOR_BLUE:   "BLUE",
}
COLOR_BGR = {
    pzv.COLOR_RED:    (40, 40, 255),
    pzv.COLOR_YELLOW: (0, 225, 255),
    pzv.COLOR_GREEN:  (60, 220, 40),
    pzv.COLOR_BLUE:   (255, 140, 40),
}


class CameraSource:
    """Unified camera wrapper supporting IMX219 CSI (Picamera2), USB V4L2, or static PNG."""

    def __init__(self, camera_type: str, width: int = 640, height: int = 480, image_path: str = None):
        self.camera_type = camera_type
        self.width = int(width)
        self.height = int(height)
        self.image_path = image_path
        self.picam2 = None
        self.usb_cam = None
        self._static_frame = None

        if image_path:
            img = cv2.imread(image_path)
            if img is None:
                raise RuntimeError(f"Could not read image file: {image_path}")
            self._static_frame = cv2.resize(img, (self.width, self.height))
            return

        if camera_type == "picamera2_imx219":
            if not pzv.HAS_PICAMERA2:
                raise RuntimeError(
                    "Picamera2 is not installed. Use '--camera-type usb_v4l2' or install python3-picamera2."
                )
            self.picam2 = pzv.Picamera2()
            cfg = self.picam2.create_still_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"},
                raw={"size": (1640, 1232)},
            )
            self.picam2.configure(cfg)
            self.picam2.start()
            time.sleep(0.8)
            meta = self.picam2.capture_metadata()
            lock_ctrls = {"AeEnable": False, "AwbEnable": False}
            if "ExposureTime" in meta:
                lock_ctrls["ExposureTime"] = int(meta["ExposureTime"])
            if "AnalogueGain" in meta:
                lock_ctrls["AnalogueGain"] = float(meta["AnalogueGain"])
            if "ColourGains" in meta and len(meta["ColourGains"]) == 2:
                lock_ctrls["ColourGains"] = (float(meta["ColourGains"][0]), float(meta["ColourGains"][1]))
            self.picam2.set_controls(lock_ctrls)
        else:
            self.usb_cam = cv2.VideoCapture(0)
            if not self.usb_cam.isOpened():
                raise RuntimeError("Failed to open USB camera (/dev/video0).")
            self.usb_cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.usb_cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.usb_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

    def read(self) -> np.ndarray:
        if self._static_frame is not None:
            return self._static_frame.copy()
        if self.picam2 is not None:
            return self.picam2.capture_array("main")
        for _ in range(2):
            self.usb_cam.grab()
        ok, frame = self.usb_cam.read()
        if not ok:
            raise RuntimeError("USB camera frame capture failed.")
        return frame

    def close(self):
        if self.picam2 is not None:
            try:
                self.picam2.stop()
            except Exception:
                pass
        if self.usb_cam is not None:
            self.usb_cam.release()


def detect_and_draw_boxes(frame_bgr: np.ndarray, min_area: float = 350.0) -> tuple:
    """
    Runs CapSorter's HSV segmentation on an uncalibrated raw frame and draws:
      - Bright bounding square/rectangle around every detected colored cap
      - Center crosshair + pixel (u, v) + color label + pixel area
      - Center-of-frame HSV swatch so you can tune HSV thresholds under real lighting
    Returns (annotated_bgr, detections_list).
    """
    vis = frame_bgr.copy()
    h, w = vis.shape[:2]
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    detections = []
    for color_id, ranges in pzv.HSV_RANGES.items():
        mask = np.zeros((h, w), dtype=np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            perim = cv2.arcLength(cnt, True)
            circ = (4.0 * math.pi * area) / (perim * perim + 1e-5)
            M = cv2.moments(cnt)
            cx = int(round(M["m10"] / M["m00"])) if M["m00"] > 0 else (x + bw // 2)
            cy = int(round(M["m01"] / M["m00"])) if M["m00"] > 0 else (y + bh // 2)

            det = {
                "color": COLOR_NAMES[color_id],
                "color_id": color_id,
                "u": cx,
                "v": cy,
                "bbox": (x, y, bw, bh),
                "area_px": int(round(area)),
                "circularity": round(circ, 2),
                "likely_cluster": circ < 0.72 or area > 2600.0,
            }
            detections.append(det)

            col = COLOR_BGR[color_id]
            # Draw square/box around cap + centroid crosshair
            cv2.rectangle(vis, (x - 2, y - 2), (x + bw + 2, y + bh + 2), col, 2)
            cv2.drawMarker(vis, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 10, 1)
            tag = f"{det['color']} ({cx},{cy})"
            if det["likely_cluster"]:
                tag += " [CLUSTER]"
            ty = max(16, y - 6)
            cv2.putText(vis, tag, (x, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(vis, tag, (x, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)

    # Center crosshair + live 11x11 median HSV readout for lighting/threshold tuning
    mx, my = w // 2, h // 2
    patch = hsv[max(0, my - 5):min(h, my + 6), max(0, mx - 5):min(w, mx + 6)]
    med_h, med_s, med_v = [int(round(v)) for v in np.median(patch.reshape(-1, 3), axis=0)]
    cv2.drawMarker(vis, (mx, my), (255, 255, 0), cv2.MARKER_CROSS, 16, 1)
    cv2.rectangle(vis, (0, 0), (w, 26), (18, 18, 18), -1)
    banner = f"STAGE 4 RAW CAP PREVIEW | Caps: {len(detections)} | Center HSV: ({med_h}, {med_s}, {med_v})"
    cv2.putText(vis, banner, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (240, 240, 240), 1, cv2.LINE_AA)
    return vis, detections


def start_mjpeg_server(port: int, frame_holder: dict):
    """Starts a zero-dependency live MJPEG stream viewable from any browser."""

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            pass

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                body = (
                    "<html><body style='background:#111;color:#eee;font-family:sans-serif;text-align:center'>"
                    "<h3>CapSorter Stage 4 — Live Camera Cap Box Preview</h3>"
                    "<img src='/stream.mjpg' style='max-width:95%;border:2px solid #0ea5e9;border-radius:8px'/>"
                    "</body></html>"
                ).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path == "/stream.mjpg":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        jpg = frame_holder.get("jpg")
                        if jpg:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii"))
                            self.wfile.write(jpg + b"\r\n")
                        time.sleep(0.12)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    return
            self.send_error(404)

    class ReusableServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
        allow_reuse_address = True
        daemon_threads = True

    srv = ReusableServer(("0.0.0.0", int(port)), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def blit_to_lcd(fb_path: str, frame_bgr: np.ndarray):
    if not fb_path or not os.path.exists(fb_path):
        return
    fb_name = os.path.basename(fb_path)
    bpp = 16
    try:
        with open(f"/sys/class/graphics/{fb_name}/bits_per_pixel", "r", encoding="utf-8") as fh:
            v = int(fh.read().strip())
            if v in (16, 32):
                bpp = v
    except (OSError, ValueError):
        pass
    raw = hud_mod.bgr_to_bgra32_bytes(frame_bgr) if bpp == 32 else hud_mod.bgr_to_rgb565_bytes(frame_bgr)
    with open(fb_path, "wb") as fb:
        fb.write(raw)


def main():
    parser = argparse.ArgumentParser(description="Stage 4: Live Camera & Bounding-Square Cap Detection Preview")
    parser.add_argument("--camera-type", choices=("picamera2_imx219", "usb_v4l2"),
                        default="picamera2_imx219" if pzv.HAS_PICAMERA2 else "usb_v4l2")
    parser.add_argument("--image", help="Test offline on an existing PNG/JPG image instead of live camera")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--min-area", type=float, default=350.0)
    parser.add_argument("--out-dir", default="stage4_preview", help="Folder to save raw + annotated PNGs")
    parser.add_argument("--once", action="store_true", help="Capture 1 frame, save PNGs, print table, and exit")
    parser.add_argument("--web-port", type=int, default=0, help="Serve live MJPEG preview on HTTP port (e.g. 8080)")
    parser.add_argument("--lcd", default="", help="Mirror preview onto 3.5-inch SPI LCD framebuffer (e.g. /dev/fb0)")
    parser.add_argument("--show", action="store_true", help="Open local OpenCV GUI window (requires desktop/X11)")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    cam = CameraSource(args.camera_type, args.width, args.height, args.image)
    frame_holder = {"jpg": None}
    srv = None
    if args.web_port > 0:
        srv = start_mjpeg_server(args.web_port, frame_holder)
        print(f"[WEB] Live preview running at http://<pi-ip>:{args.web_port}/  (Ctrl+C to stop)")

    try:
        while True:
            raw = cam.read()
            vis, dets = detect_and_draw_boxes(raw, min_area=args.min_area)

            if args.once or (args.web_port == 0 and not args.show and not args.lcd):
                raw_path = os.path.join(args.out_dir, "stage4_raw.png")
                box_path = os.path.join(args.out_dir, "stage4_boxes.png")
                cv2.imwrite(raw_path, raw)
                cv2.imwrite(box_path, vis)
                print(f"[SAVED] Raw frame       -> {raw_path}")
                print(f"[SAVED] Annotated boxes -> {box_path}")
                print(f"\nDetected {len(dets)} cap region(s):")
                print(f"{'#':>2}  {'COLOR':<8} {'PIXEL (u, v)':<14} {'BOX (x,y,w,h)':<18} {'AREA':>7} {'CIRC':>5} {'NOTE'}")
                for idx, d in enumerate(dets, 1):
                    note = "Touching/Cluster" if d["likely_cluster"] else "Single"
                    print(
                        f"{idx:>2}  {d['color']:<8} ({d['u']:>3},{d['v']:>3})       "
                        f"{str(d['bbox']):<18} {d['area_px']:>7} {d['circularity']:>5.2f} {note}"
                    )
                break

            if args.web_port > 0:
                ok, enc = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if ok:
                    frame_holder["jpg"] = enc.tobytes()

            if args.lcd:
                blit_to_lcd(args.lcd, vis)

            if args.show:
                cv2.imshow("CapSorter Stage 4 - Cap Boxes (Press Q to quit, S to save)", vis)
                key = cv2.waitKey(30) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("s"):
                    cv2.imwrite(os.path.join(args.out_dir, "stage4_boxes.png"), vis)
                    print("[SAVED] Snapshot updated.")
            else:
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n[EXIT] Stopped Stage 4 preview.")
    finally:
        cam.close()
        if srv is not None:
            srv.shutdown()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
