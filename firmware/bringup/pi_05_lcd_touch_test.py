#!/usr/bin/env python3
# =========================================================================
# Stage 7B Bench Test: 3.5" SPI Touch LCD (/dev/fb0) & XPT2046 Touch Calibrator
# Purpose:
#   1. Verifies framebuffer rendering (auto-detecting 16-bpp RGB565 vs 32-bpp
#      BGRA32) by drawing a crisp 480x320 calibration screen with 4 corner
#      crosshairs and the 3 CapSorter sidebar buttons.
#   2. Reads live XPT2046 resistive touch events and prints raw ADC values
#      (raw_x, raw_y) plus mapped (px, py) & detected button hits so you can
#      confirm whether --touch-swap-xy / --touch-invert-x / --touch-invert-y
#      are needed for your exact 3.5" V3.0 panel.
#
# Usage:
#   python3 pi_05_lcd_touch_test.py --fb-device /dev/fb0
#   python3 pi_05_lcd_touch_test.py --snapshot stage7_lcd_test.png   # offline test
# =========================================================================
import argparse
import os
import select
import struct
import sys

import cv2
import numpy as np

V2_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "v2_pizero2w"))
if V2_DIR not in sys.path:
    sys.path.insert(0, V2_DIR)

import lcd_hud as hud_mod


def render_test_pattern(last_tap=None) -> np.ndarray:
    img = np.full((hud_mod.LCD_HEIGHT, hud_mod.LCD_WIDTH, 3), hud_mod.COL_BG, dtype=np.uint8)
    w, h = hud_mod.LCD_WIDTH, hud_mod.LCD_HEIGHT

    # Outer border + RGB color verification bars
    cv2.rectangle(img, (2, 2), (w - 3, h - 3), hud_mod.COL_CYAN, 2)
    bars = [((40, 40, 240), "RED"), ((20, 210, 240), "YLW"),
            ((80, 210, 40), "GRN"), ((240, 120, 40), "BLU")]
    for i, (bgr, name) in enumerate(bars):
        x0 = 40 + i * 64
        cv2.rectangle(img, (x0, 90), (x0 + 56, 130), bgr, -1)
        cv2.putText(img, name, (x0 + 10, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (15, 15, 15), 2, cv2.LINE_AA)

    cv2.putText(img, "STAGE 7B: 3.5-INCH SPI LCD & TOUCH TEST", (28, 42),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, hud_mod.COL_WHITE, 1, cv2.LINE_AA)
    cv2.putText(img, "Tap corners & right buttons to verify alignment", (28, 68),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, hud_mod.COL_AMBER, 1, cv2.LINE_AA)

    # 4 Corner Crosshairs
    for cx, cy, lbl in [(20, 20, "TL"), (w - 20, 20, "TR"), (w - 20, h - 20, "BR"), (20, h - 20, "BL")]:
        cv2.drawMarker(img, (cx, cy), hud_mod.COL_EMERALD, cv2.MARKER_CROSS, 18, 2)
        cv2.putText(img, lbl, (cx - 10, cy + (22 if cy < 50 else -12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, hud_mod.COL_EMERALD, 1, cv2.LINE_AA)

    # Exact CapSorter HUD button hitboxes
    for key, (x0, y0, x1, y1) in hud_mod.TOUCH_BUTTONS.items():
        cv2.rectangle(img, (x0, y0), (x1, y1), hud_mod.COL_PANEL, -1)
        cv2.rectangle(img, (x0, y0), (x1, y1), hud_mod.COL_CYAN, 2)
        cv2.putText(img, key.upper(), (x0 + 42, (y0 + y1) // 2 + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, hud_mod.COL_WHITE, 1, cv2.LINE_AA)

    if last_tap is not None:
        px, py, raw_x, raw_y, btn = last_tap
        cv2.circle(img, (px, py), 10, hud_mod.COL_ORANGE, 2)
        info = f"TAP px=({px},{py}) raw=({raw_x},{raw_y}) btn={btn or 'NONE'}"
        cv2.putText(img, info, (24, 175), cv2.FONT_HERSHEY_SIMPLEX, 0.42, hud_mod.COL_WHITE, 1, cv2.LINE_AA)

    return img


def main():
    parser = argparse.ArgumentParser(description="Stage 7B: 3.5-inch SPI LCD & XPT2046 Touch Test")
    parser.add_argument("--fb-device", default="/dev/fb0")
    parser.add_argument("--touch-device", default="auto")
    parser.add_argument("--touch-swap-xy", action="store_true")
    parser.add_argument("--touch-invert-x", action="store_true")
    parser.add_argument("--touch-invert-y", action="store_true")
    parser.add_argument("--snapshot", help="Write 480x320 test pattern PNG and exit")
    args = parser.parse_args()

    if args.snapshot:
        cv2.imwrite(args.snapshot, render_test_pattern((390, 228, 3200, 2700, "pause")))
        print(f"[OK] Saved LCD test pattern -> {args.snapshot}")
        return

    hud = hud_mod.CapSorterLCDHUD(
        fb_device=args.fb_device,
        touch_device=args.touch_device,
        touch_swap_xy=args.touch_swap_xy,
        touch_invert_x=args.touch_invert_x,
        touch_invert_y=args.touch_invert_y,
    )
    print(f"[LCD] Framebuffer: {args.fb_device} (detected {hud._fb_bpp}-bpp)")
    if os.path.exists(args.fb_device):
        hud._blit_framebuffer(render_test_pattern())
        print("[OK] Test pattern written to LCD! Verify colors (RED, YLW, GRN, BLU) look right.")
    else:
        print(f"[WARN] {args.fb_device} not found; check dtoverlay=piscreen,drm in /boot/firmware/config.txt")

    dev_path = hud._find_touch_input_device()
    if not dev_path or not os.path.exists(dev_path):
        print("[WARN] No XPT2046/ADS7846 touch input device found under /dev/input/event*.")
        return

    print(f"[TOUCH] Listening on {dev_path} (tap screen corners & buttons; Ctrl+C to exit)...")
    ev_fmt = "llHHi"
    ev_size = struct.calcsize(ev_fmt)
    raw_x, raw_y = 2048, 2048
    fd = os.open(dev_path, os.O_RDONLY | os.O_NONBLOCK)
    try:
        while True:
            rlist, _, _ = select.select([fd], [], [], 0.25)
            if not rlist:
                continue
            try:
                data = os.read(fd, ev_size * 16)
            except BlockingIOError:
                continue
            for off in range(0, len(data) - (len(data) % ev_size), ev_size):
                _, _, ev_type, code, value = struct.unpack(ev_fmt, data[off:off + ev_size])
                if ev_type == 3:
                    if code == 0:
                        raw_x = value
                    elif code == 1:
                        raw_y = value
                elif ev_type == 1 and code == 330 and value == 0:
                    ax, ay = (raw_y, raw_x) if args.touch_swap_xy else (raw_x, raw_y)
                    x_min, x_max, y_min, y_max = hud.touch_cal
                    nx = (ax - x_min) / max(1.0, (x_max - x_min))
                    ny = (ay - y_min) / max(1.0, (y_max - y_min))
                    if args.touch_invert_x:
                        nx = 1.0 - nx
                    if args.touch_invert_y:
                        ny = 1.0 - ny
                    px = int(min(hud_mod.LCD_WIDTH - 1, max(0, nx * hud_mod.LCD_WIDTH)))
                    py = int(min(hud_mod.LCD_HEIGHT - 1, max(0, ny * hud_mod.LCD_HEIGHT)))
                    btn = hud_mod.hit_test_touch_button(px, py)
                    print(f"[TAP] raw=({raw_x:4d}, {raw_y:4d}) -> screen=({px:3d}, {py:3d}) | button={btn or 'NONE'}")
                    if os.path.exists(args.fb_device):
                        hud._blit_framebuffer(render_test_pattern((px, py, raw_x, raw_y, btn)))
    except KeyboardInterrupt:
        print("\n[EXIT] Stopped Stage 7B LCD test.")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
