#!/usr/bin/env python3
# =========================================================================
# lcd_hud.py - Lightweight 480x320 Native SPI LCD HUD for Pi Zero 2 W (V2)
# Target:   3.5" RPi LCD V3.0 (ILI9486 SPI + XPT2046 Touch, 480x320 HVGA)
# OS Stack: Raspberry Pi OS Lite 64-bit Bookworm (NO Desktop / NO Chromium)
# Driver:   dtoverlay=piscreen,drm,speed=18000000,rotate=90 -> /dev/fb0 (RGB565)
# Purpose:  Mirrors CapSorter web simulator (live caps, animated gantry &
#           gripper motion, bin counts, fault banners & tactile Re-Arm)
#           in <20 MB RAM without ever blocking the 250 ms Mega heartbeat.
# =========================================================================
import argparse
from dataclasses import dataclass
import glob
import math
import os
import select
import struct
import threading
import time
import cv2
import numpy as np

try:
    import fcntl
except ImportError:
    fcntl = None

FBIOGET_VSCREENINFO = 0x4600
FBIOGET_FSCREENINFO = 0x4602

LCD_WIDTH  = 480
LCD_HEIGHT = 320


@dataclass
class FramebufferInfo:
    width: int = LCD_WIDTH
    height: int = LCD_HEIGHT
    bpp: int = 16
    line_length: int = LCD_WIDTH * 2
    red_offset: int = 11
    red_len: int = 5
    green_offset: int = 5
    green_len: int = 6
    blue_offset: int = 0
    blue_len: int = 5

BOARD_SIZE_MM             = 300.0
SAFE_PICK_MIN_MM          = 18.0
SAFE_PICK_MAX_MM          = 282.0
CAP_RADIUS_MM             = 15.0
JAW_INNER_HALF_MM         = 18.0
JAW_OUTER_HALF_MM         = 26.0
JAW_WIDTH_HALF_MM         = 10.0
BRIDGE_OCCLUSION_HALF_MM  = 35.0

# Kinematic parameters matching firmware/mega_motion/mega_motion.ino
MEGA_MAX_SPEED_MM_S = 70.0
MEGA_ACCEL_MM_S2    = 150.0
PICK_LOWER_S        = 0.35
PICK_GRIP_S         = 0.30
PICK_LIFT_S         = 0.35
BIN_SETTLE_S        = 0.12
BIN_LOWER_S         = 0.25
BIN_RELEASE_S       = 0.25
BIN_RETRACT_S       = 0.30

# Layout geometry (480x320 landscape: left 300x300 board @ 1.0 px/mm + right 168 px sidebar)
BOARD_X0   = 6
BOARD_Y0   = 10
BOARD_PX   = 300
SIDEBAR_X0 = 314
SIDEBAR_W  = LCD_WIDTH - SIDEBAR_X0 - 6  # 160 px

# CapColor enum (matches packet_protocol.h / pi_zero_vision.py)
COLOR_RED, COLOR_YELLOW, COLOR_GREEN, COLOR_BLUE = 1, 2, 3, 4

# BGR color palette (OpenCV native order)
COL_BG        = (20, 13, 10)
COL_PANEL     = (34, 22, 17)
COL_BORDER    = (68, 52, 42)
COL_BOARD_BG  = (30, 21, 15)
COL_GRID      = (54, 42, 33)
COL_SAFE_LINE = (80, 135, 65)
COL_CYAN      = (255, 210, 0)
COL_ORANGE    = (0, 107, 255)
COL_EMERALD   = (129, 185, 16)
COL_AMBER     = (36, 191, 251)
COL_RED_ALERT = (68, 68, 239)
COL_WHITE     = (248, 245, 240)
COL_MUTED     = (158, 145, 132)

CAP_BGR = {
    COLOR_RED:    (58, 58, 239),
    COLOR_YELLOW: (20, 198, 238),
    COLOR_GREEN:  (125, 192, 22),
    COLOR_BLUE:   (246, 130, 59),
}
CAP_LABEL = {
    COLOR_RED:    "R",
    COLOR_YELLOW: "Y",
    COLOR_GREEN:  "G",
    COLOR_BLUE:   "B",
}
BIN_Y_BY_COLOR = {
    COLOR_RED:    55.0,
    COLOR_YELLOW: 130.0,
    COLOR_GREEN:  205.0,
    COLOR_BLUE:   280.0,
}

# Resistive Touch Button Hitboxes (x0, y0, x1, y1) in 480x320 screen space
TOUCH_BUTTONS = {
    "pause": (SIDEBAR_X0, 212, SIDEBAR_X0 + SIDEBAR_W, 244),
    "rearm": (SIDEBAR_X0, 250, SIDEBAR_X0 + SIDEBAR_W, 282),
    "view":  (SIDEBAR_X0, 288, SIDEBAR_X0 + SIDEBAR_W, 314),
}


def compute_axis_move_time(distance_mm: float) -> float:
    """Exact trapezoidal/triangular profile duration matching Mega AccelStepper."""
    d = abs(float(distance_mm))
    if d <= 1e-6:
        return 0.0
    v = MEGA_MAX_SPEED_MM_S
    a = MEGA_ACCEL_MM_S2
    d_ramp = (v * v) / a
    if d >= d_ramp:
        return (d / v) + (v / a)
    return 2.0 * math.sqrt(d / a)


def compute_xy_move_time(dx_mm: float, dy_mm: float) -> float:
    return max(compute_axis_move_time(dx_mm), compute_axis_move_time(dy_mm))


def _smoothstep(u: float) -> float:
    u = min(1.0, max(0.0, float(u)))
    return u * u * (3.0 - 2.0 * u)


def bgr_to_rgb565_bytes(bgr: np.ndarray) -> bytes:
    """
    Packs a 480x320x3 BGR uint8 OpenCV image into little-endian RGB565 bytes
    for 16-bpp Linux framebuffer (/dev/fb0 or /dev/fb1) writes (307,200 B).
    """
    if bgr.shape[:2] != (LCD_HEIGHT, LCD_WIDTH):
        bgr = cv2.resize(bgr, (LCD_WIDTH, LCD_HEIGHT), interpolation=cv2.INTER_AREA)
    b = bgr[:, :, 0].astype(np.uint16)
    g = bgr[:, :, 1].astype(np.uint16)
    r = bgr[:, :, 2].astype(np.uint16)
    rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
    return np.ascontiguousarray(rgb565, dtype="<u2").tobytes()


def bgr_to_bgra32_bytes(bgr: np.ndarray) -> bytes:
    """
    Packs a 480x320x3 BGR uint8 OpenCV image into little-endian XRGB8888/BGRA32 bytes
    for 32-bpp DRM fbdev emulation (/dev/fb0) writes (614,400 B).
    """
    if bgr.shape[:2] != (LCD_HEIGHT, LCD_WIDTH):
        bgr = cv2.resize(bgr, (LCD_WIDTH, LCD_HEIGHT), interpolation=cv2.INTER_AREA)
    bgra = cv2.cvtColor(bgr, cv2.COLOR_BGR2BGRA)
    return np.ascontiguousarray(bgra, dtype=np.uint8).tobytes()


def probe_framebuffer_info(fb_device: str) -> FramebufferInfo:
    """
    Queries true Linux fbdev resolution, bits-per-pixel, row stride (line_length),
    and RGB/BGR bitfield offsets via FBIOGET_VSCREENINFO / FBIOGET_FSCREENINFO,
    falling back gracefully to sysfs and packed 480x320 RGB565 defaults.
    """
    info = FramebufferInfo()
    if not fb_device:
        return info

    # 1. Preferred: direct Linux fbdev ioctl query
    if fcntl is not None and os.path.exists(fb_device):
        try:
            with open(fb_device, "rb") as fb:
                vbuf = bytearray(160)
                fcntl.ioctl(fb.fileno(), FBIOGET_VSCREENINFO, vbuf, True)
                xres, yres = struct.unpack_from("<II", vbuf, 0)
                bpp = struct.unpack_from("<I", vbuf, 24)[0]
                r_off, r_len, _ = struct.unpack_from("<III", vbuf, 36)
                g_off, g_len, _ = struct.unpack_from("<III", vbuf, 48)
                b_off, b_len, _ = struct.unpack_from("<III", vbuf, 60)

                fbuf = bytearray(80)
                fcntl.ioctl(fb.fileno(), FBIOGET_FSCREENINFO, fbuf, True)
                fix_fmt = "@16sLIIIIHHHI"
                line_len = struct.unpack_from(fix_fmt, fbuf, 0)[-1]

                if xres > 0 and yres > 0 and bpp in (16, 32):
                    min_stride = xres * (bpp // 8)
                    return FramebufferInfo(
                        width=int(xres),
                        height=int(yres),
                        bpp=int(bpp),
                        line_length=max(int(line_len), min_stride),
                        red_offset=int(r_off),
                        red_len=int(r_len) or (8 if bpp == 32 else 5),
                        green_offset=int(g_off),
                        green_len=int(g_len) or (8 if bpp == 32 else 6),
                        blue_offset=int(b_off),
                        blue_len=int(b_len) or (8 if bpp == 32 else 5),
                    )
        except (OSError, struct.error, ValueError):
            pass

    # 2. Secondary fallback: Linux sysfs attributes
    fb_name = os.path.basename(fb_device)
    sys_dir = f"/sys/class/graphics/{fb_name}"
    try:
        with open(os.path.join(sys_dir, "bits_per_pixel"), "r", encoding="utf-8") as fh:
            val = int(fh.read().strip())
            if val in (16, 32):
                info.bpp = val
                if val == 32:
                    info.red_offset, info.red_len = 16, 8
                    info.green_offset, info.green_len = 8, 8
                    info.blue_offset, info.blue_len = 0, 8
    except (OSError, ValueError):
        pass
    try:
        with open(os.path.join(sys_dir, "virtual_size"), "r", encoding="utf-8") as fh:
            w_s, h_s = fh.read().strip().split(",")
            w_i, h_i = int(w_s), int(h_s)
            if w_i > 0 and h_i > 0:
                info.width, info.height = w_i, h_i
    except (OSError, ValueError):
        pass
    min_stride = info.width * (info.bpp // 8)
    info.line_length = min_stride
    try:
        with open(os.path.join(sys_dir, "stride"), "r", encoding="utf-8") as fh:
            st_i = int(fh.read().strip())
            if st_i >= min_stride:
                info.line_length = st_i
    except (OSError, ValueError):
        pass
    return info


def pack_bgr_for_framebuffer(bgr: np.ndarray, fb_info: FramebufferInfo) -> bytes:
    """
    Resizes if needed, packs BGR uint8 pixels according to the framebuffer's exact
    bpp and red/green/blue bitfield offsets/lengths, and pads each scanline to
    fb_info.line_length bytes so padded DRM/SPI strides never skew or corrupt pixels.
    """
    w = int(fb_info.width)
    h = int(fb_info.height)
    if bgr.shape[:2] != (h, w):
        bgr = cv2.resize(bgr, (w, h), interpolation=cv2.INTER_AREA)

    b_u8 = bgr[:, :, 0]
    g_u8 = bgr[:, :, 1]
    r_u8 = bgr[:, :, 2]

    if fb_info.bpp == 32:
        bytes_per_px = 4
        r_shift = max(0, 8 - fb_info.red_len)
        g_shift = max(0, 8 - fb_info.green_len)
        b_shift = max(0, 8 - fb_info.blue_len)
        packed = (
            ((r_u8.astype(np.uint32) >> r_shift) << fb_info.red_offset)
            | ((g_u8.astype(np.uint32) >> g_shift) << fb_info.green_offset)
            | ((b_u8.astype(np.uint32) >> b_shift) << fb_info.blue_offset)
        )
        # Preserve opaque alpha byte in whichever 8-bit lane is unused (typically bit 24)
        used_offsets = {fb_info.red_offset, fb_info.green_offset, fb_info.blue_offset}
        for alpha_off in (24, 0):
            if alpha_off not in used_offsets:
                packed |= np.uint32(0xFF) << alpha_off
                break
        row_bytes_arr = np.ascontiguousarray(packed, dtype="<u4").view(np.uint8).reshape(h, w * bytes_per_px)
    elif fb_info.bpp == 16:
        bytes_per_px = 2
        r_shift = max(0, 8 - fb_info.red_len)
        g_shift = max(0, 8 - fb_info.green_len)
        b_shift = max(0, 8 - fb_info.blue_len)
        packed = (
            ((r_u8.astype(np.uint16) >> r_shift) << fb_info.red_offset)
            | ((g_u8.astype(np.uint16) >> g_shift) << fb_info.green_offset)
            | ((b_u8.astype(np.uint16) >> b_shift) << fb_info.blue_offset)
        )
        row_bytes_arr = np.ascontiguousarray(packed, dtype="<u2").view(np.uint8).reshape(h, w * bytes_per_px)
    else:
        raise ValueError(f"Unsupported framebuffer bpp={fb_info.bpp}; expected 16 or 32.")

    row_width_bytes = w * bytes_per_px
    stride = max(int(fb_info.line_length), row_width_bytes)
    if stride > row_width_bytes:
        padded = np.zeros((h, stride), dtype=np.uint8)
        padded[:, :row_width_bytes] = row_bytes_arr
        return padded.tobytes()
    return row_bytes_arr.tobytes()


def hit_test_touch_button(x_px: int, y_px: int):
    """Returns button id ('pause', 'rearm', 'view') if (x_px, y_px) hits a button."""
    for name, (x0, y0, x1, y1) in TOUCH_BUTTONS.items():
        if x0 <= x_px <= x1 and y0 <= y_px <= y1:
            return name
    return None


class KinematicGantryPredictor:
    """
    Predicts continuous real-time gantry (X, Y), Z-state, and gripper state
    anchored to Mega's actual UART milestone opcodes (CMD_BATCH_ACK,
    CMD_SCAN_WINDOW_OPEN, CMD_CAP_PROGRESS, CMD_AWAIT_NEXT, CMD_BATCH_COMPLETE)
    so the 3.5" LCD animates smoothly with zero extra AVR serial overhead.
    """

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z_state = "RAISED"       # RAISED | LOWER_PICK | LOWER_BIN
        self.gripper_state = "OPEN"   # OPEN | CLOSED
        self.held_color = None
        self.phase = "IDLE"
        self.occluded_y_mm = None
        self.scan_flash_until = 0.0

        self._seg_start_t = 0.0
        self._seg_dur = 0.0
        self._start_xy = (0.0, 0.0)
        self._target_xy = (0.0, 0.0)
        self._pick_xy = None
        self._bin_xy = None
        self._pick_color = None
        self._t_to_cap = 0.0
        self._t_to_bin = 0.0

    def start_pick_cycle(self, target_x: float, target_y: float, color: int, now: float):
        """Anchored when Mega acknowledges CMD_PICK_SINGLE (0x5B -> 0xA1)."""
        self.update(now)
        bin_y = BIN_Y_BY_COLOR.get(color, 130.0)
        self._start_xy = (self.x, self.y)
        self._pick_xy = (float(target_x), float(target_y))
        self._bin_xy = (355.0, bin_y)
        self._pick_color = color
        self._t_to_cap = max(0.15, compute_xy_move_time(self._pick_xy[0] - self.x, self._pick_xy[1] - self.y))
        self._t_to_bin = max(0.15, compute_xy_move_time(self._bin_xy[0] - self._pick_xy[0], self._bin_xy[1] - self._pick_xy[1]))
        self._seg_start_t = now
        self.phase = "PICK_CYCLE"
        self.occluded_y_mm = None

    def mark_bin_scan_window(self, bin_y: float, now: float):
        """Anchored on Mega CMD_SCAN_WINDOW_OPEN (0xA5): stationary at bin, dropping cap."""
        self.x = 355.0
        self.y = float(bin_y)
        self.z_state = "LOWER_BIN"
        self.gripper_state = "CLOSED"
        self.held_color = self._pick_color
        self.phase = "BIN_DROP"
        self.occluded_y_mm = float(bin_y)
        self.scan_flash_until = now + 0.45
        self._seg_start_t = now

    def mark_await_next(self, at_xy: tuple, now: float):
        """Anchored on Mega CMD_AWAIT_NEXT (0xA6)."""
        self.x, self.y = float(at_xy[0]), float(at_xy[1])
        self.z_state = "RAISED"
        self.gripper_state = "OPEN"
        self.held_color = None
        self.phase = "AWAIT_NEXT"
        self.occluded_y_mm = None

    def start_park(self, now: float, for_scan: bool = True):
        """Anchored on CMD_PARK_FOR_SCAN (0xB5) or CMD_FINISH_SESSION (0xB6)."""
        self.update(now)
        self._start_xy = (self.x, self.y)
        self._target_xy = (0.0, 0.0)
        self._seg_dur = max(0.15, compute_xy_move_time(self.x, self.y))
        self._seg_start_t = now
        self.z_state = "RAISED"
        self.gripper_state = "OPEN"
        self.held_color = None
        self.occluded_y_mm = None
        self.phase = "PARKING_SCAN" if for_scan else "PARKING_HOME"

    def mark_idle(self, now: float):
        self.x, self.y = 0.0, 0.0
        self.z_state = "RAISED"
        self.gripper_state = "OPEN"
        self.held_color = None
        self.occluded_y_mm = None
        self.phase = "IDLE"

    def mark_fault(self, label: str, now: float):
        self.update(now)
        self.phase = label
        self.occluded_y_mm = None

    def update(self, now: float) -> dict:
        dt = max(0.0, now - self._seg_start_t)

        if self.phase == "PICK_CYCLE" and self._pick_xy is not None:
            t1 = self._t_to_cap
            t2 = t1 + PICK_LOWER_S
            t3 = t2 + PICK_GRIP_S
            t4 = t3 + PICK_LIFT_S
            t5 = t4 + self._t_to_bin

            if dt < t1:
                u = _smoothstep(dt / t1)
                self.x = self._start_xy[0] + (self._pick_xy[0] - self._start_xy[0]) * u
                self.y = self._start_xy[1] + (self._pick_xy[1] - self._start_xy[1]) * u
                self.z_state = "RAISED"
                self.gripper_state = "OPEN"
                self.held_color = None
            elif dt < t2:
                self.x, self.y = self._pick_xy
                self.z_state = "LOWER_PICK"
                self.gripper_state = "OPEN"
                self.held_color = None
            elif dt < t3:
                self.x, self.y = self._pick_xy
                self.z_state = "LOWER_PICK"
                self.gripper_state = "CLOSED"
                self.held_color = self._pick_color
            elif dt < t4:
                self.x, self.y = self._pick_xy
                self.z_state = "RAISED"
                self.gripper_state = "CLOSED"
                self.held_color = self._pick_color
            elif dt < t5:
                u = _smoothstep((dt - t4) / self._t_to_bin)
                self.x = self._pick_xy[0] + (self._bin_xy[0] - self._pick_xy[0]) * u
                self.y = self._pick_xy[1] + (self._bin_xy[1] - self._pick_xy[1]) * u
                self.z_state = "RAISED"
                self.gripper_state = "CLOSED"
                self.held_color = self._pick_color
            else:
                self.x, self.y = self._bin_xy
                self.z_state = "RAISED"
                self.gripper_state = "CLOSED"
                self.held_color = self._pick_color

        elif self.phase == "BIN_DROP":
            if dt < BIN_LOWER_S:
                self.z_state = "LOWER_BIN"
                self.gripper_state = "CLOSED"
                self.held_color = self._pick_color
            elif dt < BIN_LOWER_S + BIN_RELEASE_S:
                self.z_state = "LOWER_BIN"
                self.gripper_state = "OPEN"
                self.held_color = None
            else:
                self.z_state = "RAISED"
                self.gripper_state = "OPEN"
                self.held_color = None

        elif self.phase in ("PARKING_SCAN", "PARKING_HOME"):
            u = _smoothstep(dt / max(0.05, self._seg_dur))
            self.x = self._start_xy[0] + (self._target_xy[0] - self._start_xy[0]) * u
            self.y = self._start_xy[1] + (self._target_xy[1] - self._start_xy[1]) * u

        return {
            "x": self.x,
            "y": self.y,
            "z": self.z_state,
            "gripper": self.gripper_state,
            "held_color": self.held_color,
            "phase": self.phase,
            "occluded_y_mm": self.occluded_y_mm,
            "scan_flash": now < self.scan_flash_until,
        }


class CapSorterLCDHUD:
    """
    Non-blocking 480x320 framebuffer HUD & resistive-touch operator interface.
    Safe for read-only OverlayFS (zero disk writes) and 512 MB Pi Zero 2 W RAM.
    """

    def __init__(
        self,
        fb_device: str = "/dev/fb0",
        touch_device: str = "auto",
        fps: float = 15.0,
        on_pause_toggle=None,
        on_rearm=None,
        touch_cal: tuple = (200, 3900, 200, 3900),
        touch_swap_xy: bool = False,
        touch_invert_x: bool = False,
        touch_invert_y: bool = False,
    ):
        self.fb_device = fb_device
        self.touch_device = touch_device
        self.frame_interval_s = 1.0 / max(1.0, float(fps))
        self.on_pause_toggle = on_pause_toggle
        self.on_rearm = on_rearm
        self.touch_cal = tuple(float(v) for v in touch_cal)
        self.touch_swap_xy = bool(touch_swap_xy)
        self.touch_invert_x = bool(touch_invert_x)
        self.touch_invert_y = bool(touch_invert_y)
        self._fb_info = probe_framebuffer_info(self.fb_device)
        self._fb_bpp = self._fb_info.bpp

        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._render_thread = None
        self._touch_thread = None

        self.predictor = KinematicGantryPredictor()
        self.view_mode = "MAP"  # "MAP" | "CAM"
        self.paused = False
        self.state = {
            "controller_state": "BOOTING",
            "link_status": "WAIT_MEGA",
            "serial_port": "auto",
            "caps": [],
            "active_target": None,
            "quarantined": [],
            "bin_counts": {COLOR_RED: 0, COLOR_YELLOW: 0, COLOR_GREEN: 0, COLOR_BLUE: 0},
            "scans": 0,
            "picked": 0,
            "fault_banner": "",
            "cam_thumb": None,
        }

    def update_state(self, event: str, payload: dict = None, now: float = None):
        """Thread-safe update called from PiZeroVisionController milestones."""
        payload = payload or {}
        t = time.monotonic() if now is None else float(now)

        with self._lock:
            if "serial_port" in payload:
                self.state["serial_port"] = str(payload["serial_port"])
            if "link_status" in payload:
                self.state["link_status"] = str(payload["link_status"])
            if "caps" in payload:
                self.state["caps"] = [dict(c) for c in payload["caps"]]
            if "quarantined" in payload:
                self.state["quarantined"] = [dict(q) for q in payload["quarantined"]]
            if "scans" in payload:
                self.state["scans"] = int(payload["scans"])
            if "picked" in payload:
                self.state["picked"] = int(payload["picked"])
            if "cam_frame" in payload and payload["cam_frame"] is not None:
                self.state["cam_thumb"] = cv2.resize(
                    payload["cam_frame"], (BOARD_PX, BOARD_PX), interpolation=cv2.INTER_AREA
                )

            if event == "READY":
                self.state["controller_state"] = "READY"
                self.state["link_status"] = "ONLINE"
                self.state["fault_banner"] = ""
                self.state["active_target"] = None
                self.predictor.mark_idle(t)
            elif event == "SCAN_INITIAL":
                self.state["controller_state"] = "SCANNING"
                self.predictor.scan_flash_until = t + 0.35
            elif event == "TARGET_DISPATCHED":
                tgt = payload.get("target")
                self.state["active_target"] = dict(tgt) if tgt else None
                self.state["controller_state"] = "PICKING"
                if tgt:
                    self.predictor.start_pick_cycle(tgt["boardX_mm"], tgt["boardY_mm"], tgt["color"], t)
            elif event == "SCAN_WINDOW_OPEN":
                bin_y = float(payload.get("bin_y", 130.0))
                self.state["controller_state"] = "BIN_SCAN"
                self.predictor.mark_bin_scan_window(bin_y, t)
            elif event == "CAP_SORTED":
                col = payload.get("color")
                if col in self.state["bin_counts"]:
                    self.state["bin_counts"][col] += 1
            elif event == "AWAIT_NEXT":
                at_xy = payload.get("at_xy", (self.predictor.x, self.predictor.y))
                self.predictor.mark_await_next(at_xy, t)
                self.state["controller_state"] = "AWAIT_NEXT"
            elif event == "PARK_FOR_SCAN":
                self.state["controller_state"] = "VERIFY_PARK"
                self.predictor.start_park(t, for_scan=True)
            elif event == "FINISH_SESSION":
                self.state["controller_state"] = "PARKING"
                self.state["active_target"] = None
                self.predictor.start_park(t, for_scan=False)
            elif event == "SESSION_COMPLETE":
                self.state["controller_state"] = "IDLE"
                self.state["active_target"] = None
                self.predictor.mark_idle(t)
            elif event == "PAUSED":
                self.state["controller_state"] = "PAUSED"
                self.state["active_target"] = None
                self.predictor.mark_idle(t)
            elif event == "REARMING":
                self.state["controller_state"] = "HOMING"
                self.state["fault_banner"] = ""
            elif event in ("FAULT", "LINK_LOST", "TIMEOUT"):
                self.state["controller_state"] = event
                self.state["link_status"] = "OFFLINE" if event == "LINK_LOST" else "FAULT"
                self.state["fault_banner"] = payload.get("reason", event)
                self.predictor.mark_fault(event, t)

    def handle_touch_px(self, x_px: int, y_px: int):
        """Dispatches a calibrated (x_px, y_px) tap to HUD buttons."""
        btn = hit_test_touch_button(x_px, y_px)
        if btn == "pause":
            with self._lock:
                self.paused = not self.paused
                paused_now = self.paused
            if self.on_pause_toggle is not None:
                self.on_pause_toggle(paused_now)
        elif btn == "rearm":
            if self.on_rearm is not None:
                self.on_rearm()
        elif btn == "view":
            with self._lock:
                self.view_mode = "CAM" if self.view_mode == "MAP" else "MAP"
        return btn

    def render_frame_bgr(self, now: float = None) -> np.ndarray:
        """Renders a complete 480x320 BGR frame (usable headless in tests)."""
        t = time.monotonic() if now is None else float(now)
        img = np.full((LCD_HEIGHT, LCD_WIDTH, 3), COL_BG, dtype=np.uint8)

        with self._lock:
            pose = self.predictor.update(t)
            st = {
                "controller_state": self.state["controller_state"],
                "link_status": self.state["link_status"],
                "serial_port": self.state["serial_port"],
                "caps": list(self.state["caps"]),
                "active_target": dict(self.state["active_target"]) if self.state["active_target"] else None,
                "quarantined": list(self.state["quarantined"]),
                "bin_counts": dict(self.state["bin_counts"]),
                "scans": self.state["scans"],
                "picked": self.state["picked"],
                "fault_banner": self.state["fault_banner"],
                "cam_thumb": self.state["cam_thumb"],
                "view_mode": self.view_mode,
                "paused": self.paused,
            }

        # 1. Left 300x300 Workspace Panel (1.0 px/mm)
        if st["view_mode"] == "CAM" and st["cam_thumb"] is not None:
            img[BOARD_Y0:BOARD_Y0 + BOARD_PX, BOARD_X0:BOARD_X0 + BOARD_PX] = st["cam_thumb"]
        else:
            cv2.rectangle(
                img, (BOARD_X0, BOARD_Y0), (BOARD_X0 + BOARD_PX, BOARD_Y0 + BOARD_PX), COL_BOARD_BG, -1
            )
            # 10x10 reference grid (30 mm pitch)
            for mm in range(30, 300, 30):
                cv2.line(img, (BOARD_X0 + mm, BOARD_Y0), (BOARD_X0 + mm, BOARD_Y0 + BOARD_PX), COL_GRID, 1)
                cv2.line(img, (BOARD_X0, BOARD_Y0 + mm), (BOARD_X0 + BOARD_PX, BOARD_Y0 + mm), COL_GRID, 1)
            # Safe pick reachability box (18..282 mm)
            cv2.rectangle(
                img,
                (BOARD_X0 + int(SAFE_PICK_MIN_MM), BOARD_Y0 + int(SAFE_PICK_MIN_MM)),
                (BOARD_X0 + int(SAFE_PICK_MAX_MM), BOARD_Y0 + int(SAFE_PICK_MAX_MM)),
                COL_SAFE_LINE, 1,
            )

        cv2.rectangle(
            img, (BOARD_X0, BOARD_Y0), (BOARD_X0 + BOARD_PX, BOARD_Y0 + BOARD_PX), COL_BORDER, 1
        )

        # Bridge occlusion strip during bin-side re-scan
        if pose["occluded_y_mm"] is not None:
            y_top = max(0, int(round(pose["occluded_y_mm"] - BRIDGE_OCCLUSION_HALF_MM)))
            y_bot = min(BOARD_PX, int(round(pose["occluded_y_mm"] + BRIDGE_OCCLUSION_HALF_MM)))
            overlay = img[BOARD_Y0 + y_top:BOARD_Y0 + y_bot, BOARD_X0:BOARD_X0 + BOARD_PX].copy()
            cv2.rectangle(overlay, (0, 0), (BOARD_PX, max(1, y_bot - y_top)), (20, 60, 110), -1)
            cv2.addWeighted(
                overlay, 0.35,
                img[BOARD_Y0 + y_top:BOARD_Y0 + y_bot, BOARD_X0:BOARD_X0 + BOARD_PX], 0.65,
                0.0,
                img[BOARD_Y0 + y_top:BOARD_Y0 + y_bot, BOARD_X0:BOARD_X0 + BOARD_PX],
            )

        # Draw quarantined spots
        for q in st["quarantined"]:
            qx = BOARD_X0 + int(round(q["x"]))
            qy = BOARD_Y0 + int(round(q["y"]))
            cv2.circle(img, (qx, qy), 14, COL_RED_ALERT, 1)
            cv2.line(img, (qx - 6, qy - 6), (qx + 6, qy + 6), COL_RED_ALERT, 1)
            cv2.line(img, (qx + 6, qy - 6), (qx - 6, qy + 6), COL_RED_ALERT, 1)

        # Draw detected bottle caps at continuous (X, Y) mm coordinates
        active = st["active_target"]
        for cap in st["caps"]:
            cx = BOARD_X0 + int(round(cap["boardX_mm"]))
            cy = BOARD_Y0 + int(round(cap["boardY_mm"]))
            col_bgr = CAP_BGR.get(cap["color"], COL_WHITE)
            cv2.circle(img, (cx, cy), int(CAP_RADIUS_MM), col_bgr, -1)
            ring_col = COL_WHITE if cap.get("in_safe_bounds", True) else COL_RED_ALERT
            cv2.circle(img, (cx, cy), int(CAP_RADIUS_MM), ring_col, 1)

        # Highlight active pick target + X-jaw descent corridors
        if active is not None:
            tx = BOARD_X0 + int(round(active["boardX_mm"]))
            ty = BOARD_Y0 + int(round(active["boardY_mm"]))
            cv2.circle(img, (tx, ty), 18, COL_CYAN, 2)
            # Left & right open jaw footprint boxes (+/-18..26 mm along X)
            cv2.rectangle(
                img,
                (tx - int(JAW_OUTER_HALF_MM), ty - int(JAW_WIDTH_HALF_MM)),
                (tx - int(JAW_INNER_HALF_MM), ty + int(JAW_WIDTH_HALF_MM)),
                COL_CYAN, 1,
            )
            cv2.rectangle(
                img,
                (tx + int(JAW_INNER_HALF_MM), ty - int(JAW_WIDTH_HALF_MM)),
                (tx + int(JAW_OUTER_HALF_MM), ty + int(JAW_WIDTH_HALF_MM)),
                COL_CYAN, 1,
            )

        # Scan flash line when camera captures
        if pose["scan_flash"]:
            cv2.rectangle(
                img, (BOARD_X0 + 1, BOARD_Y0 + 1), (BOARD_X0 + BOARD_PX - 1, BOARD_Y0 + BOARD_PX - 1), COL_CYAN, 2
            )

        # Draw Animated Moving Y-Bridge & X-Carriage + Parallel Gripper
        gx_clamped = min(BOARD_PX, max(0, int(round(pose["x"]))))
        gy_clamped = min(BOARD_PX, max(0, int(round(pose["y"]))))
        gy_px = BOARD_Y0 + gy_clamped
        gx_px = BOARD_X0 + gx_clamped

        # Y-Bridge horizontal beam spanning the board
        cv2.line(img, (BOARD_X0, gy_px), (BOARD_X0 + BOARD_PX, gy_px), (120, 130, 145), 3)
        cv2.line(img, (BOARD_X0, gy_px), (BOARD_X0 + BOARD_PX, gy_px), COL_CYAN, 1)

        # Carriage & parallel jaws (visible over board, or indicator arrow when over bin at X=355)
        if pose["x"] <= 305.0:
            jaw_gap = 12 if pose["gripper"] == "CLOSED" else 18
            if pose["held_color"] is not None:
                cv2.circle(img, (gx_px, gy_px), 13, CAP_BGR.get(pose["held_color"], COL_WHITE), -1)
            car_col = COL_ORANGE if pose["z"] != "RAISED" else COL_CYAN
            cv2.rectangle(img, (gx_px - 10, gy_px - 10), (gx_px + 10, gy_px + 10), car_col, 2)
            # Two parallel gripper jaw fingers
            cv2.rectangle(img, (gx_px - jaw_gap - 5, gy_px - 8), (gx_px - jaw_gap, gy_px + 8), COL_WHITE, -1)
            cv2.rectangle(img, (gx_px + jaw_gap, gy_px - 8), (gx_px + jaw_gap + 5, gy_px + 8), COL_WHITE, -1)
        else:
            # Gantry is at external color bin (X = 355 mm)
            cv2.circle(img, (BOARD_X0 + BOARD_PX - 6, gy_px), 6, COL_EMERALD, -1)

        # Fault banner overlay on workspace if faulted
        if st["fault_banner"]:
            cv2.rectangle(img, (BOARD_X0 + 12, BOARD_Y0 + 122), (BOARD_X0 + 288, BOARD_Y0 + 178), (22, 22, 120), -1)
            cv2.rectangle(img, (BOARD_X0 + 12, BOARD_Y0 + 122), (BOARD_X0 + 288, BOARD_Y0 + 178), COL_RED_ALERT, 2)
            cv2.putText(
                img, "HALT / FAULT", (BOARD_X0 + 82, BOARD_Y0 + 145),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, COL_WHITE, 2, cv2.LINE_AA,
            )
            cv2.putText(
                img, st["fault_banner"][:28], (BOARD_X0 + 22, BOARD_Y0 + 168),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, COL_AMBER, 1, cv2.LINE_AA,
            )

        # 2. Right Telemetry, Bins & Touch Control Sidebar (160x300)
        cv2.rectangle(img, (SIDEBAR_X0, 10), (SIDEBAR_X0 + SIDEBAR_W, 206), COL_PANEL, -1)
        cv2.rectangle(img, (SIDEBAR_X0, 10), (SIDEBAR_X0 + SIDEBAR_W, 206), COL_BORDER, 1)

        cv2.putText(img, "CAPSORTER V2", (SIDEBAR_X0 + 8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.48, COL_CYAN, 1, cv2.LINE_AA)
        link_col = COL_EMERALD if st["link_status"] == "ONLINE" else COL_RED_ALERT
        cv2.circle(img, (SIDEBAR_X0 + SIDEBAR_W - 12, 23), 5, link_col, -1)

        # 4 Color Bin Count Chips
        bin_w = (SIDEBAR_W - 16) // 4
        for idx, cid in enumerate((COLOR_RED, COLOR_YELLOW, COLOR_GREEN, COLOR_BLUE)):
            bx0 = SIDEBAR_X0 + 6 + idx * (bin_w + 1)
            by0 = 38
            cv2.rectangle(img, (bx0, by0), (bx0 + bin_w - 2, by0 + 36), COL_BG, -1)
            cv2.rectangle(img, (bx0, by0), (bx0 + bin_w - 2, by0 + 36), CAP_BGR[cid], 1)
            cv2.putText(img, CAP_LABEL[cid], (bx0 + 11, by0 + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.40, CAP_BGR[cid], 1, cv2.LINE_AA)
            cv2.putText(
                img, str(st["bin_counts"].get(cid, 0)), (bx0 + 10, by0 + 31),
                cv2.FONT_HERSHEY_SIMPLEX, 0.44, COL_WHITE, 1, cv2.LINE_AA,
            )

        # Telemetry rows
        rows = [
            ("ST",   "PAUSED" if st["paused"] else st["controller_state"][:11], COL_AMBER if st["paused"] else COL_CYAN),
            ("XY",   f"{pose['x']:5.1f},{pose['y']:5.1f}", COL_WHITE),
            ("Z",    pose["z"], COL_ORANGE if pose["z"] != "RAISED" else COL_EMERALD),
            ("GRIP", pose["gripper"], COL_ORANGE if pose["gripper"] == "CLOSED" else COL_WHITE),
            ("CAPS", f"{len(st['caps'])}  SC:{st['scans']}", COL_WHITE),
            ("TTL",  os.path.basename(st["serial_port"])[:14], link_col),
        ]
        y_row = 96
        for k, v, v_col in rows:
            cv2.putText(img, k, (SIDEBAR_X0 + 8, y_row), cv2.FONT_HERSHEY_SIMPLEX, 0.38, COL_MUTED, 1, cv2.LINE_AA)
            v_scale = 0.34 if len(v) > 12 else 0.39
            cv2.putText(img, v, (SIDEBAR_X0 + 48, y_row), cv2.FONT_HERSHEY_SIMPLEX, v_scale, v_col, 1, cv2.LINE_AA)
            y_row += 18

        # 3 Tactile Resistive Touch Buttons
        btn_specs = [
            ("pause", "RESUME" if st["paused"] else "PAUSE SORT", COL_AMBER if st["paused"] else COL_CYAN),
            ("rearm", "RE-ARM & HOME", COL_ORANGE),
            ("view",  f"VIEW: {st['view_mode']}", COL_EMERALD),
        ]
        for key, label, border_col in btn_specs:
            x0, y0, x1, y1 = TOUCH_BUTTONS[key]
            cv2.rectangle(img, (x0, y0), (x1, y1), COL_PANEL, -1)
            cv2.rectangle(img, (x0, y0), (x1, y1), border_col, 1)
            cv2.putText(
                img, label, (x0 + 14, y0 + (y1 - y0) // 2 + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.43, COL_WHITE, 1, cv2.LINE_AA,
            )

        return img

    def _detect_fb_bpp(self) -> int:
        self._fb_info = probe_framebuffer_info(self.fb_device)
        return self._fb_info.bpp

    def _blit_framebuffer(self, frame_bgr: np.ndarray):
        payload = pack_bgr_for_framebuffer(frame_bgr, self._fb_info)
        try:
            with open(self.fb_device, "r+b", buffering=0) as fb:
                fb.seek(0)
                fb.write(payload)
        except OSError:
            with open(self.fb_device, "wb") as fb:
                fb.write(payload)

    def _render_loop(self):
        while not self._stop_evt.is_set():
            t0 = time.monotonic()
            try:
                frame = self.render_frame_bgr(now=t0)
                if self.fb_device and os.path.exists(self.fb_device):
                    self._blit_framebuffer(frame)
            except OSError:
                pass
            elapsed = time.monotonic() - t0
            self._stop_evt.wait(max(0.01, self.frame_interval_s - elapsed))

    def _find_touch_input_device(self):
        if self.touch_device and self.touch_device != "auto":
            return self.touch_device
        for path in sorted(glob.glob("/sys/class/input/event*/device/name")):
            try:
                with open(path, "r", encoding="utf-8") as fh:
                    name = fh.read().strip().lower()
                if any(tok in name for tok in ("ads7846", "xpt2046", "touch")):
                    ev_name = path.split("/")[4]
                    return f"/dev/input/{ev_name}"
            except OSError:
                continue
        return None

    def _touch_loop(self):
        """
        Reads Linux evdev input_event structs directly from XPT2046 (/dev/input/event*)
        with zero external dependencies (EV_ABS=3, EV_KEY=1, BTN_TOUCH=330).
        """
        dev_path = self._find_touch_input_device()
        if not dev_path or not os.path.exists(dev_path):
            return
        # 64-bit Linux struct input_event: (timeval sec:q, usec:q, type:H, code:H, value:i) = 24B
        ev_fmt = "llHHi"
        ev_size = struct.calcsize(ev_fmt)
        raw_x, raw_y = 2048, 2048
        try:
            fd = os.open(dev_path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            return
        try:
            while not self._stop_evt.is_set():
                rlist, _, _ = select.select([fd], [], [], 0.2)
                if not rlist:
                    continue
                try:
                    data = os.read(fd, ev_size * 16)
                except BlockingIOError:
                    continue
                except OSError:
                    break
                for off in range(0, len(data) - (len(data) % ev_size), ev_size):
                    _, _, ev_type, code, value = struct.unpack(ev_fmt, data[off:off + ev_size])
                    if ev_type == 3:  # EV_ABS
                        if code == 0:    # ABS_X
                            raw_x = value
                        elif code == 1:  # ABS_Y
                            raw_y = value
                    elif ev_type == 1 and code == 330 and value == 0:  # BTN_TOUCH release
                        ax, ay = (raw_y, raw_x) if self.touch_swap_xy else (raw_x, raw_y)
                        x_min, x_max, y_min, y_max = self.touch_cal
                        nx = (ax - x_min) / max(1.0, (x_max - x_min))
                        ny = (ay - y_min) / max(1.0, (y_max - y_min))
                        if self.touch_invert_x:
                            nx = 1.0 - nx
                        if self.touch_invert_y:
                            ny = 1.0 - ny
                        px = int(min(LCD_WIDTH - 1, max(0, nx * LCD_WIDTH)))
                        py = int(min(LCD_HEIGHT - 1, max(0, ny * LCD_HEIGHT)))
                        self.handle_touch_px(px, py)
        finally:
            os.close(fd)

    def start(self):
        if self._render_thread is not None:
            return
        self._stop_evt.clear()
        self._render_thread = threading.Thread(target=self._render_loop, name="lcd_hud_render", daemon=True)
        self._render_thread.start()
        self._touch_thread = threading.Thread(target=self._touch_loop, name="lcd_hud_touch", daemon=True)
        self._touch_thread.start()

    def stop(self):
        self._stop_evt.set()
        if self._render_thread is not None:
            self._render_thread.join(timeout=1.0)
            self._render_thread = None
        if self._touch_thread is not None:
            self._touch_thread.join(timeout=1.0)
            self._touch_thread = None


def main():
    parser = argparse.ArgumentParser(description="CapSorter V2 3.5-inch SPI LCD Native HUD (480x320)")
    parser.add_argument("--fb-device", default="/dev/fb0", help="Linux framebuffer device (default: /dev/fb0)")
    parser.add_argument("--touch-device", default="auto", help="XPT2046 evdev input path or 'auto'")
    parser.add_argument("--touch-swap-xy", action="store_true", help="Swap XPT2046 X/Y touch axes")
    parser.add_argument("--touch-invert-x", action="store_true", help="Invert XPT2046 X touch axis")
    parser.add_argument("--touch-invert-y", action="store_true", help="Invert XPT2046 Y touch axis")
    parser.add_argument("--demo", action="store_true", help="Run standalone animated demo loop on LCD")
    parser.add_argument("--snapshot", help="Write one 480x320 PNG preview frame and exit")
    args = parser.parse_args()

    hud = CapSorterLCDHUD(
        fb_device=args.fb_device,
        touch_device=args.touch_device,
        touch_swap_xy=args.touch_swap_xy,
        touch_invert_x=args.touch_invert_x,
        touch_invert_y=args.touch_invert_y,
    )
    demo_caps = [
        {"capId": 1, "color": COLOR_RED,    "boardX_mm": 64.0,  "boardY_mm": 72.0,  "in_safe_bounds": True},
        {"capId": 2, "color": COLOR_YELLOW, "boardX_mm": 195.0, "boardY_mm": 98.0,  "in_safe_bounds": True},
        {"capId": 3, "color": COLOR_GREEN,  "boardX_mm": 115.0, "boardY_mm": 212.0, "in_safe_bounds": True},
        {"capId": 4, "color": COLOR_BLUE,   "boardX_mm": 248.0, "boardY_mm": 236.0, "in_safe_bounds": True},
    ]
    hud.update_state("READY", {"caps": demo_caps, "serial_port": "/dev/capsorter_mega", "scans": 1})
    hud.update_state("TARGET_DISPATCHED", {"target": demo_caps[0], "caps": demo_caps})

    if args.snapshot:
        frame = hud.render_frame_bgr(now=time.monotonic() + 0.8)
        cv2.imwrite(args.snapshot, frame)
        print(f"[OK] Saved 480x320 HUD snapshot -> {args.snapshot}")
        return

    if args.demo:
        print(f"[HUD] Running 480x320 demo on {args.fb_device} (Ctrl+C to exit)...")
        hud.start()
        try:
            idx = 0
            while True:
                tgt = demo_caps[idx % len(demo_caps)]
                hud.update_state("TARGET_DISPATCHED", {"target": tgt, "caps": demo_caps, "scans": idx + 1})
                time.sleep(3.2)
                hud.update_state("SCAN_WINDOW_OPEN", {"bin_y": BIN_Y_BY_COLOR[tgt["color"]]})
                time.sleep(0.85)
                hud.update_state("CAP_SORTED", {"color": tgt["color"]})
                idx += 1
        except KeyboardInterrupt:
            pass
        finally:
            hud.stop()


if __name__ == "__main__":
    main()
