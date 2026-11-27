# CapSorter Firmware, Safety & Documentation Remediation Plan

## Overview

This document records the line-by-line engineering audit and implemented remediations across all CapSorter V1/V2 firmware files, Stage 3 bring-up tooling, V2 system architecture poster, research manuscript, HTML code vaults, and automated SIL test suite.

All **12 reported issues** plus **2 additional findings** (Stage 3 homing seek bounds and stale claims in `website/assets/system_poster_v2.jpg`) were verified and remediated.

---

## Audit & Remediation Matrix

| # | Area | Severity | Status | Files Updated | Implemented Fix |
| :-: | :--- | :---: | :---: | :--- | :--- |
| **1** | Homing Overtravel Guard | **P0 · Critical** | **Fixed** | `firmware/mega_motion/mega_motion.ino`, `firmware/bringup/mega_03_stepper_homing_test/mega_03_stepper_homing_test.ino` | Zeroed steppers before negative seek, enforced axis step bounds `MAX_HOMING_SEEK_X_MM = 395 mm` / `MAX_HOMING_SEEK_Y_MM = 325 mm` (`MAX_TRAVEL + 25 mm`), derived `HOMING_TIMEOUT_MS = 23000UL`, and checked `ENABLE_MAX_ENDSTOPS` during pre-homing backoff. |
| **2** | Switch Debounce Timing & V2 Poster Drift | **P0 · Critical** | **Fixed** | `firmware/mega_motion/mega_motion.ino`, `website/assets/system_poster_v2.jpg` | Replaced loop-count-only sampling with `millis()`-timed persistence filters (`ESTOP_DEBOUNCE_MS = 4UL` + >=2 samples; `LIMIT_DEBOUNCE_MS = 8UL` + >=3 samples). Regenerated `website/assets/system_poster_v2.jpg` from verified specs removing stale interrupt, 1/16 microstep, isolation, and RAMPS pin claims. |
| **3** | Z-Retract Verification Semantics | **P1 · High** | **Fixed** | `firmware/mega_motion/mega_motion.ino` | Renamed `isZRetractedSafeForXY()` to `isZSafeForXYMotion()` with explicit mode-specific fault messages (`zUnsafeReason()`) and boot diagnostic clarifying that `Z_LIMIT_IS_OVERTRAVEL = 1` verifies Pin 24 overtravel-circuit health only (not physical Z-retracted position proof). |
| **4** | Unverified Cap Possession & Dead `isCapVerifiedHeld()` | **P1 · High** | **Fixed** | `firmware/mega_motion/mega_motion.ino` | Added `capPossessionVerified` state, wired `isCapVerifiedHeld()` into `STATE_GRIP_CAP` (skips false bin transit on verified miss) and `STATE_LIFT_Z` (aborts bin transit cleanly on mid-lift slip when `HAS_GRIP_SENSOR == 1`), and clarified open-loop/unverified wording when `HAS_GRIP_SENSOR == 0`. |
| **5** | Stage 3 Manual Jog Command Safety Gaps | **P0 · Critical** | **Fixed** | `firmware/bringup/mega_03_stepper_homing_test/mega_03_stepper_homing_test.ino` | Added shared `canAcceptMotionCmd()` safety gate (`!isEstop() && !faulted && steppersEnabled`) across `x`, `y`, `go`, and all servo commands; cleared pending targets and disabled `PIN_SERVO_OE` on `dis`; synced targets on `en`; clamped relative jogs (`[0, MAX_TRAVEL]` homed, `±50 mm` unhomed); added step-bounded homing. |
| **6** | V1 BFS Queue Overflow Silent Corruption | **P1 · High** | **Fixed** | `firmware/v1_teensy41/teensy_vision.cpp` | Doubled `BFS_QUEUE_SIZE` to `8192` (`32 KiB` DMAMEM -> `375.0 KiB` total DMAMEM, `36.6%` of 1024 KiB SRAM), checked queue capacity before setting visited bit (`0x80`), tracked `componentOverflow`, and rejected corrupted blobs instead of emitting truncated centroids. |
| **7** | V1 Reports `EXEC_COMPLETE` on Capture Failure | **P0 · Critical** | **Fixed** | `firmware/v1_teensy41/teensy_vision.cpp` | Added `EXEC_CAPTURE_FAULT` enum value and bounded `captureAndSegmentWithRetry(..., maxAttempts=2)`. Scan #0 failure and clear-FOV verification failure now return `EXEC_CAPTURE_FAULT` (parking Mega safely first when needed) instead of falsely returning `EXEC_COMPLETE`. |
| **8** | V1 Retry Tracking Capped at 16 Zones | **P2 · Medium** | **Fixed** | `firmware/v1_teensy41/teensy_vision.cpp` | Expanded `MAX_RETRY_RECORDS` from `16` to `MAX_BATCH_CAPS` (`100`), added slot recycling for cleared non-quarantined entries, and triggered bounded `EXEC_FAULT_HALT` abort if the table is ever exhausted. |
| **9** | V1 Missing Operator Re-Arm Command Path | **P1 · High** | **Fixed** | `firmware/v1_teensy41/teensy_vision.cpp` | Added `sendRearmToMega()` (`CMD_REARM_HOME` `0xB4`) and `pollOperatorRearmInput()` supporting both USB Serial `REARM` command (always active) and optional debounced physical pushbutton on Pin 2 (`ENABLE_V1_REARM_BTN = 0` default). |
| **10** | V2 Heartbeat Shares Thread with Camera & OpenCV | **P1 · High** | **Fixed** | `firmware/v2_pizero2w/pi_zero_vision.py` | Implemented `_GuardedHeartbeatSupervisor` background thread feeding `0xF0` every `250 ms` under `_uart_lock` with a strict bounded `MAX_VISION_STAGE_STALL_S = 3.5 s` liveness deadline (stopping automatically on real deadlock or serial error), plus per-frame `capture_ms`/`detect_ms`/`max_vision_ms` timing instrumentation. |
| **11** | V2 LCD Framebuffer Assumes Packed Layout | **P2 · Medium** | **Fixed** | `firmware/v2_pizero2w/lcd_hud.py` | Implemented `FramebufferInfo`, `probe_framebuffer_info()` (`FBIOGET_VSCREENINFO` + `FBIOGET_FSCREENINFO` Linux ioctls with sysfs fallback), and `pack_bgr_for_framebuffer()` handling dynamic resize, RGB/BGR 16/32-bpp bitfields, and padded `line_length` row strides. |
| **12** | Calibration Accepts Invalid Corner Quads | **P1 · High** | **Fixed** | `firmware/v2_pizero2w/fisheye_calibrate.py`, `firmware/bringup/pi_02_capture_and_mark_corners.py`, `firmware/v1_teensy41/teensy_vision.cpp` | Added strict quadrilateral validation: image bounds, edge length (`>= 40 px`), opposite-edge ratio (`[0.4, 2.5]`), convexity, clockwise winding (`TL -> TR -> BR -> BL`), minimum area, interior corner angles (`[40°, 140°]`), spatial orientation, and optional independent interior check-point error verification (`--check-points-px` / `--check-points-mm`). |

---

## Hardware Configuration Defaults

Per baseline BOM alignment:
- `ENABLE_MAX_ENDSTOPS = 0` (set to `1` when Pins 27/28 positive-end NC limits are wired)
- `Z_LIMIT_IS_OVERTRAVEL = 1` (set to `0` if Pin 24 is fitted as a dedicated top-retracted confirmation switch)
- `HAS_GRIP_SENSOR = 0` (set to `1` when Pin 26 active-LOW cap-presence sensor is fitted)
- `ENABLE_V1_REARM_BTN = 0` (USB Serial `REARM` always active; set to `1` when a panel button is wired to Teensy Pin 2)

---

## Verification Evidence

- **Automated SIL & Structural Verification:** `python -m unittest discover -s firmware/tests -v` (expanded from 28 to 36+ targeted regression tests covering every issue above plus verbatim HTML code-vault parity and banned-claim documentation guards).
- **Physical Bench Checklist (Stages 0–8):** Verify homing step bound, limit stopping distance at 70 mm/s, Stage 3 `dis -> go -> en` lockout, V2 Pi Zero 2 W worst-case `max_vision_ms < 800 ms`, and native `/dev/fb0` LCD stride/bitfield rendering during hardware commissioning.
