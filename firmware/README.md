# CapSorter Firmware Vault

> **Status: Pre-Validation Engineering Firmware.** Static review fixes have been applied, but no physical bench tests have been completed yet. Do not treat this repository as production-validated until exact hardware SKUs are confirmed and the full bench validation checklist below has passed.

## Directory Structure

```
firmware/
├── README.md                 # Firmware documentation, pinned toolchain, safety & validation checklist
├── HARDWARE_BRINGUP_GUIDE.md # Step-by-step physical bench testing manual (Stages 0-8)
├── bringup/                  # Staged bench-test sketches & Python vision/UART/LCD tools
│   ├── mega_01_switches_estop/       # Stage 1: Zero-motion NC limit switch & E-stop verifier
│   ├── mega_02_servo_calibration/    # Stage 2: Interactive PCA9685 Z-axis & gripper tick calibrator
│   ├── mega_03_stepper_homing_test/  # Stage 3: Stepper polarity, homing, STEPS_PER_MM, offsets & bins
│   ├── pi_01_camera_detect_preview.py     # Stage 4: Live camera cap bounding-box & HSV preview
│   ├── pi_02_capture_and_mark_corners.py  # Stage 5: Checkerboard capture & 4-corner coordinate marker
│   ├── pi_03_verify_mm_coordinates.py     # Stage 6: Calibrated (X, Y) mm, parallax & clearance verifier
│   ├── pi_04_uart_link_test.py            # Stage 7A: Pi <-> Mega binary UART ping, 1-cap & cut tester
│   └── pi_05_lcd_touch_test.py            # Stage 7B: 3.5" SPI LCD framebuffer & XPT2046 touch calibrator
├── common/
│   └── packet_protocol.h     # Shared binary packet structures, handshake opcodes, CRC16-CCITT & limits
├── mega_motion/
│   └── mega_motion.ino       # Arduino Mega 2560 motion sequencer, X/Y/Z NC limits, E-stop sense & fail-safe halt
├── v1_esp32cam/
│   └── esp32cam_uart.ino     # ESP32-CAM (AI-Thinker) OV2640 QVGA RGB565 frame streamer (zero-init config + PSRAM)
├── v1_teensy41/
│   └── teensy_vision.cpp     # Teensy 4.1 HSV vision, 4-pt fiducial homography solver, area-sorted NMS & planner
├── v2_pizero2w/
│   ├── pi_zero_vision.py     # Raspberry Pi Zero 2 W OpenCV vision pipeline, USB-TTL auto-detect & planner
│   ├── lcd_hud.py            # Lightweight 480x320 native SPI LCD RGB565 HUD & XPT2046 touch UI
│   ├── 99-capsorter-mega.rules # Persistent udev symlink (/dev/capsorter_mega) for CH9102 / CP2102
│   ├── fisheye_calibrate.py  # Checkerboard intrinsic (fisheye / pinhole) + 4-corner homography calibration CLI
│   └── calibration.npz       # Explicitly marked uncalibrated placeholder (fails closed until bench calibrated)
└── tests/
    └── test_validation.py    # Automated protocol, USB-TTL, LCD HUD, end-to-end sim, bringup & doc-sync tests
```

## Target Hardware & Pin Mapping Summary

| Subsystem | Target Board | Processor / OS | Interface / Baud Rate | Logic Level & Safety |
| :--- | :--- | :--- | :--- | :--- |
| **V1 Camera Streamer** | ESP32-CAM (AI-Thinker, PSRAM) | ESP32-D0WDQ6 (240 MHz) | UART2 @ 921,600 Baud | **3.3V ONLY** |
| **V1 Vision & Planner** | Teensy 4.1 | NXP i.MX RT1062 Cortex-M7 (600 MHz) | Serial2 @ 921,600, Serial1 @ 115,200 | **3.3V ONLY — NO pins are 5V tolerant.** Route BOTH TX1 & RX1 through bidirectional 3.3V/5V shifter |
| **V2 Vision & HMI** | Raspberry Pi Zero 2 W + 3.5" SPI Touch LCD V3.0 | BCM2710A1 4× Cortex-A53 (1.0 GHz, OS Lite) | GPIO 1–26 SPI0 (`piscreen,drm` `/dev/fb0` + XPT2046); Micro-USB OTG → CH9102/CP2102 (`/dev/capsorter_mega`) @ 115,200 | **Power Pi via `PWR IN` Micro-USB** (LCD covers Header Pins 1–26). Route CH9102/CP2102 TXD/RXD through 3.3V/5V shifter (`LV` = converter `3V3`); **leave converter `+5V` disconnected!** |
| **Motion & Safety** | Arduino Mega 2560 Rev3 | Microchip ATmega2560 (16 MHz AVR) | Serial1 @ 115,200 Baud, I2C (Pins 20/21) | 5.0V Native (feed regulated 5V to `5V` pin, **NOT `VIN`**; avoid USB backfeed) |
| **Servo PWM Controller** | PCA9685 16-Channel | NXP PCA9685 I2C (Address 0x40) | I2C (Mega Pins 20 SDA, 21 SCL, Pin 9 OE) | VCC 5.0V (Branch C), V+ 5.0V/5A separate regulated rail (Branch B); verify terminal/MOSFET ratings |
| **Stepper Drivers** | 2× TMC2208 Carrier | Trinamic TMC2208 (MS1=GND, MS2=GND -> 1/8) | STEP/DIR/EN via Mega Pins 2,3,5,6,8 | Logic 5.0V, VMOT 12.0V fused (Branch A); inspect exact carrier `R_sense` before setting `V_ref` |

## Pinned Toolchain & Library Versions (Recommended Baseline)

Pin exact cores and libraries before building so `camera_config_t` fields, `AccelStepper` timing, and OpenCV APIs remain reproducible:

| Target | Core / Runtime | Libraries |
| :--- | :--- | :--- |
| **ESP32-CAM (V1)** | `arduino-esp32` **2.0.17** (Board: AI Thinker ESP32-CAM, PSRAM: Enabled) | Bundled `esp32-camera` 2.0.x |
| **Teensy 4.1 (V1)** | Teensyduino **1.59** (Arduino IDE 2.3.x / CLI) | Standard Teensy core |
| **Arduino Mega 2560 (V1 & V2)** | `arduino:avr` **1.8.6** | `AccelStepper` **1.64**, `Adafruit PWM Servo Driver Library` **3.0.2** |
| **Raspberry Pi Zero 2 W (V2)** | Raspberry Pi OS **Lite** Bookworm (64-bit, Python 3.11) | `opencv-python` 4.x, `numpy` 1.24+, `pyserial` 3.5, `picamera2` 0.3.x *(native OpenCV RGB565 HUD needs zero Desktop/Chromium)* |

## Critical Safety, Electrical & Protocol Specifications

1. **Hardwired Emergency Stop (Primary Safety Path)**: A dual-pole latching NC mushroom E-stop must **hard-cut 12V actuator power** feeding Branch A (TMC2208 `VMOT`) and Branch B (5V servo buck input) independently of firmware. Pole 2 connects as an NC auxiliary contact to **Mega Pin 25 (`PIN_ESTOP`)** so firmware latches `STATE_HARD_FAULT`, disables `PIN_STEPPER_EN` (`HIGH`) and `PIN_SERVO_OE` (`HIGH`), notifies the host (`0xEF`), and requires physical E-stop release plus explicit operator re-arm (`CMD_REARM_HOME` `0xB4`) and re-homing before any motion can resume. Account mechanically (counterbalance spring / compliant stop) for Z-axis loss of holding torque when servo power is removed.
2. **Polled NC Limit Switches (X, Y, Z)**: Mega Pins 22 (`X`), 23 (`Y`), and 24 (`Z`) sit on ATmega2560 Port A (`PA0`–`PA2`), which has **no external or pin-change interrupts**. Firmware polls all three switches in `loop()` using `millis()`-timed persistence debouncing (`LIMIT_DEBOUNCE_MS = 8 ms` + >=3 samples; `ESTOP_DEBOUNCE_MS = 4 ms` + >=2 samples). Under default `Z_LIMIT_IS_OVERTRAVEL = 1`, Pin 24 verifies Z overtravel-circuit health only (not physical Z-retracted confirmation); set `0` if a top-retracted confirmation switch is installed. Do not claim `<10 µs` ISR stop times; measure actual mechanical stop time and distance on the bench.
3. **Explicit Startup Handshake & Step-Bounded Homing**: Homing runs at `800 steps/s` (`20 mm/s` nominal) with explicit step bounds (`MAX_HOMING_SEEK_X_MM = 395 mm`, `MAX_HOMING_SEEK_Y_MM = 325 mm`) so a broken switch or belt slip cannot drive continuously into a hard stop. `HOMING_TIMEOUT_MS` is derived at `23,000 ms` (`395 mm / 20 mm/s = 19.75 s` + `3.25 s` margin). Vision hosts poll `CMD_QUERY_STATUS` (`0xB0`) and **wait for `CMD_STATUS_READY` (`0xB1`)** before sending any target. V1 accepts `REARM` over USB Serial (plus an optional Pin 2 pushbutton when `ENABLE_V1_REARM_BTN = 1`) to emit `CMD_REARM_HOME` (`0xB4`).
4. **Closed-Loop Bin-Side Re-Imaging & Continuous Coordinates**: Caps may sit anywhere on the continuous 300 × 300 mm open flat plywood plate (no cell-center snapping). Vision hosts dispatch one CRC16-CCITT verified `CapTarget` at a time via `PROTOCOL_SINGLE_MAGIC` (`0x5B`, 18 bytes total, with safe reachability bounds `18..282 mm`). Upon arriving over a color bin (`X = 355 mm`), Mega settles `120 ms` with Z raised and emits `CMD_SCAN_WINDOW_OPEN` (`0xA5`) so the overhead top-center camera captures a fresh vibration-free frame *in parallel* with the `800 ms` bin drop (`LOWER_BIN` → `RELEASE_CAP` → `RETRACT_Z`), immediately re-acquiring any neighboring caps bumped during the pick. V2 separates touching same-color caps via Euclidean distance-transform peak detection (`cv2.distanceTransform`), V1 uses two-pass chamfer peak splitting, both apply configurable radial cap-height parallax compensation, X-axis jaw-footprint clearance-first peeling (`blocked_sides = 0 → 1 → 2`), bridge-occlusion masking (`±35 mm` around `BIN_Y`) with mandatory clear-FOV verification parking (`CMD_PARK_FOR_SCAN` `0xB5` → `CMD_FINISH_SESSION` `0xB6`), and 3-attempt missed-pick quarantine to prevent infinite loops. Legacy multi-cap batches (`0x5A`, paced in 32-byte chunks with 2 ms pauses) remain supported.
5. **Dynamic Supervision, Guarded Heartbeat & Controlled Comms-Loss Finish-and-Park**: Realistic per-cap cycles at `70 mm/s` (`2800 steps/s`) and `150 mm/s²` range from **~5–13 s/cap** (including `1.92 s` fixed servo/settle dwells). V2 runs a guarded background heartbeat thread (`_GuardedHeartbeatSupervisor`) emitting `0xF0` every `250 ms` across camera captures and OpenCV passes while enforcing a bounded `3.5 s` vision liveness deadline (stopping automatically on deadlock or serial disconnect); V1 interleaves `0xF0` pings mid-stream inside `receiveFrameWithHeartbeat()`. On UART heartbeat loss (`>1.2 s`), if a scanned target is already actively dispatched (`inActiveCapCycle`), the Mega finishes **only that one active commanded cycle** (cap possession is unverified unless optional `HAS_GRIP_SENSOR = 1` on Pin 26 is wired, which skips/aborts blind bin drops on verified miss/slip), then parks cleanly at **`(0, 0)`** (`STATE_FAILSAFE_PARKING`), disables steppers, emits `CMD_RECOVERY_HALT`, and latches `STATE_FAILSAFE_HALT` requiring explicit operator re-arm + re-home (no 3-ping auto-recovery). Hardwired E-stop and NC limits remain active throughout.
6. **Power Architecture & Driver Current Formula**: The 12V SMPS feeds **three separate regulated branches sharing a common star ground** (NOT galvanically isolated). Feed regulated 5V to Mega's `5V` pin (NOT `VIN`, which requires 7–12V; guard against USB backfeed when flashing). Inspect the exact sense resistor `R_sense` on your TMC2208 carrier and compute run current using the ADI/Trinamic Rev 1.13 equation:
   ```text
   I_RMS = (0.325 V / (R_sense + 0.03 Ω)) * (1 / √2) * (V_ref / 2.5 V)
   ```
   Verify exact servo SKU voltage (original TowerPro MG90S is rated at `4.8 V` nominal), stall current, wire gauges, branch fuses, and PCA9685 `V+` terminal ratings.
7. **V2 3.5" SPI LCD (Bookworm OS Lite) & Micro-USB OTG CH9102/CP2102 Setup**:
   - **OS & Native HUD (Do NOT install Desktop/Chromium on 512 MB RAM):** Keep Raspberry Pi OS Lite (64-bit Bookworm) so background browser memory never starves the 250 ms heartbeat or 800 ms bin-side scan window. Configure `/boot/firmware/config.txt` with the kernel's native DRM overlay (do not use legacy `LCD-show` / `fbcp` scripts):
     ```ini
     dtparam=spi=on
     # Comment out vc4-kms-v3d if the 3.5" SPI LCD is the sole display:
     #dtoverlay=vc4-kms-v3d
     dtoverlay=piscreen,drm,speed=18000000,rotate=90
     ```
   - **Persistent USB-to-TTL Rule (CH9102 / CP2102):** Many affordable CP2102 listings ship with the **WCH CH9102** chip (VID `1a86`), which enumerates as `/dev/ttyUSB0` or `/dev/ttyACM0` under Bookworm. Install [`v2_pizero2w/99-capsorter-mega.rules`](v2_pizero2w/99-capsorter-mega.rules) so both CH9102 and genuine Silicon Labs CP2102 bind cleanly to `/dev/capsorter_mega`:
     ```bash
     sudo cp firmware/v2_pizero2w/99-capsorter-mega.rules /etc/udev/rules.d/
     sudo udevadm control --reload-rules && sudo udevadm trigger
     python3 firmware/v2_pizero2w/pi_zero_vision.py --enable-lcd --fb-device /dev/fb0
     ```
   - **Wiring & Electrical Safety:** Power the Pi Zero 2 W through its **`PWR IN` Micro-USB port** (≥20 AWG from Branch C 5V/3A buck) because the 3.5" LCD covers Header Pins 1–26. Plug the Official Micro-USB OTG adapter + CH9102/CP2102 module into the inner **`USB` data port** (leaving the 22-pin CSI port for the 8MP IMX219 camera; a USB webcam now requires a USB hub). Route CH9102/CP2102 `TXD` & `RXD` through the 4-channel bidirectional level shifter (`LV` ← converter `3V3`, `HV` ← Mega `5V`, shared `GND`) to Mega `Serial1` (Pins 19/18). **NEVER connect the converter's `+5V` pin to the Mega `5V` rail.**
8. **Fail-Closed Camera Calibration**:
   - **V2**: Run `fisheye_calibrate.py` with real checkerboard images and measured board fiducials for your exact camera and resolution:
     ```bash
     # 8MP IMX219 160-deg CSI camera
     python3 fisheye_calibrate.py --camera-type picamera2_imx219 --lens-model fisheye \
       --checkerboard-glob "calib_csi/*.png" --pattern 9x6 --square-mm 25 \
       --corners "u0,v0;u1,v1;u2,v2;u3,v3" --out calibration_csi_imx219.npz

     # Standard USB webcam (separate pinhole calibration)
     python3 fisheye_calibrate.py --camera-type usb_v4l2 --lens-model pinhole \
       --checkerboard-glob "calib_usb/*.png" --pattern 9x6 --square-mm 25 \
       --corners "u0,v0;u1,v1;u2,v2;u3,v3" --out calibration_usb.npz
     ```
     `pi_zero_vision.py` fails closed if calibration is missing, uncalibrated, or mismatched to the active camera/resolution.
   - **V1**: Measure the 4 board corner fiducials on the installed ESP32-CAM, update `BOARD_FIDUCIALS_PX` in `teensy_vision.cpp`, verify grid accuracy across board edges, and only then set `V1_CALIBRATION_VERIFIED` to `1`.

## Required Bench Validation Before Relying on Either Design

- [ ] Confirm exact hardware SKUs (SMPS, buck converters, TMC2208 carrier & `R_sense`, NEMA 17, MG90S servo voltage/torque, fuses, wire gauges, connectors) and compile with the pinned cores/libraries above.
- [ ] Measure all power rails, transients, motor current, and driver/servo temperatures under simultaneous load (including Pi `PWR IN` 5V voltage drop with 3.5" SPI LCD backlight + IMX219 + CH9102 active; confirm `vcgencmd get_throttled` = `0x0`); verify UART logic levels at Mega `RX1` (5V) and CH9102 `RXD` (3.3V) with a multimeter/oscilloscope and confirm CH9102 `+5V` is disconnected.
- [ ] Verify `/dev/fb0` (480×320 RGB565), XPT2046 resistive touch calibration (`lcd_hud.py --demo`), `/dev/capsorter_mega` udev symlink, and clean USB-TTL unplug/replug `LINK_LOST` recovery + touchscreen `RE-ARM & HOME` (`0xB4`).
- [ ] Test hardwired E-stop and X/Y/Z limit switches during motion (both open and shorted wiring), measure stop time/distance, inspect Z-axis behavior when actuator power is cut, and verify deliberate re-arm + re-home is required.
- [ ] Test homing from multiple starting points including near maximum X/Y travel and verify no host sends motion data before `CMD_STATUS_READY`.
- [ ] Time short and long moves, run batches of increasing size through completion, and drop UART at every pick-and-place stage (including empty/missed picks) to confirm safe halt and zero unintended motion on link return.
- [ ] Calibrate each installed camera (CSI and/or USB separately, including `--camera-height-mm` and `--cap-height-mm` for corner parallax), verify mapped coordinate accuracy across a measured grid including edges and lighting changes, and measure physical `STEPS_PER_MM`, `TOOL_OFFSET_X/Y`, `BRIDGE_OCCLUSION_HALF_MM`, and X-jaw clearance footprint under load.
- [ ] Bench-test touching same-color caps, deliberately nudge a neighboring cap 30 mm mid-cycle to confirm bin-side re-acquisition, test a cap under a bin's bridge strip to verify clear-FOV park reveal, and tape one cap down to confirm 3-attempt quarantine prevents infinite loops.
