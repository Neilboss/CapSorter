# CapSorter — Step-by-Step Hardware Bring-Up & Physical Testing Guide

> **Why this guide exists:** Never upload the final autonomous sorting firmware to untested hardware on day one. Follow Stages **0 → 8** in order so you verify power safety, switches, servos, steppers, camera vision, millimeter accuracy, UART link, and touchscreen **one isolated subsystem at a time** before running full autonomous sorting.

---

## Master Roadmap (Follow in Order)

| Stage | Subsystem Tested | Device | Code File to Upload / Run | Motor Power (12V)? | Pass Gate Before Next Stage |
| :---: | :--- | :--- | :--- | :---: | :--- |
| **0** | Power rails, E-Stop cut, Level Shifter, TMC2208 `V_ref` | Bench / Multimeter | *(None — MCUs disconnected)* | Yes (tuned) | All voltages exact, E-stop cuts 12V to `0.0 V`, `V_ref` set |
| **1** | X/Y/Z NC Limit Switches & E-Stop Pole 2 | Arduino Mega 2560 | `bringup/mega_01_switches_estop/mega_01_switches_estop.ino` | **OFF** | All 4 read `LOW` at rest and flip `HIGH` when pressed |
| **2** | Z-Axis & Gripper Servos (PCA9685) | Arduino Mega 2560 | `bringup/mega_02_servo_calibration/mega_02_servo_calibration.ino` | 5V Servo rail | Smooth raised/pick/bin/open/close ticks recorded |
| **3** | Stepper Direction, Homing, `STEPS_PER_MM`, `TOOL_OFFSET`, Bins | Arduino Mega 2560 | `bringup/mega_03_stepper_homing_test/mega_03_stepper_homing_test.ino` | **ON** | Homes cleanly, ruler error `< 1 mm`, offsets & bins recorded |
| **4** | Pi OS Setup, Camera Focus, Lighting & Live Cap Boxes | Pi Zero 2 W | `bringup/pi_01_camera_detect_preview.py` | **OFF** | Colored bounding squares lock onto R/Y/G/B caps reliably |
| **5** | Lens Calibration & 4-Corner Coordinate Marking | Pi Zero 2 W | `bringup/pi_02_capture_and_mark_corners.py` | **OFF** | Checkerboard RMS `< 1.0 px`, green 300×300 mm grid aligns flush |
| **6** | Calibrated `(X, Y)` mm, Parallax, Split & Clearance | Pi Zero 2 W | `bringup/pi_03_verify_mm_coordinates.py` | **OFF** | Ruler check `< ±2.5 mm` across board; touching caps split cleanly |
| **7** | Final Mega Upload + Pi↔Mega UART + 3.5" LCD Touch | Mega + Pi Zero 2 W | `mega_motion/mega_motion.ino`, `bringup/pi_04_uart_link_test.py`, `bringup/pi_05_lcd_touch_test.py` | **ON** | `READY` ping, 1-cap pick, UART-cut park-and-halt, touch aligned |
| **8** | Full Autonomous Closed-Loop Build & Acceptance Tests | Full System | `v2_pizero2w/pi_zero_vision.py` (or V1 Teensy + ESP32-CAM) | **ON** | All 5 physical acceptance tests pass |

> **Golden Safety Rules:**
> 1. Keep one hand on the **physical latching E-Stop mushroom** during every Stage 2, 3, 7, and 8 test.
> 2. **NEVER** connect the CH9102 / CP2102 USB-TTL converter's `+5V` pin to the Arduino Mega `5V` rail.
> 3. **NEVER** power the Arduino Mega from the `5V` rail and USB PC cable simultaneously unless you have a diode/power-cut USB data cable (unplug 12V SMPS when flashing Mega over USB!).
> 4. Always unplug 12V power before connecting or disconnecting TMC2208 stepper motor wires.

---

## Stage 0 — Power Rails, E-Stop, Level Shifter & TMC2208 `V_ref`

**Goal:** Prove every power rail and logic voltage is safe with a multimeter **before** plugging in your Arduino Mega or Raspberry Pi Zero 2 W.

### Step 0.1 — Verify 12V SMPS & Hardwired E-Stop (Pole 1)
1. Disconnect the Arduino Mega, Raspberry Pi, PCA9685, and TMC2208 drivers.
2. Measure the 12V SMPS output: should read **`12.0 V ± 0.3 V`**.
3. Press the latching **E-Stop mushroom**:
   - **Branch A** (TMC2208 `VMOT` feed) must drop to **`0.0 V`**.
   - **Branch B** (5V servo buck input) must drop to **`0.0 V`**.
   - **Branch C** (5V logic buck for Pi + Mega) stays energized so the LCD/Mega can report `HARD_FAULT` cleanly.
4. Twist-release the E-Stop mushroom and confirm Branches A & B return to `12.0 V`.

### Step 0.2 — Adjust Both Buck Converters (Before Attaching Loads)
1. **Branch B (Servo Buck → PCA9685 `V+`):** Adjust potentiometer to **`4.8 V – 5.0 V`** (check your exact MG90S SKU datasheet — original TowerPro MG90S is rated for `4.8 V` nominal).
2. **Branch C (Logic Buck → Pi `PWR IN` Micro-USB & Mega `5V` pin):** Adjust potentiometer to **`5.10 V`** (provides headroom under Pi + 3.5" SPI LCD load; feed Mega's **`5V`** pin, **NOT `VIN`**).
3. Verify continuity (`< 0.5 Ω`) between SMPS `GND`, Branch A `GND`, Branch B `GND`, and Branch C `GND` at the common **star ground** bus (shared reference ground; these rails are NOT galvanically isolated).

### Step 0.3 — Verify the 3.3V ↔ 5V Bidirectional Level Shifter
1. Wire the level shifter (with Pi and Mega disconnected or unpowered):
   - `LV` ← CH9102 / CP2102 **`3V3`** pin
   - `HV` ← Branch C **`5.0 V`** rail
   - `GND` ← common star ground (both sides)
   - CH9102 / CP2102 **`+5V` pin → LEAVE DISCONNECTED!**
2. Measure `LV` (`3.3 V ± 0.1 V`) and `HV` (`5.0 V ± 0.2 V`).
3. Verify UART crossover wiring:
   - USB-TTL `TXD` → Shifter `LV1` → Shifter `HV1` → Mega **`Pin 19 (RX1)`**
   - USB-TTL `RXD` ← Shifter `LV2` ← Shifter `HV2` ← Mega **`Pin 18 (TX1)`**

### Step 0.4 — Tune TMC2208 Stepper Driver `V_ref`
1. Visually inspect the small sense resistors marked `R110`, `R100`, or `R150` on your TMC2208 carrier board to confirm `R_sense` (most StepStick TMC2208 boards use `R110 = 0.11 Ω`).
2. Confirm `MS1 = GND` and `MS2 = GND` for **1/8 microstepping** (40 steps/mm nominal on a 20-tooth GT2 pulley).
3. Use the official ADI/Trinamic Rev 1.13 equation:
   ```text
   I_RMS = (0.325 V / (R_sense + 0.03 Ω)) * (1 / √2) * (V_ref / 2.5 V)
   V_ref = I_RMS * 2.5 * √2 * (R_sense + 0.03) / 0.325
   ```
   **Quick Reference for `R_sense = 0.11 Ω` (most TMC2208 StepSticks):**

   | Target Motor Current `I_RMS` | Set `V_ref` (GND to Potentiometer) | Typical Use |
   | :---: | :---: | :--- |
   | `0.50 A` | `0.76 V` | Cool bench bring-up (recommended start!) |
   | `0.70 A` | `1.07 V` | Normal NEMA 17 GT2 gantry operation |
   | `0.85 A` | `1.30 V` | High torque (attach heatsink + check temp!) |

4. With 5V logic power ON (motors disconnected), measure DC voltage from `GND` to the driver's tiny brass potentiometer and adjust slowly with a ceramic/insulated screwdriver.

---

## Stage 1 — Arduino Mega: Limit Switches & E-Stop (Zero Motion)

**Goal:** Confirm all 3 Normally-Closed (NC) limit switches and E-Stop Pole 2 are wired cleanly before moving anything.

### Steps
1. Keep **12V SMPS OFF**. Connect the Arduino Mega to your PC via USB.
2. Open and upload:
   - [`firmware/bringup/mega_01_switches_estop/mega_01_switches_estop.ino`](bringup/mega_01_switches_estop/mega_01_switches_estop.ino)
3. Open **Arduino Serial Monitor** at **`115200 baud`**.
4. Verify at rest (nothing pressed, E-Stop released):
   - `X_HOME (Pin 22)` = `LOW -> CLOSED [NORMAL / SAFE]`
   - `Y_HOME (Pin 23)` = `LOW -> CLOSED [NORMAL / SAFE]`
   - `Z_LIMIT (Pin 24)` = `LOW -> CLOSED [NORMAL / SAFE]`
   - `E_STOP (Pin 25)` = `LOW -> CLOSED [NORMAL / SAFE]`
5. Click each microswitch with your finger one by one, press the E-Stop mushroom, and unplug one switch lead temporarily:
   - Each action must immediately print `[CHANGE] ... HIGH -> OPEN [TRIGGERED / WIRE BROKEN]`.

**Pass Gate:** All 4 inputs read `LOW` at rest and reliably flip `HIGH` when triggered or unplugged.

---

## Stage 2 — Arduino Mega: Servo & Gripper Calibration (Steppers Off)

**Goal:** Find the exact 12-bit PCA9685 PWM tick values for your Z-axis and parallel X-axis gripper without binding or buzzing the servos.

### Steps
1. Verify **PCA9685** wiring: `SDA = Pin 20`, `SCL = Pin 21`, `OE = Pin 9`, `VCC = 5V logic`, `V+ = 4.8–5.0V Servo Buck`, `CH0 = Z-Axis`, `CH1 = Gripper`.
2. **Tip for very first run:** Detach the plastic servo horns first, or manually position Z and gripper near mid-travel so nothing crashes on power-up.
3. Upload [`firmware/bringup/mega_02_servo_calibration/mega_02_servo_calibration.ino`](bringup/mega_02_servo_calibration/mega_02_servo_calibration.ino) (with 12V OFF during USB upload!).
4. Power on the 5V servo rail (E-Stop released), open Serial Monitor @ **`115200 baud`** with **Newline** line ending.
5. Type `on` to enable PCA9685 outputs (`PIN_SERVO_OE = LOW`).
6. Dial in all 5 positions gently using `z+`, `z-`, `g+`, `g-` (or `z <num>`, `g <num>`):
   - Move Z all the way UP clear of all caps (no buzzing!) → type **`mark raised`**
   - Place a bottle cap beneath the gripper, lower Z until jaw tips surround the cap ~2 mm above the plywood → type **`mark pick`**
   - Move Z to bin-drop height (~25 mm down) → type **`mark bin`**
   - Open gripper jaws to ~36 mm inner gap (`±18 mm` around cap) → type **`mark open`**
   - Close gripper jaws firmly on a 30 mm bottle cap without stalling/buzzing → type **`mark close`**
7. Type **`print`** and copy the 5 `#define` lines into your calibration sheet below!
8. Press the physical **E-Stop** once and confirm Serial Monitor prints `[E-STOP] PCA9685 outputs DISABLED.`

### Stage 2 Calibration Worksheet (Fill In Your Numbers)
```cpp
#define PWM_Z_RAISED      _____   // Default: 180
#define PWM_Z_LOWER_PICK  _____   // Default: 410
#define PWM_Z_LOWER_BIN   _____   // Default: 320
#define PWM_GRIP_OPEN     _____   // Default: 210
#define PWM_GRIP_CLOSE    _____   // Default: 385
```

---

## Stage 3 — Arduino Mega: Stepper Polarity, Homing, Scale, `TOOL_OFFSET` & Bins

**Goal:** Verify stepper directions, test homing safely, calibrate exact `STEPS_PER_MM`, measure `TOOL_OFFSET_X/Y_MM`, and measure your 4 color bin positions.

### Steps
1. Open [`firmware/bringup/mega_03_stepper_homing_test/mega_03_stepper_homing_test.ino`](bringup/mega_03_stepper_homing_test/mega_03_stepper_homing_test.ino), paste your 5 Stage 2 `PWM_*` `#define`s at the top, and upload to Mega.
2. **By hand (with 12V OFF)**, push both X and Y carriages to the **middle** of the frame so they have plenty of travel in both directions.
3. Turn on 12V SMPS (keep hand on E-Stop!), open Serial Monitor @ `115200 baud` (Newline):
   - **Step 3A — Direction Check:**
     - Type `en` (raises Z, opens gripper, energizes steppers).
     - Type `x -10` → X carriage **MUST** move **toward** the X-home switch!
     - Type `y -10` → Y bridge **MUST** move **toward** the Y-home switch!
     - *If an axis moved the wrong way:* turn OFF 12V, unplug that motor connector, reverse one coil pair (or flip the 4-pin motor plug 180°), and re-test `x -10` / `y -10`.
   - **Step 3B — First Auto-Home:**
     - Type `home`. Both axes seek their NC home switches at 20 mm/s, back off `3.0 mm`, confirm both switches re-close, and zero at machine `(0.00, 0.00)`.
   - **Step 3C — Measure True `STEPS_PER_MM`:**
     - Mark carriage position with tape, type `go 100 0`, measure actual millimeter travel `D_x` with a steel ruler.
     - Repeat `go 0 100`, measure actual travel `D_y`.
     - Compute:
       ```text
       STEPS_PER_MM_X = 40.0 * (100.0 / D_x)
       STEPS_PER_MM_Y = 40.0 * (100.0 / D_y)
       ```
   - **Step 3D — Measure `TOOL_OFFSET_X_MM` & `TOOL_OFFSET_Y_MM`:**
     - Type `home`, then use `x +5`, `y +5`, `zpick` (carefully!) until the center of the gripper jaws sits **dead-center over your plywood board's Top-Left `(0, 0)` corner fiducial**.
     - Type `pos` → record those exact machine `(X, Y)` values as **`TOOL_OFFSET_X_MM`** and **`TOOL_OFFSET_Y_MM`**!
   - **Step 3E — Measure Bin Coordinates `BIN_X_MM` & `BIN_Y_MM`:**
     - Type `zup`, then jog the gripper over the center of each physical color bin (`Red = Bin 1`, `Yellow = Bin 2`, `Green = Bin 3`, `Blue = Bin 4`), verify clearance with `zbin`, and type `pos` at each bin!

### Stage 3 Calibration Worksheet (Fill In Your Numbers)
```cpp
#define STEPS_PER_MM_X     _____f  // Nominal: 40.0f
#define STEPS_PER_MM_Y     _____f  // Nominal: 40.0f
#define TOOL_OFFSET_X_MM   _____f  // Machine X when gripper is over board TL (0,0)
#define TOOL_OFFSET_Y_MM   _____f  // Machine Y when gripper is over board TL (0,0)

const float BIN_X_MM[5] = { 0.0f, ____f, ____f, ____f, ____f }; // Bins 1..4 (machine mm)
const float BIN_Y_MM[5] = { 0.0f, ____f, ____f, ____f, ____f }; // Bins 1..4 (machine mm)
```

---

## Stage 4 — Raspberry Pi Zero 2 W: Setup, Camera & Bounding-Box Cap Preview

**Goal:** Prepare Raspberry Pi OS Lite (64-bit Bookworm), verify camera focus & top-center framing, and draw live colored **squares around detected caps** to confirm lighting and HSV thresholds — with **zero motor movement**.

### Step 4.1 — Pi Zero 2 W OS, Packages, LCD Overlay & Persistent USB-TTL Rule
Run these commands over SSH on your Raspberry Pi Zero 2 W (**keep Raspberry Pi OS Lite — do NOT install Desktop/Chromium on 512 MB RAM**):

```bash
# 1. Install required Python libraries
sudo apt update
sudo apt install -y python3-opencv python3-numpy python3-serial python3-picamera2

# 2. Allow your user to open serial ports without sudo
sudo usermod -aG dialout,video,input $USER

# 3. Install the persistent udev rule so CH9102 / CP2102 always binds to /dev/capsorter_mega
sudo cp ~/CapSorter/firmware/v2_pizero2w/99-capsorter-mega.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# 4. Enable SPI & the 3.5" SPI Touch LCD V3.0 DRM overlay in /boot/firmware/config.txt:
#    Add these lines (and comment out dtoverlay=vc4-kms-v3d if LCD is the sole screen):
#      dtparam=spi=on
#      dtoverlay=piscreen,drm,speed=18000000,rotate=90
sudo reboot
```

After reboot, verify power health (must report **`throttled=0x0`** with LCD + camera + USB-TTL plugged in):
```bash
vcgencmd get_throttled
ls -l /dev/capsorter_mega /dev/fb0
```

### Step 4.2 — Run Live Cap Bounding-Square Preview (`pi_01_camera_detect_preview.py`)
Place 1 Red, 1 Yellow, 1 Green, and 1 Blue cap on your open plywood board (gantry parked at home `(0, 0)` out of view) and run **one** of these:

```bash
cd ~/CapSorter/firmware/bringup

# Option A (Recommended): Stream LIVE video with bounding squares to your phone/laptop browser!
python3 pi_01_camera_detect_preview.py --camera-type picamera2_imx219 --web-port 8080
# -> Open http://<your-pi-ip>:8080 in Chrome/Safari!

# Option B: Mirror live bounding squares straight onto the 3.5" SPI LCD
python3 pi_01_camera_detect_preview.py --camera-type picamera2_imx219 --lcd /dev/fb0

# Option C: Save one raw PNG + one boxed PNG and print a terminal table (pure SSH)
python3 pi_01_camera_detect_preview.py --camera-type picamera2_imx219 --once --out-dir ./stage4_out
```
*(If using a USB webcam instead of the 8MP IMX219 CSI camera, replace `--camera-type picamera2_imx219` with `--camera-type usb_v4l2`.)*

### What to Check in Stage 4
1. **Framing:** All 4 corners of your 300×300 mm plywood plate are clearly visible inside the image with margin to spare.
2. **Focus & Diffused Lighting:** Caps look sharp with minimal white specular glare rings.
3. **Boxes:** Every Red, Yellow, Green, and Blue cap gets a clean colored square around it with the correct label (`RED`, `YELLOW`, `GREEN`, `BLUE`) and zero false boxes on the bare plywood background.
4. **If one color is missed:** Place that cap directly under the yellow **center crosshair** and read `Center HSV: (H, S, V)` in the top black banner, then widen that color's range in `HSV_RANGES` inside `firmware/v2_pizero2w/pi_zero_vision.py`.

**Pass Gate:** All 4 cap colors draw stable bounding squares with zero false detections on empty plywood.

---

## Stage 5 — Checkerboard Calibration & 4-Corner Coordinate Marking

**Goal:** Remove wide-angle lens distortion (`K, D`), mark your 4 plywood board corners `(0,0) → (300,300)` to compute homography `H`, measure overhead camera height for cap-height parallax correction, and save a validated `.npz` calibration file.

### Step 5A — Capture 12–15 Checkerboard Photos
1. Print a standard **10×7 square** black-and-white checkerboard (**9×6 inner corners**, `25 mm` squares) and tape it flat onto stiff cardboard.
2. Run (keep `--web-port 8080` open on your phone/laptop so you can see green detected corners live!):
   ```bash
   python3 pi_02_capture_and_mark_corners.py --mode checkerboard \
       --camera-type picamera2_imx219 --pattern 9x6 --count 12 \
       --auto-interval 2.5 --out-dir ./calib_imgs --web-port 8080
   ```
3. Slowly move and slightly tilt the checkerboard across the center, all 4 corners, and all 4 edges until 12 images (`cb_00.png` .. `cb_11.png`) are saved.

### Step 5B — Mark the 4 Board Corners & Generate Validated `.npz`
1. Clear everything off the 300×300 mm plywood board (leave 4 high-contrast corner dots/L-marks at `(0,0)`, `(300,0)`, `(300,300)`, `(0,300)`).
2. Measure with a tape measure the vertical height **`H_cam` (in mm)** from the top of the plywood plate up to the camera lens (e.g., `350 mm`) and your typical bottle cap height **`h_cap`** (typically `13 mm`).
3. First pass — undistort a photo of the empty board and overlay a **20 px numbered pixel ruler grid**:
   ```bash
   python3 pi_02_capture_and_mark_corners.py --mode corners \
       --camera-type picamera2_imx219 --lens-model fisheye \
       --checkerboard-glob "./calib_imgs/*.png" --out-dir ./stage5_out
   ```
   *(If you have a monitor/VNC attached, add `--show` to simply click the 4 corners with your mouse!)*
4. Copy or view `stage5_out/stage5_rectified_ruler_grid.png` and read the `(u, v)` pixel coordinates of your 4 board corners in exact clockwise order:
   - **1. Top-Left** `(0, 0)` → `uTL,vTL`
   - **2. Top-Right** `(300, 0)` → `uTR,vTR`
   - **3. Bottom-Right** `(300, 300)` → `uBR,vBR`
   - **4. Bottom-Left** `(0, 300)` → `uBL,vBL`
5. Second pass — compute homography `H`, draw the verification grid, and save your final validated `.npz`:
   ```bash
   mkdir -p /home/pi/capsorter
   python3 pi_02_capture_and_mark_corners.py --mode corners \
       --camera-type picamera2_imx219 --lens-model fisheye \
       --checkerboard-glob "./calib_imgs/*.png" --out-dir ./stage5_out \
       --corners "uTL,vTL;uTR,vTR;uBR,vBR;uBL,vBL" \
       --camera-height-mm 350 --cap-height-mm 13 \
       --out /home/pi/capsorter/calibration_csi_imx219.npz
   ```
   *(For a USB webcam, use `--camera-type usb_v4l2 --lens-model pinhole --out /home/pi/capsorter/calibration_usb.npz`.)*

**Pass Gate:**
- Console reports `RMS reprojection error < 1.0 px` and `[OK] Saved validated calibration ...`.
- Inspect `stage5_out/stage5_corners_verified.png`: the **bright green 300×300 mm square** must sit flush on your 4 physical board corners and the cyan `18..282 mm` inner box must look straight and square.

---

## Stage 6 — Verify Millimeter Coordinates, Touching-Cap Splitting & Clearance

**Goal:** Prove the calibrated Pi vision pipeline outputs accurate real-world `(X, Y)` millimeter coordinates before connecting to the Arduino Mega.

### Steps
Run `pi_03_verify_mm_coordinates.py` (uses the exact production `PiZeroVisionController` math with **zero motor commands**):
```bash
python3 pi_03_verify_mm_coordinates.py \
    --calib /home/pi/capsorter/calibration_csi_imx219.npz \
    --camera-type picamera2_imx219 --web-port 8080
```
*(Or use `--once` for a single saved PNG + terminal table.)*

### 4 Quick Physical Checks on the Board
1. **Ruler Accuracy Check:** Place caps at measured ruler coordinates `(50, 50)`, `(150, 150)`, `(250, 50)`, and `(250, 250)`. Confirm the printed `(BOARD X, BOARD Y)` values match the ruler within **`±2.5 mm`**.
   - *If corners consistently read slightly pushed outward from center:* verify your `--camera-height-mm` measurement in Stage 5B.
2. **Touching Same-Color Split Check:** Push two **Red** caps until their rims touch. Confirm the screen draws **two separate squares** with two distinct `(X, Y)` coordinates ~30 mm apart!
3. **X-Jaw Clearance-First Peeling Check:** Line up 3 caps side-by-side along the X-axis (touching). Confirm the outer left/right caps are tagged `BLK:1` and chosen as `#1` before the sandwiched middle cap (`BLK:2`) — and notice the cyan open-jaw footprint rectangles (`±18..26 mm`) drawn around pick `#1`.
4. **Safe Boundary Check:** Place one cap very close to the board edge (`X ≈ 10 mm`). Confirm its box turns **red** and is marked `OUT-OF-SAFE` (`SKIP`).

**Pass Gate:** All 4 checks pass with `< ±2.5 mm` ruler accuracy.

---

## Stage 7 — Final Mega Upload, UART Handshake & 3.5" SPI Touch LCD Test

**Goal:** Upload final motion firmware to the Mega, verify the Pi ↔ Level Shifter ↔ Mega binary protocol, test one controlled single-cap pick, test the UART-cut finish-and-park policy, and verify the 3.5" SPI LCD touchscreen.

### Step 7A — Update & Upload Final `mega_motion.ino`
1. Open [`firmware/mega_motion/mega_motion.ino`](mega_motion/mega_motion.ino) and enter your measured constants from Stages 2 & 3:
   - `PWM_Z_RAISED`, `PWM_Z_LOWER_PICK`, `PWM_Z_LOWER_BIN`, `PWM_GRIP_OPEN`, `PWM_GRIP_CLOSE`
   - `STEPS_PER_MM_X`, `STEPS_PER_MM_Y`
   - `TOOL_OFFSET_X_MM`, `TOOL_OFFSET_Y_MM`
   - `BIN_X_MM[5]`, `BIN_Y_MM[5]`
2. Confirm [`firmware/common/packet_protocol.h`](common/packet_protocol.h) is alongside `mega_motion/` (if opening inside Arduino IDE, copy `packet_protocol.h` into the `mega_motion/` sketch folder and change `#include "../common/packet_protocol.h"` to `#include "packet_protocol.h"`).
3. **Turn 12V SMPS OFF**, connect Mega USB, compile & upload with pinned baseline (`arduino:avr` `1.8.6`, `AccelStepper` `1.64`, `Adafruit PWM Servo Driver` `3.0.2`).
4. Unplug USB, turn 12V SMPS ON (hand on E-Stop!) — Mega automatically homes to machine `(0, 0)` and waits silently in `STATE_IDLE`.

### Step 7B — Verify Pi ↔ Mega Binary UART Link (`pi_04_uart_link_test.py`)
Run these 4 tests in order on the Pi Zero 2 W:

```bash
cd ~/CapSorter/firmware/bringup

# 1. Ping Mega readiness (must reply CMD_STATUS_READY 0xB1)
python3 pi_04_uart_link_test.py --mode ping

# 2. Place 1 RED cap at ruler spot (150, 150) mm and run ONE closed-loop pick-and-place!
python3 pi_04_uart_link_test.py --mode single-pick --x 150 --y 150 --color 1

# 3. Place 1 RED cap at (150, 150) mm and test UART CUT safety policy:
#    Pi stops heartbeats after 1.0 s -> watch Mega finish depositing ONLY this cap
#    into Bin 1, then park cleanly at (0, 0), disable steppers, and latch FAILSAFE_HALT!
python3 pi_04_uart_link_test.py --mode uart-cut-test --x 150 --y 150 --color 1

# 4. Verify explicit operator re-arm & re-home recovers cleanly to READY:
python3 pi_04_uart_link_test.py --mode rearm
```

### Step 7C — Verify 3.5" SPI Touch LCD & Touch Orientation (`pi_05_lcd_touch_test.py`)
```bash
python3 pi_05_lcd_touch_test.py --fb-device /dev/fb0
```
1. Confirm the 480×320 screen shows crisp `RED`, `YLW`, `GRN`, `BLU` bars and 4 green corner crosshairs (`TL`, `TR`, `BR`, `BL`).
2. Tap each corner and the 3 sidebar buttons (`PAUSE`, `REARM`, `VIEW`) and check the terminal output:
   - If taps land cleanly inside each button, no extra flags are needed!
   - If X/Y are swapped or mirrored on your specific 3.5" V3.0 panel, re-run with `--touch-swap-xy`, `--touch-invert-x`, and/or `--touch-invert-y` until tapping `PAUSE`, `REARM`, and `VIEW` prints the matching button name every time. Record those flags for Stage 8!

---

## Stage 8 — Launch Final Autonomous Build & 5-Point Physical Acceptance Tests

### Launch Command (V2 Raspberry Pi Zero 2 W)
```bash
python3 ~/CapSorter/firmware/v2_pizero2w/pi_zero_vision.py \
    --calib /home/pi/capsorter/calibration_csi_imx219.npz \
    --camera-type picamera2_imx219 \
    --serial-port auto \
    --enable-lcd --fb-device /dev/fb0
```
*(Append `--touch-swap-xy` / `--touch-invert-x` / `--touch-invert-y` if needed from Stage 7C.)*

### 5-Point Physical Acceptance Checklist
Run all 5 live tests before treating the machine as bench-validated:

- [ ] **Test 1 — Off-Grid Scattered Caps:** Place 4 caps (R, Y, G, B) at arbitrary non-grid positions across the board. Confirm all 4 sort cleanly into Bins 1–4 and the gantry parks at `(0, 0)`.
- [ ] **Test 2 — Touching Same-Color Pair & 3-Cap Row:** Place 2 touching Red caps and a 3-cap horizontal cluster. Confirm the outer caps are peeled first and every cap sorts cleanly.
- [ ] **Test 3 — Mid-Cycle Bump Re-Acquisition:** Place 2 caps. While the gantry is lifting Cap #1 and carrying it to the bin, slide Cap #2 by `~30 mm` with a wooden stick. Confirm the bin-side re-scan captures the **new** coordinates and picks Cap #2 accurately on the very next cycle!
- [ ] **Test 4 — Bridge Occlusion Reveal & Stuck-Cap Quarantine:**
  - Place a cap inside Bin 1's `Y = 55 ± 35 mm` band while picking a Red cap first → confirm Mega executes a clear-FOV park (`CMD_PARK_FOR_SCAN`), reveals the hidden cap, and sorts it!
  - Tape one cap lightly to the board → confirm Pi retries 3 times, logs a quarantine warning, skips the stuck cap, sorts any remaining caps, and parks cleanly (zero infinite loop!).
- [ ] **Test 5 — Safety & Fault Recovery:**
  - Tap **`PAUSE SORT`** on the 3.5" LCD mid-session → confirm Mega finishes the current cap, parks at `(0, 0)` without faulting, and resumes cleanly when you tap **`RESUME`**.
  - Unplug the USB-TTL cable mid-cap → confirm Mega finishes depositing ONLY that active cap, parks at `(0, 0)`, disables steppers, and latches `FAILSAFE_HALT`; plug cable back in and tap **`RE-ARM & HOME`** on the LCD to verify clean recovery.
  - Press the physical **E-Stop mushroom** mid-move → confirm 12V cuts immediately, Mega latches `HARD_FAULT`, releasing E-Stop alone causes **zero** motion, and tapping **`RE-ARM & HOME`** re-homes cleanly.

---

## Appendix — V1 (ESP32-CAM + Teensy 4.1) Staged Bring-Up Notes

If building the **V1** architecture instead of V2:
1. Complete **Stages 0, 1, 2, 3, and 7A** on the Arduino Mega identically.
2. Flash [`firmware/v1_esp32cam/esp32cam_uart.ino`](v1_esp32cam/esp32cam_uart.ino) (`arduino-esp32 2.0.17`, Board: **AI Thinker ESP32-CAM**, **PSRAM: Enabled**). Verify `[ESP32-CAM READY]` on USB Serial @ 115,200 baud.
3. Wire ESP32-CAM UART2 (`GPIO13 TX2` → Teensy `Pin 7 RX2`, `GPIO14 RX2` ← Teensy `Pin 8 TX2`, shared GND, 3.3V only) and Teensy `Serial1` (`Pin 0 RX1`, `Pin 1 TX1`) through the **3.3V ↔ 5V Bidirectional Level Shifter** to Mega `Serial1` (`Pin 18 TX1`, `Pin 19 RX1`). **Teensy 4.1 pins are strictly 3.3V — NEVER connect 5V directly.**
4. Measure the 4 board corner pixel coordinates `(u, v)` on the 320×240 OV2640 frame, update `BOARD_FIDUCIALS_PX` and `CAMERA_HEIGHT_MM` in [`firmware/v1_teensy41/teensy_vision.cpp`](v1_teensy41/teensy_vision.cpp), verify grid accuracy, and only then change `#define V1_CALIBRATION_VERIFIED 0` to `1` and upload (`Teensyduino 1.59`).

---

## Quick Troubleshooting Reference

| Symptom | Likely Cause | Exact Fix |
| :--- | :--- | :--- |
| `Stage 1`: A switch reads `HIGH` at rest | Wired to NO instead of NC terminal, or loose wire | Move wire to the switch's `COM` + `NC` tabs; check GND continuity |
| `Stage 2`: Servo jitters or Mega resets | Servo `V+` sharing Logic Buck or weak power rail | Move PCA9685 `V+` strictly to separate Branch B 5V/5A buck; check shared GND |
| `Stage 3`: Axis moves away from home on `-10` | Stepper coil polarity inverted | Power OFF 12V and flip the 4-pin stepper plug 180° (or swap one coil pair) |
| `Stage 3`: Homing faults with `"wire broken"` | Switch stuck open or wired NO | Re-verify Stage 1; ensure all 3 NC switches read `LOW` when untouched |
| `Stage 4`: Caps not boxed / wrong color | Lighting hue shift or specular glare | Center cap on yellow crosshair in `pi_01`, read HSV, tune `HSV_RANGES`, diffuse LED |
| `Stage 5`: Calibration fails `"RMS > 1.0 px"` | Motion blur or flat/identical checkerboard angles | Re-shoot 12 sharp photos with varied tilts across center + all 4 corners |
| `Stage 6`: Corners off by 3–6 mm radially | Parallax height unset or inaccurate | Re-run Stage 5B with tape-measured `--camera-height-mm` and `--cap-height-mm 13` |
| `Stage 7`: `pi_04 --mode ping` times out | TX/RX not crossed, missing GND, or missing `LV`/`HV` | Check `TX→RX` crossover through shifter, `LV=3.3V`, `HV=5.0V`, shared GND, `mega_motion.ino` flashed |
| `Stage 7`: `/dev/capsorter_mega` missing | Udev rule not reloaded or user not in `dialout` | Run `sudo udevadm control --reload-rules && sudo udevadm trigger`; check `lsusb` (`1a86` or `10c4`) |
| `Stage 7`: LCD touch taps miss buttons | Panel XPT2046 axis swap/inversion for `rotate=90` | Test flags `--touch-swap-xy`, `--touch-invert-x`, `--touch-invert-y` in `pi_05_lcd_touch_test.py` |
| `Stage 8`: Pi reboots or `get_throttled != 0x0` | Voltage drop on Pi `PWR IN` cable | Use short ≥20 AWG wire from Branch C buck tuned to `5.10 V` |
