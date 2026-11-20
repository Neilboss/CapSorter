# Autonomous Bottle-Cap Sorting Machine
### Vision-Guided Pick & Place System (Offline • Dual-Controller Architecture)

**By Neil Shettigar, Kahan, and Shourya**  
*Main development & documentation by Neil Shettigar*

🌐 **Live Website (Vercel):** [https://capsorter.vercel.app](https://capsorter.vercel.app)  
🌐 **Mirror (GitHub Pages):** [https://neilboss.github.io/CapSorter/](https://neilboss.github.io/CapSorter/)  
📄 **Technical Report (12-Page PDF):** [Autonomous_Bottle_Cap_Sorting_Machine_Technical_Report.pdf](https://capsorter.vercel.app/Autonomous_Bottle_Cap_Sorting_Machine_Technical_Report.pdf)

---

![System Poster](assets/system_poster.jpg)

## 📌 Executive Overview

The **Autonomous Bottle-Cap Sorting Machine** is an embedded robotics project designed to detect, pick, and sort standard plastic bottle caps ($\varnothing 28\text{–}32\text{ mm}$) by colour across a $300 \times 300\text{ mm}$ grid workspace, depositing them into four designated color bins (Red, Yellow, Green, Blue).

The system operates **100% offline** without laptops, Wi-Fi, or cloud connectivity. The web portal (`index.html`) now presents **two complete hardware architecture versions**:

- **Version 1 (`v1.html`)**: **Teensy 4.1 + ESP32-CAM (OV2640) + Arduino Mega 2560** (Bare-metal C++ dual-MCU + camera streamer architecture).
- **Version 2 (`v2.html`)**: **Raspberry Pi Zero 2 W (16 GB microSD) + 8MP Sony IMX219 160° CSI Camera (or USB Webcam) + Arduino Mega 2560** (Consolidated offline Linux + Python OpenCV vision SBC paired with deterministic real-time Mega motion control).

---

## 🚀 Key Specifications (V1 vs. V2)

| Specification | Version 1 (`v1.html`) | Version 2 (`v2.html`) |
| :--- | :--- | :--- |
| **Machine Envelope** | $500 \times 500 \times 500\text{ mm}$ | $500 \times 500 \times 500\text{ mm}$ |
| **Sorting Workspace** | $300 \times 300\text{ mm}$ (10 × 10 cells, $30\text{ mm}$ pitch) | $300 \times 300\text{ mm}$ (10 × 10 cells, $30\text{ mm}$ pitch) |
| **Cap Outer Diameter** | $\varnothing 28\text{–}32\text{ mm}$ standard beverage bottle caps | $\varnothing 28\text{–}32\text{ mm}$ standard beverage bottle caps |
| **Colours Handled** | 4 primary colours (Red, Yellow, Green, Blue) | 4 primary colours (Red, Yellow, Green, Blue) |
| **Primary Vision Processor** | Teensy 4.1 ($600\text{ MHz}$ Cortex-M7, $1\text{ MB}$ RAM) | **Raspberry Pi Zero 2 W** ($4\times 1\text{ GHz}$ Cortex-A53, $512\text{ MB}$ RAM, $16\text{ GB}$ microSD) |
| **Camera Sensor** | ESP32-CAM (OV2640 • $320 \times 240$ RGB565) | **8MP Sony IMX219 160° Wide-Angle CSI** (or USB Webcam • $640 \times 480$ RGB888) |
| **Capture + Vision Latency** | $\sim 1.67\text{ s}$ UART frame stream + $\sim 40\text{ ms}$ C++ vision | **$\sim 20\text{ ms}$ MIPI CSI-2 DMA + $\sim 25\text{ ms}$ Python OpenCV ($\sim 45\text{ ms}$ total)** |
| **Motion Controller** | Arduino Mega 2560 ($16\text{ MHz}$ AVR) | Arduino Mega 2560 ($16\text{ MHz}$ AVR) |
| **Actuation** | 2× NEMA 17 Steppers (TMC2208) + 2× MG90S Servos (PCA9685) | 2× NEMA 17 Steppers (TMC2208) + 2× MG90S Servos (PCA9685) |
| **Main Power Supply** | $12\text{V}, 10\text{A}$ SMPS ($120\text{W}$) feeding 3 separate regulated branches (common star ground) | $12\text{V}, 10\text{A}$ SMPS ($120\text{W}$) feeding 3 separate regulated branches (common star ground) |

---

## 🧠 System Architecture & Division of Responsibilities

```
+---------------+      Wired UART (921,600 Baud)      +-------------------+
|   ESP32-CAM   | ----------------------------------> |    Teensy 4.1     |
| (Image Capture|                                     | (Vision & Planning|
|  OV2640 QVGA) | <---------------------------------- |   Cortex-M7 600M) |
+---------------+          Capture Trigger            +-------------------+
                                                                |
                                             Level-Shifted UART | Plan Packets &
                                             (115,200 Baud)     | Heartbeat
                                                                v
                                                      +-------------------+
                                                      | Arduino Mega 2560 |
                                                      | (Motion & Safety) |
                                                      +-------------------+
                                                       /        |        \
                                     STEP/DIR Pulses  /         |         \ I2C PWM
                                                     v          v          v
                                                +---------+ +-------+ +---------+
                                                | TMC2208 | | Limit | | PCA9685 |
                                                | Stepper | | Switch| | 16-Ch   |
                                                | Drivers | |  (NC) | | 12-Bit  |
                                                +---------+ +-------+ +---------+
                                                     |                     |
                                                     v                     v
                                                2x NEMA 17             2x MG90S
                                                (X & Y Gantry)       (Z & Gripper)
```

### Division of Responsibilities
1. **ESP32-CAM**: Dedicated purely to top-down frame acquisition (QVGA $320 \times 240$ RGB565). No network overhead.
2. **Teensy 4.1**: Decodes raw frame, performs fixed-point HSV color segmentation, extracts connected-component centroids, applies $3 \times 3$ planar homography for perspective distortion correction, and computes the shortest-travel sorting plan.
3. **Arduino Mega 2560**: Manages deterministic step generation, microswitch homing, 8-step pick-and-place choreography, and fail-safe state recovery.
4. **PCA9685**: Dedicated 16-channel 12-bit I2C PWM generator driving the two MG90S metal-gear micro servos (Z-axis linkage & parallel gripper jaws).
5. **TMC2208 × 2**: SilentStepStick drivers running in StealthChop2 mode for silent, resonance-free XY gantry travel.

---

## ⚡ Power Distribution Tree

> **Note:** All three branches share a **common single-point star ground** (separate regulated branches for noise decoupling, **not** galvanic isolation). Verify exact part SKUs, buck converter efficiencies, wire gauges, branch fuse ratings, and servo voltage specifications before power-up.

- **Branch A (Motors - 12V Direct)**: Fused 12V line (switched via dual-pole hardware E-stop) feeding TMC2208 VMOT with $100\,\mu\text{F}$ low-ESR decoupling capacitors.
- **Branch B (Servos - 5V/5A Buck)**: Separate high-current buck converter (12V input switched via hardware E-stop) feeding PCA9685 V+ to absorb servo stall spikes without pulling down logic. Verify MG90S SKU voltage rating (original TowerPro MG90S is specified at $4.8\text{V}$ nominal) and PCA9685 V+ terminal ratings.
- **Branch C (Logic - 5V/3A Buck)**: Separate regulated 5V logic rail powering Arduino Mega `5V` pin (**not `VIN`**; avoid USB backfeed during flashing), Teensy 4.1 VIN (V1) or Raspberry Pi Zero 2 W 5V (V2), ESP32-CAM 5V (V1), and PCA9685 VCC.
- **Level Shifter**: 4-channel bidirectional logic shifter mandatory on **both TX and RX** between Arduino Mega 5V Serial1 and Teensy 4.1 / Pi Zero 2 W 3.3V UART (neither Teensy 4.1 nor Pi Zero 2 W has 5V-tolerant pins).

---

## 🛡️ Fail-Safe Comms Recovery & Emergency Stop Protocol

- **Hardwired Emergency Stop**: A dual-pole latching NC mushroom E-stop directly cuts 12V actuator power (Branches A & B) independently of firmware while an auxiliary NC contact on Mega Pin 25 latches `STATE_HARD_FAULT`, disables `PIN_STEPPER_EN` and `PIN_SERVO_OE`, and requires explicit operator re-arm (`CMD_REARM_HOME`) and re-homing before motion resumes.
- **UART Communication Loss (`>1.2 s` without `0xF0` heartbeat)**: Because the default gripper has no physical cap-presence sensor, the Mega **does not assume a cap was picked** and does not traverse blindly to a bin. Instead it immediately stops XY motion, retracts Z to safe overhead clearance while preserving jaw state, disables steppers, emits `CMD_RECOVERY_HALT`, and latches `STATE_FAILSAFE_HALT` until explicit operator re-arm and re-homing. (Verified in-flight bin completion is gated behind optional physical grip sensing.)

---

## 📄 Comprehensive Engineering Technical Report (PDF)

A publication-grade, 12-page technical dossier is included directly in this repository for formal academic or industrial engineering submissions:

👉 **[Download Autonomous_Bottle_Cap_Sorting_Machine_Technical_Report.pdf](Autonomous_Bottle_Cap_Sorting_Machine_Technical_Report.pdf)**

### Dossier Contents & Page Structure:
1. **Cover & Master System Poster**: Project identity, key specifications, and full-bleed CAD architecture render.
2. **System Overview & Architectural Partitioning**: Dual-controller design rationale, responsibility division table, and UART pipeline.
3. **Mechanical Engineering & Gantry Motion**: 4-axis mechanical analysis (X, Y, Z, Gripper) + workspace geometry and travel limits.
4. **Embedded Computer Vision Pipeline**: 4-stage vision pipeline (RGB565 → HSV → Centroid Extraction → Homography Coordinate Mapping).
5. **Electrical & Power Distribution Architecture**: 3-branch separate regulated power tree (12V Motors, 5V/5A Servos, 5V/3A Logic, common star ground) & bidirectional level-shifter safeguards.
6. **Hardware Pinout & Wiring Interface Matrix**: Complete pin-by-pin mapping for Teensy 4.1, Arduino Mega 2560, TMC2208, and PCA9685.
7. **Motion Control & Pick-and-Place Choreography**: 8-step kinematic pick-and-place sequence, fail-safe communication recovery state machine.
8. **Comprehensive Bill of Materials (BOM)**: Categorized itemized list with technical specifications, quantities, and SKU verification notes.
9. **Implementation Roadmap & Milestones**: 6-stage structured milestone plan with physical prototype build photos.
10. **Critical Engineering Considerations & FMEA**: Power rail decoupling, thermal dissipation, homing accuracy, servo chatter mitigation, and optical calibration.
11. **Firmware Vault — Part 1**: Pre-validation C/C++ firmware headers (`packet_protocol.h`) and ESP32-CAM frame streaming driver (`esp32cam_uart.ino`).
12. **Firmware Vault — Part 2 & Pre-Build Review**: Teensy 4.1 computer vision pipeline (`teensy_vision.cpp`), Arduino Mega motion sequencer (`mega_motion.ino`), and bench validation checklist.

---

## 💻 Web Presentation & Multi-Version Interactive Simulator

The repository includes a standalone, zero-dependency engineering web application with a **Version Selector Portal**:
- **`index.html` (Version Portal)**: Landing gateway with **V1** and **V2** buttons and a side-by-side architectural comparison matrix.
- **`v1.html` (Version 1 Specification)**: Complete Teensy 4.1 + ESP32-CAM + Arduino Mega 2560 blueprint, simulator, pinout matrix, BOM, and C++ firmware vault.
- **`v2.html` (Version 2 Specification)**: Complete Raspberry Pi Zero 2 W (16 GB microSD) + 8MP IMX219 160° Camera + Arduino Mega 2560 blueprint, 5-stage OpenCV fisheye pipeline, simulator, GPIO pinout matrix, V2 BOM, and Python OpenCV + C++ firmware vault (`pi_zero_vision.py`, `fisheye_calibrate.py`, `mega_motion.ino`, `packet_protocol.h`).
- **`pdf_dossier.html` & `pdf_dossier_v2.html`**: 12-page printable A4 technical report dossiers for V1 and V2.

### How to Run Locally
Simply open `index.html` in any modern web browser:
```bash
# On Windows
start index.html

# Or via Python local server
python -m http.server 8000
```

---

## 👥 Credits
- **Designed & Engineered by:** Neil Shettigar, Kahan, and Shourya
- **Lead Embedded Architecture & Software:** Neil Shettigar
