# CapSorter Firmware Vault

Production-ready, hardened firmware source code for the Autonomous Bottle-Cap Sorting Machine (V1 and V2 architectures).

## Directory Structure

```
firmware/
├── README.md                 # Firmware documentation, flashing guide & safety specifications
├── common/
│   └── packet_protocol.h     # Shared binary packet structures, opcodes, CRC8 validation & baud constants
├── mega_motion/
│   └── mega_motion.ino       # Arduino Mega 2560 motion sequencer, Step/Dir generator, NC homing, PCA9685 servo driver
├── v1_esp32cam/
│   └── esp32cam_uart.ino     # ESP32-CAM (AI-Thinker) dedicated top-down QVGA RGB565 frame streaming driver
├── v1_teensy41/
│   └── teensy_vision.cpp     # Teensy 4.1 ARM Cortex-M7 embedded fixed-point HSV vision, BFS centroid extraction & travel planner
└── v2_pizero2w/
    ├── pi_zero_vision.py     # Raspberry Pi Zero 2 W Python OpenCV vision pipeline, fisheye undistortion, NMS & plan dispatcher
    ├── fisheye_calibrate.py  # Checkerboard intrinsic camera matrix & distortion coefficient calibration utility
    └── calibration.npz       # Pre-computed intrinsic calibration matrix and distortion coefficients
```

## Target Hardware & Pin Mapping Summary

| Subsystem | Target Board | Processor / OS | Interface / Baud Rate | Logic Level |
| :--- | :--- | :--- | :--- | :--- |
| **V1 Camera Streamer** | ESP32-CAM (AI-Thinker) | ESP32-D0WDQ6 (240 MHz) | UART2 @ 921,600 Baud | 3.3V Native |
| **V1 Vision & Planner** | Teensy 4.1 | NXP i.MX RT1062 Cortex-M7 (600 MHz) | Serial2 @ 921,600, Serial1 @ 115,200 | 3.3V (5V Tolerant on select pins, Shifter used for Mega) |
| **V2 Vision & Planner** | Raspberry Pi Zero 2 W | BCM2710A1 4× Cortex-A53 (1.0 GHz, Linux) | `/dev/serial0` (UART0) @ 115,200 Baud | **3.3V ONLY** (Must use 4-ch bidirectional shifter to Mega) |
| **Motion & Fail-Safe** | Arduino Mega 2560 Rev3 | Microchip ATmega2560 (16 MHz AVR) | Serial1 @ 115,200 Baud, Hardware I2C (Pins 20/21) | 5.0V Native |
| **Servo PWM Controller**| PCA9685 16-Channel | NXP PCA9685 I2C (Address 0x40) | I2C (Mega Pins 20 SDA, 21 SCL) | VCC 5.0V, V+ 5.0V/5A High Current Rail |
| **Stepper Drivers** | 2× TMC2208 SilentStepStick| Trinamic TMC2208 in StealthChop2 | STEP / DIR / EN via Mega Digital Pins | Logic 5.0V, VMOT 12.0V Fused |

## Critical Safety & Hardware Limits

1. **AVR Step Rate Ceiling**: Step generation is paced at $70\text{ mm/s}$ ($\le 2,800\text{ steps/s}$ at $1/8$ microstepping), well within the single-ISR ATmega2560 timer limit ($\sim 4,000\text{ steps/s}$).
2. **UART FIFO Pacing**: Packets between Vision SBC/MCU and Mega are transmitted in 32–64 byte chunks with $5\text{ ms}$ inter-chunk delay to prevent overrun of the 64-byte hardware UART ring buffer.
3. **Logic Level Isolation**: Raspberry Pi Zero 2 W and Teensy 4.1 pins operate at 3.3V. **Never connect 5V Mega signals directly to 3.3V GPIOs.** Always route through the 4-channel bidirectional logic level shifter.
4. **Servo Inrush Isolation**: PCA9685 $V^+$ is powered by an independent $5\text{V}/5\text{A}$ buck converter with a $1000\,\mu\text{F}$ capacitor to absorb servo stall current spikes without causing MCU brownout. Pin 9 is used as Output Enable (`OE`) to suppress startup twitching.
