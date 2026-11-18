// =========================================================================
// Arduino Mega 2560 - Deterministic Motion & Fail-Safe Sequencer (V1 & V2)
// Hardware: ATmega2560 @ 16 MHz | 2x TMC2208 Steppers | PCA9685 Servos
// Wiring:
//   Serial1 (Pin 19 RX1, Pin 18 TX1) <-> 5V/3.3V Shifter <-> Teensy or Pi Zero 2W
//   X Stepper (TMC2208): STEP = Pin 2, DIR = Pin 5, EN = Pin 8 (Active LOW)
//   Y Stepper (TMC2208): STEP = Pin 3, DIR = Pin 6, EN = Pin 8 (Active LOW)
//   Limit Switches (NC to GND, INPUT_PULLUP): X = Pin 22, Y = Pin 23, Z = Pin 24
//   PCA9685 Servo Driver (I2C 0x40): SDA = Pin 20, SCL = Pin 21
// =========================================================================
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>
#include "../common/packet_protocol.h"

// Pin Definitions
#define PIN_X_STEP         2
#define PIN_X_DIR          5
#define PIN_Y_STEP         3
#define PIN_Y_DIR          6
#define PIN_STEPPER_EN     8   // Active LOW enable shared by both TMC2208 drivers
#define PIN_X_HOME        22   // NC switch to GND: LOW = normal, HIGH = triggered/open
#define PIN_Y_HOME        23   // NC switch to GND: LOW = normal, HIGH = triggered/open
#define PIN_Z_HOME        24   // NC switch to GND: LOW = normal, HIGH = triggered/open

// Kinematics: 200 steps/rev * 16 microsteps / (20T * 2mm GT2 belt) = 80 steps/mm
#define STEPS_PER_MM      80.0f
#define MAX_TRAVEL_X_MM  370.0f // 300mm sorting board + 55-65mm external bin zone
#define MAX_TRAVEL_Y_MM  300.0f

// PCA9685 50Hz (12-bit, 4096 ticks / 20ms) Calibrated MG90S Servo Pulse Counts
#define SERVO_Z_CH         0
#define SERVO_GRIPPER_CH   1
#define PWM_Z_RAISED     180   // 0 mm (retracted overhead clearance)
#define PWM_Z_LOWER_PICK 410   // ~40 mm (lowered onto cap on board)
#define PWM_Z_LOWER_BIN  320   // ~25 mm (lowered into bin opening)
#define PWM_GRIP_OPEN    210   // ~36 mm jaw gap (open)
#define PWM_GRIP_CLOSE   385   // ~26 mm jaw gap (clamped on 28-32mm cap)

// Watchdog timeout (if no heartbeat byte 0xF0 from Teensy/Pi Zero 2W for >1.2s)
#define COMMS_TIMEOUT_MS 1200UL

AccelStepper stepperX(AccelStepper::DRIVER, PIN_X_STEP, PIN_X_DIR);
AccelStepper stepperY(AccelStepper::DRIVER, PIN_Y_STEP, PIN_Y_DIR);
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// External Sorting Bin Coordinates (mm) for Bins 1..4 (Red, Yellow, Green, Blue)
const float BIN_X_MM[5] = { 0.0f, 355.0f, 355.0f, 355.0f, 355.0f };
const float BIN_Y_MM[5] = { 0.0f,  55.0f, 130.0f, 205.0f, 280.0f };

enum RobotState : uint8_t {
  STATE_IDLE,
  STATE_HOMING,
  STATE_MOVING_TO_CAP,
  STATE_LOWER_Z,
  STATE_GRIP_CAP,
  STATE_LIFT_Z,
  STATE_TRAVERSE_BIN,
  STATE_LOWER_BIN,
  STATE_RELEASE_CAP,
  STATE_RETRACT_Z,
  STATE_PARKING_HOME,
  STATE_FAILSAFE_HALT
};

RobotState    currentState      = STATE_HOMING;
CapTarget     batchQueue[100];
CapTarget     currentCap;
uint8_t       batchTotal        = 0;
uint8_t       currentIdx        = 0;
bool          hasInFlightCap    = false;
bool          commsFaultLatched = false;
unsigned long stateTimerMs      = 0;
unsigned long lastHeartbeatMs   = 0;

void moveGantryToMm(float targetX_mm, float targetY_mm) {
  float clampedX = constrain(targetX_mm, 0.0f, MAX_TRAVEL_X_MM);
  float clampedY = constrain(targetY_mm, 0.0f, MAX_TRAVEL_Y_MM);
  stepperX.moveTo(lroundf(clampedX * STEPS_PER_MM));
  stepperY.moveTo(lroundf(clampedY * STEPS_PER_MM));
}

bool isGantryAtTarget() {
  return (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0);
}

void homeAxes() {
  currentState = STATE_HOMING;
  digitalWrite(PIN_STEPPER_EN, LOW); // Enable TMC2208 drivers

  // 1. Retract Z-axis and open gripper before any XY homing travel
  pca.setPWM(SERVO_Z_CH, 0, PWM_Z_RAISED);
  pca.setPWM(SERVO_GRIPPER_CH, 0, PWM_GRIP_OPEN);
  delay(400);

  // 2. Seek X & Y Normally-Closed home switches (LOW = closed, HIGH = hit/open)
  stepperX.setMaxSpeed(1600);
  stepperY.setMaxSpeed(1600);
  while (digitalRead(PIN_X_HOME) == LOW || digitalRead(PIN_Y_HOME) == LOW) {
    if (digitalRead(PIN_X_HOME) == LOW) {
      stepperX.setSpeed(-1200);
      stepperX.runSpeed();
    }
    if (digitalRead(PIN_Y_HOME) == LOW) {
      stepperY.setSpeed(-1200);
      stepperY.runSpeed();
    }
  }

  // 3. Back off 3.0 mm (240 steps) off the switches and zero coordinates
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  moveGantryToMm(3.0f, 3.0f);
  while (!isGantryAtTarget()) {
    stepperX.run();
    stepperY.run();
  }
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);

  // 4. Restore normal operating speed (4000 steps/s = 50 mm/s) & acceleration
  stepperX.setMaxSpeed(4000);
  stepperX.setAcceleration(2000);
  stepperY.setMaxSpeed(4000);
  stepperY.setAcceleration(2000);

  hasInFlightCap    = false;
  commsFaultLatched = false;
  lastHeartbeatMs   = millis();
  currentState      = STATE_IDLE;
}

bool readSerial1Bytes(uint8_t *dest, uint16_t len, unsigned long timeoutMs) {
  unsigned long start = millis();
  uint16_t idx = 0;
  while (idx < len) {
    if (Serial1.available() > 0) {
      dest[idx++] = (uint8_t)Serial1.read();
    } else if (millis() - start > timeoutMs) {
      return false;
    }
    stepperX.run();
    stepperY.run();
  }
  return true;
}

void pollVisionHostPackets() {
  while (Serial1.available() > 0) {
    uint8_t peekByte = (uint8_t)Serial1.read();

    // 1. Heartbeat Keep-Alive Ping from Teensy 4.1 or Raspberry Pi Zero 2 W
    if (peekByte == CMD_HEARTBEAT_PING) {
      lastHeartbeatMs = millis();
      continue;
    }

    // 2. New Batch Header (only accepted when IDLE)
    if (peekByte == PROTOCOL_MAGIC_BYTE && currentState == STATE_IDLE) {
      BatchHeader hdr;
      hdr.magic = PROTOCOL_MAGIC_BYTE;
      if (!readSerial1Bytes(((uint8_t*)&hdr) + 1, sizeof(BatchHeader) - 1, 200)) {
        Serial1.write(CMD_BATCH_NACK);
        return;
      }
      if (hdr.totalCaps == 0 || hdr.totalCaps > 100) {
        Serial1.write(CMD_BATCH_NACK);
        return;
      }

      uint16_t payloadBytes = (uint16_t)hdr.totalCaps * sizeof(CapTarget);
      if (!readSerial1Bytes((uint8_t*)batchQueue, payloadBytes, 500)) {
        Serial1.write(CMD_BATCH_NACK);
        return;
      }

      uint16_t calcCrc = computeCRC16((const uint8_t*)batchQueue, payloadBytes);
      if (calcCrc != hdr.crc16) {
        Serial1.write(CMD_BATCH_NACK);
        return;
      }

      // Valid batch received! Store locally & acknowledge
      batchTotal        = hdr.totalCaps;
      currentIdx        = 0;
      commsFaultLatched = false;
      lastHeartbeatMs   = millis();
      Serial1.write(CMD_BATCH_ACK);

      // Start Step 01 on first target cap
      currentCap = batchQueue[0];
      moveGantryToMm(currentCap.boardX_mm, currentCap.boardY_mm);
      currentState = STATE_MOVING_TO_CAP;
    }
  }
}

bool checkCommunicationLoss() {
  if (currentState == STATE_IDLE || currentState == STATE_HOMING ||
      currentState == STATE_FAILSAFE_HALT || commsFaultLatched) {
    return false;
  }
  return (millis() - lastHeartbeatMs > COMMS_TIMEOUT_MS);
}

void executeSafeLocalizedCompletion() {
  commsFaultLatched = true;
  if (hasInFlightCap) {
    // Cap is currently gripped: allow state machine to finish depositing
    // this cap into its assigned bin, then halt at STATE_RETRACT_Z!
    return;
  }
  // No cap gripped yet: immediately retract Z, open jaws, and park at (0,0)
  pca.setPWM(SERVO_Z_CH, 0, PWM_Z_RAISED);
  pca.setPWM(SERVO_GRIPPER_CH, 0, PWM_GRIP_OPEN);
  Serial1.write(CMD_RECOVERY_HALT);
  moveGantryToMm(0.0f, 0.0f);
  currentState = STATE_FAILSAFE_HALT;
}

void executeMotionStateMachine() {
  switch (currentState) {
    case STATE_IDLE:
    case STATE_HOMING:
      break;

    // Step 01: Traverse to Cap (X, Y) with Z raised and Gripper open
    case STATE_MOVING_TO_CAP:
      if (isGantryAtTarget()) {
        pca.setPWM(SERVO_Z_CH, 0, PWM_Z_LOWER_PICK);
        stateTimerMs = millis();
        currentState = STATE_LOWER_Z;
      }
      break;

    // Step 02: Lower Z linkage (~40 mm) over cap
    case STATE_LOWER_Z:
      if (millis() - stateTimerMs >= 350) {
        pca.setPWM(SERVO_GRIPPER_CH, 0, PWM_GRIP_CLOSE);
        hasInFlightCap = true; // Cap is now held by gripper!
        stateTimerMs = millis();
        currentState = STATE_GRIP_CAP;
      }
      break;

    // Step 03: Clamp Gripper jaws (~26 mm)
    case STATE_GRIP_CAP:
      if (millis() - stateTimerMs >= 300) {
        pca.setPWM(SERVO_Z_CH, 0, PWM_Z_RAISED);
        stateTimerMs = millis();
        currentState = STATE_LIFT_Z;
      }
      break;

    // Step 04: Lift Z linkage back to 0 mm overhead clearance
    case STATE_LIFT_Z:
      if (millis() - stateTimerMs >= 350) {
        uint8_t binId = constrain(currentCap.targetBinId, 1, 4);
        moveGantryToMm(BIN_X_MM[binId], BIN_Y_MM[binId]);
        currentState = STATE_TRAVERSE_BIN;
      }
      break;

    // Step 05: Rapid traverse to designated color bin
    case STATE_TRAVERSE_BIN:
      if (isGantryAtTarget()) {
        pca.setPWM(SERVO_Z_CH, 0, PWM_Z_LOWER_BIN);
        stateTimerMs = millis();
        currentState = STATE_LOWER_BIN;
      }
      break;

    // Step 06: Lower Z partially (~25 mm) into bin opening
    case STATE_LOWER_BIN:
      if (millis() - stateTimerMs >= 250) {
        pca.setPWM(SERVO_GRIPPER_CH, 0, PWM_GRIP_OPEN);
        stateTimerMs = millis();
        currentState = STATE_RELEASE_CAP;
      }
      break;

    // Step 07: Release Gripper jaws (cap drops into bin)
    case STATE_RELEASE_CAP:
      if (millis() - stateTimerMs >= 250) {
        hasInFlightCap = false;
        pca.setPWM(SERVO_Z_CH, 0, PWM_Z_RAISED);
        stateTimerMs = millis();
        currentState = STATE_RETRACT_Z;
      }
      break;

    // Step 08: Retract Z to 0 mm & check fail-safe or advance to next cap
    case STATE_RETRACT_Z:
      if (millis() - stateTimerMs >= 300) {
        if (commsFaultLatched) {
          // Fail-safe recovery completed in-flight cap drop; now park & halt
          Serial1.write(CMD_RECOVERY_HALT);
          moveGantryToMm(0.0f, 0.0f);
          currentState = STATE_FAILSAFE_HALT;
          return;
        }

        currentIdx++;
        if (currentIdx < batchTotal) {
          currentCap = batchQueue[currentIdx];
          moveGantryToMm(currentCap.boardX_mm, currentCap.boardY_mm);
          currentState = STATE_MOVING_TO_CAP;
        } else {
          // Batch finished! Return gantry to (0,0) out of camera FOV
          moveGantryToMm(0.0f, 0.0f);
          currentState = STATE_PARKING_HOME;
        }
      }
      break;

    case STATE_PARKING_HOME:
      if (isGantryAtTarget()) {
        batchTotal = 0;
        currentIdx = 0;
        Serial1.write(CMD_BATCH_COMPLETE);
        currentState = STATE_IDLE;
      }
      break;

    case STATE_FAILSAFE_HALT:
      // Safely parked at (0,0); wait for operator reset or new ping
      break;
  }
}

void setup() {
  pinMode(PIN_STEPPER_EN, OUTPUT);
  digitalWrite(PIN_STEPPER_EN, LOW); // Enable TMC2208 drivers (Active LOW)

  pinMode(PIN_X_HOME, INPUT_PULLUP); // NC limit switch
  pinMode(PIN_Y_HOME, INPUT_PULLUP); // NC limit switch
  pinMode(PIN_Z_HOME, INPUT_PULLUP); // NC limit switch

  Serial.begin(115200);   // USB Debug
  Serial1.begin(115200);  // Vision Host UART (Level shifted)

  Wire.begin();
  Wire.setClock(400000);  // 400 kHz Fast-Mode I2C
  pca.begin();
  pca.setPWMFreq(50);     // 50 Hz analog servo PWM

  homeAxes();
}

void loop() {
  stepperX.run();
  stepperY.run();

  // Poll incoming batch packets & 250ms heartbeat pings
  pollVisionHostPackets();

  // Execute non-blocking 8-step pick-and-place state machine
  executeMotionStateMachine();

  // Watchdog: trigger localized fail-safe recovery if UART link drops
  if (checkCommunicationLoss()) {
    executeSafeLocalizedCompletion();
  }
}
