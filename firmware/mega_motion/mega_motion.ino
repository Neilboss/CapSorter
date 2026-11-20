// =========================================================================
// Arduino Mega 2560 - Motion & Fail-Safe Sequencer (V1 & V2)
// Status: Pre-Validation Engineering Firmware (requires physical bench tests)
// Hardware: ATmega2560 @ 16 MHz | 2x TMC2208 Steppers | PCA9685 Servos
// Wiring:
//   Serial1 (Pin 19 RX1, Pin 18 TX1) <-> 5V/3.3V Shifter <-> Teensy or Pi Zero 2W
//   X Stepper (TMC2208): STEP = Pin 2, DIR = Pin 5, EN = Pin 8 (Active LOW)
//   Y Stepper (TMC2208): STEP = Pin 3, DIR = Pin 6, EN = Pin 8 (Active LOW)
//   Limit Switches (NC to GND, INPUT_PULLUP, polled in loop - Port A has no ISR):
//     X Home = Pin 22, Y Home = Pin 23, Z Limit = Pin 24
//   E-Stop Auxiliary Contact (NC to GND, INPUT_PULLUP): Pin 25
//     Primary E-stop pole MUST hard-cut 12V actuator power (Branch A + Branch B)
//     independently of firmware. Pin 25 latches STATE_HARD_FAULT & prevents
//     unexpected motion on E-stop release until explicit re-arm + re-home.
//   PCA9685 Servo Driver (I2C 0x40): SDA = Pin 20, SCL = Pin 21, OE = Pin 9
// =========================================================================
#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>
#include "../common/packet_protocol.h"

// Pin Definitions
#define PIN_X_STEP          2
#define PIN_X_DIR           5
#define PIN_Y_STEP          3
#define PIN_Y_DIR           6
#define PIN_STEPPER_EN      8   // Active LOW enable shared by both TMC2208 drivers
#define PIN_SERVO_OE        9   // Active LOW PCA9685 Output Enable (HIGH = outputs disabled)
#define PIN_X_HOME         22   // NC switch to GND: LOW = closed/normal, HIGH = open/fault
#define PIN_Y_HOME         23   // NC switch to GND: LOW = closed/normal, HIGH = open/fault
#define PIN_Z_HOME         24   // NC switch to GND: Z overtravel/retract limit (see mode below)
#define PIN_ESTOP          25   // NC auxiliary pole of latching E-stop to GND: HIGH = E-stop active

// Optional Hardware Feature Hooks (set to 1 only when physical hardware is wired & verified)
// Z_LIMIT_IS_OVERTRAVEL:
//   1 (default): Pin 24 is an NC overtravel/broken-wire limit above normal Z stroke (LOW=safe, HIGH=fault).
//   0:           Pin 24 is an NC retracted-home confirmation switch pressed when Z is raised (HIGH=retracted).
#define Z_LIMIT_IS_OVERTRAVEL  1
#define ENABLE_MAX_ENDSTOPS    0   // Set to 1 if positive-end NC limit switches are fitted
#define PIN_X_MAX             27   // Optional positive-end X NC limit switch to GND
#define PIN_Y_MAX             28   // Optional positive-end Y NC limit switch to GND
#define HAS_GRIP_SENSOR        0   // Set to 1 ONLY if physical cap-presence/grip sensor is fitted
#define PIN_GRIP_SENSE        26   // Optional active-LOW cap-presence sensor input

// Nominal 16 MHz AVR Kinematics (Open-Loop Stepper Control - Bench Verification Required):
// Nominal scaling: 200 steps/rev * 8 microsteps (TMC2208 MS1=GND, MS2=GND) / 40 mm GT2 pulley = 40.0 steps/mm.
// 0.025 mm/step is commanded step resolution, NOT verified positioning accuracy (measure actual travel on bench).
// 2800 steps/s = 70 mm/s per axis (<4000 steps/s 16 MHz AVR AccelStepper ceiling).
// 6000 steps/s^2 = 150 mm/s^2 belt acceleration (0.47 s ramp).
// Realistic cycle time: ~5-13 s/cap depending on board-to-bin travel (1.8 s fixed servo dwells + XY moves).
#define STEPS_PER_MM_X     40.0f
#define STEPS_PER_MM_Y     40.0f
#define MAX_SPEED_STEPS  2800.0f // 70 mm/s nominal linear speed
#define ACCEL_STEPS      6000.0f // 150 mm/s^2 smooth GT2 belt acceleration
#define HOMING_SPEED_STEPS 800.0f // 20 mm/s homing seek speed
#define MAX_TRAVEL_X_MM   370.0f // 300 mm board + bin zone (software clamp; guard mechanically!)
#define MAX_TRAVEL_Y_MM   300.0f // 300 mm board travel (software clamp; guard mechanically!)

// Physical Toolhead Offset (mm) from Homed (0,0) to Board (0,0) Fiducial Corner.
// MUST be measured on the physical machine; (0.0, 0.0) is an uncalibrated placeholder.
#define TOOL_OFFSET_X_MM    0.0f
#define TOOL_OFFSET_Y_MM    0.0f

// PCA9685 50Hz (12-bit, 4096 ticks / 20ms) Servo Pulse Counts (verify against exact servo SKU!)
#define SERVO_Z_CH          0
#define SERVO_GRIPPER_CH    1
#define PWM_Z_RAISED      180    // 0 mm (retracted overhead clearance)
#define PWM_Z_LOWER_PICK  410    // ~40 mm (lowered onto cap on board)
#define PWM_Z_LOWER_BIN   320    // ~25 mm (lowered into bin opening)
#define PWM_GRIP_OPEN     210    // ~36 mm jaw gap (open)
#define PWM_GRIP_CLOSE    385    // ~26 mm jaw gap (clamped on 28-32mm cap)

// Safety, Settle & Watchdog Timeouts
// Worst-case X homing: 370 mm / (800 / 40 = 20 mm/s) = 18.5 s -> 30 s timeout (~1.6x margin).
#define BIN_SETTLE_MS        120UL  // Stationary XY settle at bin (Z raised) before bin-side camera shutter
#define COMMS_TIMEOUT_MS    1200UL  // 1.2 s without 0xF0 heartbeat triggers fail-safe halt
#define HOMING_TIMEOUT_MS  30000UL  // 30.0 s max homing seek duration
#define BACKOFF_TIMEOUT_MS  3000UL  // 3.0 s max duration for pre-check / switch-release backoff

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
  STATE_BIN_SETTLE,
  STATE_LOWER_BIN,
  STATE_RELEASE_CAP,
  STATE_RETRACT_Z,
  STATE_AWAIT_NEXT_TARGET,
  STATE_PARKING_FOR_SCAN,
  STATE_PARKING_HOME,
  STATE_FAILSAFE_PARKING,
  STATE_FAILSAFE_HALT,
  STATE_HARD_FAULT
};

RobotState    currentState      = STATE_HOMING;
CapTarget     batchQueue[MAX_BATCH_CAPS];
CapTarget     currentCap;
uint8_t       batchTotal        = 0;
uint8_t       currentIdx        = 0;
bool          closedLoopMode    = false;
bool          isHomed           = false;
bool          hasInFlightCap    = false;
bool          commsFaultLatched = false;
uint8_t       limitFaultCount   = 0;
uint8_t       estopFaultCount   = 0;
unsigned long stateTimerMs      = 0;
unsigned long lastHeartbeatMs   = 0;

bool isEstopActive() {
  return digitalRead(PIN_ESTOP) == HIGH;
}

bool isAxisLimitOpen(uint8_t pin) {
  return digitalRead(pin) == HIGH;
}

bool isAnyOvertravelLimitOpen() {
  if (isAxisLimitOpen(PIN_X_HOME) || isAxisLimitOpen(PIN_Y_HOME)) {
    return true;
  }
#if Z_LIMIT_IS_OVERTRAVEL
  if (isAxisLimitOpen(PIN_Z_HOME)) {
    return true;
  }
#endif
#if ENABLE_MAX_ENDSTOPS
  if (isAxisLimitOpen(PIN_X_MAX) || isAxisLimitOpen(PIN_Y_MAX)) {
    return true;
  }
#endif
  return false;
}

bool isZRetractedSafeForXY() {
#if !Z_LIMIT_IS_OVERTRAVEL
  // In retracted-confirmation mode, NC switch opens (HIGH) when Z reaches top stop
  return digitalRead(PIN_Z_HOME) == HIGH;
#else
  return !isAxisLimitOpen(PIN_Z_HOME);
#endif
}

bool isCapVerifiedHeld() {
#if HAS_GRIP_SENSOR
  return hasInFlightCap && (digitalRead(PIN_GRIP_SENSE) == LOW);
#else
  // Without physical cap-presence or grip-force sensing, grip cannot be verified
  return false;
#endif
}

void moveGantryToMm(float boardX_mm, float boardY_mm) {
  float targetX = constrain(boardX_mm + TOOL_OFFSET_X_MM, 0.0f, MAX_TRAVEL_X_MM);
  float targetY = constrain(boardY_mm + TOOL_OFFSET_Y_MM, 0.0f, MAX_TRAVEL_Y_MM);
  stepperX.moveTo(lroundf(targetX * STEPS_PER_MM_X));
  stepperY.moveTo(lroundf(targetY * STEPS_PER_MM_Y));
}

bool isGantryAtTarget() {
  return (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0);
}

void replyStatusQuery() {
  if (currentState == STATE_IDLE && isHomed && !commsFaultLatched && !isEstopActive()) {
    Serial1.write(CMD_STATUS_READY);
  } else if (currentState == STATE_FAILSAFE_HALT || currentState == STATE_HARD_FAULT) {
    Serial1.write(CMD_STATUS_FAULT);
  } else {
    Serial1.write(CMD_STATUS_BUSY);
  }
}

void serviceSerialDuringHoming() {
  // Drain Serial1 non-blockingly while homing so host queries get BUSY and FIFO never stalls
  while (Serial1.available() > 0) {
    uint8_t b = (uint8_t)Serial1.read();
    if (b == CMD_QUERY_STATUS) {
      Serial1.write(CMD_STATUS_BUSY);
    } else if (b == PROTOCOL_MAGIC_BYTE) {
      Serial1.write(CMD_BATCH_NACK);
    }
  }
}

void triggerHardFault(const __FlashStringHelper *reason) {
  stepperX.setSpeed(0.0f);
  stepperY.setSpeed(0.0f);
  stepperX.moveTo(stepperX.currentPosition());
  stepperY.moveTo(stepperY.currentPosition());
  digitalWrite(PIN_STEPPER_EN, HIGH); // Disable stepper coils immediately
  digitalWrite(PIN_SERVO_OE, HIGH);   // Disable PCA9685 PWM outputs (no commanded fault motion)
  isHomed           = false;
  hasInFlightCap    = false;
  batchTotal        = 0;
  currentIdx        = 0;
  currentState      = STATE_HARD_FAULT;
  Serial1.write(CMD_HARD_FAULT);
  Serial.print(F("[HARD_FAULT] "));
  Serial.println(reason);
}

bool waitServicedDelayMs(unsigned long durationMs) {
  unsigned long start = millis();
  while (millis() - start < durationMs) {
    if (isEstopActive()) {
      triggerHardFault(F("E-Stop active during servo settle"));
      return false;
    }
    serviceSerialDuringHoming();
  }
  return true;
}

void homeAxes() {
  currentState      = STATE_HOMING;
  isHomed           = false;
  hasInFlightCap    = false;
  commsFaultLatched = false;
  limitFaultCount   = 0;
  estopFaultCount   = 0;
  batchTotal        = 0;
  currentIdx        = 0;

  // 0. Refuse homing if hardwired E-stop auxiliary contact is active or Z overtravel is open
  if (isEstopActive()) {
    triggerHardFault(F("E-Stop active at homing start"));
    return;
  }
#if Z_LIMIT_IS_OVERTRAVEL
  if (isAxisLimitOpen(PIN_Z_HOME)) {
    triggerHardFault(F("Z overtravel switch open or broken wire at homing start"));
    return;
  }
#endif

  // 1. Retract Z-axis and open gripper cleanly before energizing XY steppers
  pca.setPWM(SERVO_Z_CH, 0, PWM_Z_RAISED);
  pca.setPWM(SERVO_GRIPPER_CH, 0, PWM_GRIP_OPEN);
  digitalWrite(PIN_SERVO_OE, LOW);    // Enable PCA9685 outputs after latching safe PWM
  if (!waitServicedDelayMs(450UL)) return;

  if (!isZRetractedSafeForXY()) {
    triggerHardFault(F("Z-axis did not confirm retracted safe state before XY homing"));
    return;
  }

  digitalWrite(PIN_STEPPER_EN, LOW);  // Enable TMC2208 drivers (Active LOW)
  stepperX.setMinPulseWidth(2);
  stepperY.setMinPulseWidth(2);
  stepperX.setMaxSpeed(1000.0f);
  stepperY.setMaxSpeed(1000.0f);
  stepperX.setAcceleration(800.0f);
  stepperY.setAcceleration(800.0f);

  // 2. Broken NC Wire Pre-Check: Back off ONLY the axis whose switch already reads HIGH
  bool xInitiallyHigh = isAxisLimitOpen(PIN_X_HOME);
  bool yInitiallyHigh = isAxisLimitOpen(PIN_Y_HOME);
  if (xInitiallyHigh || yInitiallyHigh) {
    stepperX.setCurrentPosition(0);
    stepperY.setCurrentPosition(0);
    stepperX.moveTo(xInitiallyHigh ? lroundf(10.0f * STEPS_PER_MM_X) : 0L);
    stepperY.moveTo(yInitiallyHigh ? lroundf(10.0f * STEPS_PER_MM_Y) : 0L);
    unsigned long preStart = millis();
    while (!isGantryAtTarget()) {
      if (isEstopActive()) {
        triggerHardFault(F("E-Stop triggered during pre-homing backoff"));
        return;
      }
      if (millis() - preStart > BACKOFF_TIMEOUT_MS) {
        triggerHardFault(F("Pre-homing backoff timeout"));
        return;
      }
      stepperX.run();
      stepperY.run();
      serviceSerialDuringHoming();
    }
    // If switch STILL reads HIGH after +10 mm backoff, NC wire is broken/unplugged or stuck!
    if (isAxisLimitOpen(PIN_X_HOME) || isAxisLimitOpen(PIN_Y_HOME)) {
      triggerHardFault(F("X/Y NC home switch stuck open or wire broken"));
      return;
    }
  }

  // 3. Seek X & Y Normally-Closed home switches at 20 mm/s with 30 s worst-case timeout
  unsigned long homeStartMs = millis();
  while (!isAxisLimitOpen(PIN_X_HOME) || !isAxisLimitOpen(PIN_Y_HOME)) {
    if (isEstopActive()) {
      triggerHardFault(F("E-Stop triggered during homing seek"));
      return;
    }
#if Z_LIMIT_IS_OVERTRAVEL
    if (isAxisLimitOpen(PIN_Z_HOME)) {
      triggerHardFault(F("Z overtravel limit tripped during homing seek"));
      return;
    }
#endif
    if (millis() - homeStartMs > HOMING_TIMEOUT_MS) {
      triggerHardFault(F("Homing seek timeout (>30s): stall, belt slip, or switch failure"));
      return;
    }
    if (!isAxisLimitOpen(PIN_X_HOME)) {
      stepperX.setSpeed(-HOMING_SPEED_STEPS);
      stepperX.runSpeed();
    }
    if (!isAxisLimitOpen(PIN_Y_HOME)) {
      stepperY.setSpeed(-HOMING_SPEED_STEPS);
      stepperY.runSpeed();
    }
    serviceSerialDuringHoming();
  }

  // 4. Back off 3.0 mm off switches and VERIFY switches re-close (LOW) within timeout
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  stepperX.moveTo(lroundf(3.0f * STEPS_PER_MM_X));
  stepperY.moveTo(lroundf(3.0f * STEPS_PER_MM_Y));
  unsigned long releaseStartMs = millis();
  while (!isGantryAtTarget()) {
    if (isEstopActive()) {
      triggerHardFault(F("E-Stop triggered during homing switch release"));
      return;
    }
    if (millis() - releaseStartMs > BACKOFF_TIMEOUT_MS) {
      triggerHardFault(F("Homing switch release backoff timeout"));
      return;
    }
    stepperX.run();
    stepperY.run();
    serviceSerialDuringHoming();
  }
  if (isAxisLimitOpen(PIN_X_HOME) || isAxisLimitOpen(PIN_Y_HOME)) {
    triggerHardFault(F("X/Y home switch did not re-close after 3mm backoff"));
    return;
  }

  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);

  // 5. Restore operating speed (2800 steps/s = 70 mm/s) & acceleration (6000 steps/s^2 = 150 mm/s^2)
  stepperX.setMaxSpeed(MAX_SPEED_STEPS);
  stepperX.setAcceleration(ACCEL_STEPS);
  stepperY.setMaxSpeed(MAX_SPEED_STEPS);
  stepperY.setAcceleration(ACCEL_STEPS);

  // Drain any stale bytes before declaring READY
  while (Serial1.available() > 0) {
    Serial1.read();
  }

  isHomed         = true;
  lastHeartbeatMs = millis();
  currentState    = STATE_IDLE;
  Serial1.write(CMD_STATUS_READY);
  Serial.println(F("[READY] Homing complete. Gantry at (0, 0)."));
}

bool readSerial1Bytes(uint8_t *dest, uint16_t len, unsigned long timeoutMs) {
  // Called only during STATE_IDLE when steppers are stationary; drains 64B FIFO at max rate
  unsigned long start = millis();
  uint16_t idx = 0;
  while (idx < len) {
    if (isEstopActive()) {
      triggerHardFault(F("E-Stop triggered during batch receive"));
      return false;
    }
    if (Serial1.available() > 0) {
      dest[idx++] = (uint8_t)Serial1.read();
    } else if (millis() - start > timeoutMs) {
      return false;
    }
  }
  return true;
}

void pollVisionHostPackets() {
  while (Serial1.available() > 0) {
    uint8_t peekByte = (uint8_t)Serial1.read();

    // 1. Explicit Host Readiness Query
    if (peekByte == CMD_QUERY_STATUS) {
      replyStatusQuery();
      continue;
    }

    // 2. Heartbeat Keep-Alive Ping (never auto-clears faults or initiates motion!)
    if (peekByte == CMD_HEARTBEAT_PING) {
      lastHeartbeatMs = millis();
      continue;
    }

    // 3. Explicit Operator Re-Arm & Re-Home Command from Fault/Halt State
    if (peekByte == CMD_REARM_HOME) {
      if (currentState == STATE_FAILSAFE_HALT || currentState == STATE_HARD_FAULT) {
        if (isEstopActive()) {
          Serial.println(F("[REARM_REJECTED] Release physical E-Stop before re-arming."));
          Serial1.write(CMD_STATUS_FAULT);
          continue;
        }
        Serial.println(F("[REARM] Explicit re-arm accepted; starting full re-home."));
        homeAxes();
      } else {
        replyStatusQuery();
      }
      continue;
    }

    // 4. New Batch Header (ONLY accepted when homed, fault-free, and IDLE)
    if (peekByte == PROTOCOL_MAGIC_BYTE) {
      if (currentState != STATE_IDLE || !isHomed || commsFaultLatched || isEstopActive()) {
        Serial1.write(CMD_BATCH_NACK);
        continue;
      }

      BatchHeader hdr;
      hdr.magic = PROTOCOL_MAGIC_BYTE;
      if (!readSerial1Bytes(((uint8_t*)&hdr) + 1, sizeof(BatchHeader) - 1, 250)) {
        if (currentState == STATE_IDLE) Serial1.write(CMD_BATCH_NACK);
        return;
      }
      if (hdr.totalCaps == 0 || hdr.totalCaps > MAX_BATCH_CAPS) {
        Serial1.write(CMD_BATCH_NACK);
        return;
      }

      uint16_t payloadBytes = (uint16_t)hdr.totalCaps * sizeof(CapTarget);
      if (!readSerial1Bytes((uint8_t*)batchQueue, payloadBytes, 1500)) {
        if (currentState == STATE_IDLE) Serial1.write(CMD_BATCH_NACK);
        return;
      }

      uint16_t calcCrc = computeCRC16((const uint8_t*)batchQueue, payloadBytes);
      if (calcCrc != hdr.crc16) {
        Serial1.write(CMD_BATCH_NACK);
        return;
      }

      if (!isZRetractedSafeForXY()) {
        triggerHardFault(F("Z-axis not retracted before batch start"));
        return;
      }

      // Valid batch received! Store locally & acknowledge
      batchTotal        = hdr.totalCaps;
      currentIdx        = 0;
      hasInFlightCap    = false;
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

void checkRuntimeSafetyInputs() {
  if (currentState == STATE_HOMING || currentState == STATE_HARD_FAULT) return;

  // Priority 1: Auxiliary NC E-Stop contact (2 consecutive samples)
  if (isEstopActive()) {
    estopFaultCount++;
    if (estopFaultCount >= 2) {
      triggerHardFault(F("Physical E-Stop auxiliary contact open"));
      return;
    }
  } else {
    estopFaultCount = 0;
  }

  // Priority 2: Polled NC limit switches for X, Y, and Z (3-sample debounce)
  if (isAnyOvertravelLimitOpen()) {
    limitFaultCount++;
    if (limitFaultCount >= 3) {
      triggerHardFault(F("Runtime X/Y/Z NC limit switch open or broken wire"));
    }
  } else {
    limitFaultCount = 0;
  }
}

bool checkCommunicationLoss() {
  if (currentState == STATE_IDLE || currentState == STATE_HOMING ||
      currentState == STATE_FAILSAFE_PARKING || currentState == STATE_FAILSAFE_HALT ||
      currentState == STATE_HARD_FAULT || commsFaultLatched) {
    return false;
  }
  return (millis() - lastHeartbeatMs > COMMS_TIMEOUT_MS);
}

void executeSafeCommsLossPolicy() {
  commsFaultLatched = true;

  // Only continue bin deposit if a physical grip sensor verifies a cap is held
  // AND Z has already lifted the cap clear of the workspace.
  bool inCarryStage = (currentState == STATE_TRAVERSE_BIN ||
                       currentState == STATE_LOWER_BIN ||
                       currentState == STATE_RELEASE_CAP);
  if (inCarryStage && isCapVerifiedHeld()) {
    Serial.println(F("[COMMS_LOSS] Verified held cap in transit; completing deposit then halting."));
    return;
  }

  // Default fail-closed policy (un-sensed gripper or not carrying):
  // Never assume a cap was picked and never traverse blindly to a bin.
  // Stop XY immediately, raise Z safely (preserving jaw state), disable steppers,
  // latch STATE_FAILSAFE_HALT, and require explicit CMD_REARM_HOME + re-home.
  stepperX.setSpeed(0.0f);
  stepperY.setSpeed(0.0f);
  stepperX.moveTo(stepperX.currentPosition());
  stepperY.moveTo(stepperY.currentPosition());
  digitalWrite(PIN_STEPPER_EN, HIGH); // Remove stepper drive torque safely
  pca.setPWM(SERVO_Z_CH, 0, PWM_Z_RAISED); // Retract Z clear of board; preserve jaw state
  isHomed        = false;
  hasInFlightCap = false;
  batchTotal     = 0;
  currentIdx     = 0;
  currentState   = STATE_FAILSAFE_HALT;
  Serial1.write(CMD_RECOVERY_HALT);
  Serial.println(F("[COMMS_LOSS] Fail-safe halt latched. Explicit re-arm & re-home required."));
}

void executeMotionStateMachine() {
  switch (currentState) {
    case STATE_IDLE:
    case STATE_HOMING:
    case STATE_FAILSAFE_HALT:
    case STATE_HARD_FAULT:
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
        stateTimerMs = millis();
        currentState = STATE_GRIP_CAP;
      }
      break;

    // Step 03: Clamp Gripper jaws (~26 mm)
    case STATE_GRIP_CAP:
      if (millis() - stateTimerMs >= 300) {
        hasInFlightCap = true; // Commanded grip complete (unverified unless HAS_GRIP_SENSOR == 1)
        pca.setPWM(SERVO_Z_CH, 0, PWM_Z_RAISED);
        stateTimerMs = millis();
        currentState = STATE_LIFT_Z;
      }
      break;

    // Step 04: Lift Z linkage back to 0 mm overhead clearance
    case STATE_LIFT_Z:
      if (millis() - stateTimerMs >= 350) {
        if (!isZRetractedSafeForXY()) {
          triggerHardFault(F("Z-axis failed retracted check before bin traverse"));
          return;
        }
        uint8_t binId = constrain(currentCap.targetBinId, 1, 4);
        moveGantryToMm(BIN_X_MM[binId], BIN_Y_MM[binId]);
        currentState = STATE_TRAVERSE_BIN;
      }
      break;

    // Step 05: Traverse to designated color bin
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

    // Step 08: Retract Z to 0 mm, emit progress pulse, advance or halt
    case STATE_RETRACT_Z:
      if (millis() - stateTimerMs >= 300) {
        if (!isZRetractedSafeForXY()) {
          triggerHardFault(F("Z-axis failed retracted check after bin release"));
          return;
        }

        Serial1.write(CMD_CAP_PROGRESS);

        if (commsFaultLatched) {
          // Verified in-flight cap deposit finished; park at (0,0) then require explicit re-arm
          Serial1.write(CMD_RECOVERY_HALT);
          moveGantryToMm(0.0f, 0.0f);
          currentState = STATE_FAILSAFE_PARKING;
          return;
        }

        currentIdx++;
        if (currentIdx < batchTotal) {
          currentCap = batchQueue[currentIdx];
          moveGantryToMm(currentCap.boardX_mm, currentCap.boardY_mm);
          currentState = STATE_MOVING_TO_CAP;
        } else {
          // Batch finished! Return gantry to (0,0) clear of camera FOV
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

    case STATE_FAILSAFE_PARKING:
      if (isGantryAtTarget()) {
        digitalWrite(PIN_STEPPER_EN, HIGH);
        isHomed    = false;
        batchTotal = 0;
        currentIdx = 0;
        currentState = STATE_FAILSAFE_HALT;
      }
      break;
  }
}

void setup() {
  // Keep actuators disabled during power-up until E-stop & I2C are initialized
  pinMode(PIN_SERVO_OE, OUTPUT);
  digitalWrite(PIN_SERVO_OE, HIGH);

  pinMode(PIN_STEPPER_EN, OUTPUT);
  digitalWrite(PIN_STEPPER_EN, HIGH);

  pinMode(PIN_X_HOME, INPUT_PULLUP);
  pinMode(PIN_Y_HOME, INPUT_PULLUP);
  pinMode(PIN_Z_HOME, INPUT_PULLUP);
  pinMode(PIN_ESTOP,  INPUT_PULLUP);

#if ENABLE_MAX_ENDSTOPS
  pinMode(PIN_X_MAX, INPUT_PULLUP);
  pinMode(PIN_Y_MAX, INPUT_PULLUP);
#endif
#if HAS_GRIP_SENSOR
  pinMode(PIN_GRIP_SENSE, INPUT_PULLUP);
#endif

  Serial.begin(115200);   // USB Debug
  Serial1.begin(115200);  // Vision Host UART (Bidirectional 5V <-> 3.3V Level Shifter)

  Serial.println(F("=== CapSorter Mega Motion Controller (Pre-Validation) ==="));
  Serial.print(F("Nominal Scale (steps/mm): X=")); Serial.print(STEPS_PER_MM_X);
  Serial.print(F(" Y=")); Serial.println(STEPS_PER_MM_Y);
  Serial.print(F("Tool Offset (mm): dX=")); Serial.print(TOOL_OFFSET_X_MM);
  Serial.print(F(" dY=")); Serial.println(TOOL_OFFSET_Y_MM);
  if (TOOL_OFFSET_X_MM == 0.0f && TOOL_OFFSET_Y_MM == 0.0f) {
    Serial.println(F("[WARN] TOOL_OFFSET is (0,0) placeholder; measure physical alignment on bench."));
  }

  Wire.begin();
  Wire.setClock(400000);  // 400 kHz Fast-Mode I2C
  pca.begin();
  pca.setPWMFreq(50);     // 50 Hz analog servo PWM

  homeAxes();
}

void loop() {
  // 1. Poll E-stop auxiliary contact and X/Y/Z NC limit switches
  checkRuntimeSafetyInputs();

  // 2. Step only when actively moving and fault-free
  if (currentState != STATE_HARD_FAULT && currentState != STATE_FAILSAFE_HALT) {
    stepperX.run();
    stepperY.run();
  }

  // 3. Poll host status queries, batch packets, heartbeat pings & re-arm commands
  pollVisionHostPackets();

  // 4. Execute non-blocking 8-step pick-and-place state machine
  executeMotionStateMachine();

  // 5. Watchdog: latch fail-safe halt if UART heartbeat drops during active motion
  if (checkCommunicationLoss()) {
    executeSafeCommsLossPolicy();
  }
}
