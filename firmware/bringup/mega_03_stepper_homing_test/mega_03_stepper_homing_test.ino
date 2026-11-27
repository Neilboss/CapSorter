// =========================================================================
// Stage 3 Bench Test: Arduino Mega 2560 — Stepper Polarity, Homing,
//                     STEPS_PER_MM, TOOL_OFFSET & Bin Coordinate Finder
// Purpose:  1. Verify X & Y stepper direction BEFORE first auto-home.
//           2. Test full NC switch homing with broken-wire pre-check.
//           3. Measure true STEPS_PER_MM_X / STEPS_PER_MM_Y with a ruler.
//           4. Jog gripper directly over Board Corner (0,0) -> TOOL_OFFSET.
//           5. Jog gripper over Bins 1..4 centers -> BIN_X_MM / BIN_Y_MM.
// Usage:    Upload, open Serial Monitor @ 115200 baud (Newline ending).
//           Keep one hand on the physical E-Stop during every test!
// =========================================================================
#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AccelStepper.h>

#define PIN_X_STEP          2
#define PIN_X_DIR           5
#define PIN_Y_STEP          3
#define PIN_Y_DIR           6
#define PIN_STEPPER_EN      8
#define PIN_SERVO_OE        9
#define PIN_X_HOME         22
#define PIN_Y_HOME         23
#define PIN_Z_HOME         24
#define PIN_ESTOP          25

// Paste your Stage 2 calibrated servo values here before running Stage 3!
#define SERVO_Z_CH          0
#define SERVO_GRIPPER_CH    1
#define PWM_Z_RAISED      180
#define PWM_Z_LOWER_PICK  410
#define PWM_Z_LOWER_BIN   320
#define PWM_GRIP_OPEN     210
#define PWM_GRIP_CLOSE    385

// Initial nominal scaling (update after ruler measurement!)
#define STEPS_PER_MM_X          40.0f
#define STEPS_PER_MM_Y          40.0f
#define JOG_SPEED_STEPS       1000.0f  // Slow, safe 25 mm/s jog speed
#define JOG_ACCEL_STEPS       2500.0f
#define HOMING_SPEED           800.0f  // 20 mm/s homing seek speed
#define MAX_TRAVEL_X_MM        370.0f
#define MAX_TRAVEL_Y_MM        300.0f
#define HOMING_SEEK_MARGIN_MM   25.0f
#define MAX_HOMING_SEEK_X_MM   (MAX_TRAVEL_X_MM + HOMING_SEEK_MARGIN_MM) // 395 mm
#define MAX_HOMING_SEEK_Y_MM   (MAX_TRAVEL_Y_MM + HOMING_SEEK_MARGIN_MM) // 325 mm
#define MAX_HOMING_SEEK_X_STEPS ((long)(MAX_HOMING_SEEK_X_MM * STEPS_PER_MM_X))
#define MAX_HOMING_SEEK_Y_STEPS ((long)(MAX_HOMING_SEEK_Y_MM * STEPS_PER_MM_Y))
#define HOMING_TIMEOUT_MS      23000UL // Derived from 395 mm / 20 mm/s (19.75 s) + 3.25 s margin
#define MAX_UNHOMED_JOG_MM      50.0f  // Clamp relative jogs when unhomed so typos cannot crash frame

AccelStepper stepperX(AccelStepper::DRIVER, PIN_X_STEP, PIN_X_DIR);
AccelStepper stepperY(AccelStepper::DRIVER, PIN_Y_STEP, PIN_Y_DIR);
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

bool isHomed = false;
bool faulted = false;
bool steppersEnabled = false;
String cmdBuf = "";

bool isEstop()     { return digitalRead(PIN_ESTOP)  == HIGH; }
bool isXHomeOpen() { return digitalRead(PIN_X_HOME) == HIGH; }
bool isYHomeOpen() { return digitalRead(PIN_Y_HOME) == HIGH; }
bool isZLimitOpen(){ return digitalRead(PIN_Z_HOME) == HIGH; }

void emergencyStop(const __FlashStringHelper *msg) {
  stepperX.setSpeed(0.0f);
  stepperY.setSpeed(0.0f);
  stepperX.moveTo(stepperX.currentPosition());
  stepperY.moveTo(stepperY.currentPosition());
  digitalWrite(PIN_STEPPER_EN, HIGH);
  digitalWrite(PIN_SERVO_OE, HIGH);
  steppersEnabled = false;
  isHomed = false;
  faulted = true;
  Serial.print(F("[HALT] "));
  Serial.println(msg);
}

void printPosition() {
  float mx = (float)stepperX.currentPosition() / STEPS_PER_MM_X;
  float my = (float)stepperY.currentPosition() / STEPS_PER_MM_Y;
  Serial.print(F("[POS] Machine X = ")); Serial.print(mx, 2);
  Serial.print(F(" mm (")); Serial.print(stepperX.currentPosition());
  Serial.print(F(" steps) | Y = ")); Serial.print(my, 2);
  Serial.print(F(" mm (")); Serial.print(stepperY.currentPosition());
  Serial.print(F(" steps) | Homed=")); Serial.println(isHomed ? F("YES") : F("NO"));
}

void runHomeSequence() {
  if (isEstop() || isZLimitOpen()) {
    emergencyStop(F("E-Stop or Z limit open at homing start."));
    return;
  }
  faulted = false;
  pca.setPWM(SERVO_Z_CH, 0, PWM_Z_RAISED);
  pca.setPWM(SERVO_GRIPPER_CH, 0, PWM_GRIP_OPEN);
  digitalWrite(PIN_SERVO_OE, LOW);
  delay(450);

  digitalWrite(PIN_STEPPER_EN, LOW);
  steppersEnabled = true;

  // Pre-check: back off +10 mm if already resting on a switch
  bool xHigh = isXHomeOpen();
  bool yHigh = isYHomeOpen();
  if (xHigh || yHigh) {
    stepperX.setCurrentPosition(0);
    stepperY.setCurrentPosition(0);
    stepperX.moveTo(xHigh ? lroundf(10.0f * STEPS_PER_MM_X) : 0L);
    stepperY.moveTo(yHigh ? lroundf(10.0f * STEPS_PER_MM_Y) : 0L);
    unsigned long t0 = millis();
    while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
      if (isEstop()) { emergencyStop(F("E-Stop during backoff")); return; }
      if (millis() - t0 > 3000UL) { emergencyStop(F("Backoff timeout")); return; }
      stepperX.run(); stepperY.run();
    }
    if (isXHomeOpen() || isYHomeOpen()) {
      emergencyStop(F("Switch still open after +10mm backoff (broken NC wire?)"));
      return;
    }
  }

  Serial.println(F("[HOME] Seeking X & Y home switches at 20 mm/s (step- & time-bounded)..."));
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  unsigned long t0 = millis();
  while (!isXHomeOpen() || !isYHomeOpen()) {
    if (isEstop() || isZLimitOpen()) { emergencyStop(F("E-Stop/Z fault during seek")); return; }
    if (labs(stepperX.currentPosition()) > MAX_HOMING_SEEK_X_STEPS) {
      emergencyStop(F("Homing X seek exceeded 395 mm bound (missing switch or belt slip)"));
      return;
    }
    if (labs(stepperY.currentPosition()) > MAX_HOMING_SEEK_Y_STEPS) {
      emergencyStop(F("Homing Y seek exceeded 325 mm bound (missing switch or belt slip)"));
      return;
    }
    if (millis() - t0 > HOMING_TIMEOUT_MS) { emergencyStop(F("Homing seek timeout (>23s)")); return; }
    if (!isXHomeOpen()) { stepperX.setSpeed(-HOMING_SPEED); stepperX.runSpeed(); }
    if (!isYHomeOpen()) { stepperY.setSpeed(-HOMING_SPEED); stepperY.runSpeed(); }
  }

  // Back off 3.0 mm and verify re-close
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  stepperX.moveTo(lroundf(3.0f * STEPS_PER_MM_X));
  stepperY.moveTo(lroundf(3.0f * STEPS_PER_MM_Y));
  t0 = millis();
  while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
    if (isEstop()) { emergencyStop(F("E-Stop during release")); return; }
    if (millis() - t0 > 3000UL) { emergencyStop(F("Release backoff timeout")); return; }
    stepperX.run(); stepperY.run();
  }
  if (isXHomeOpen() || isYHomeOpen()) {
    emergencyStop(F("Switch did not re-close after 3 mm release"));
    return;
  }

  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  isHomed = true;
  Serial.println(F("[HOME OK] Gantry homed at machine (0.00, 0.00) mm."));
  printPosition();
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  en / dis        Enable / Disable stepper coils"));
  Serial.println(F("  x <+/-mm>       Relative X jog (e.g. 'x +10' or 'x -10')"));
  Serial.println(F("  y <+/-mm>       Relative Y jog (e.g. 'y +10' or 'y -10')"));
  Serial.println(F("  home            Run full NC switch homing -> machine (0,0)"));
  Serial.println(F("  go <X> <Y>      Move to absolute machine (X,Y) mm (requires home)"));
  Serial.println(F("  zup / zpick / zbin / open / close   Test Z & gripper over a spot"));
  Serial.println(F("  pos             Print current machine (X,Y) mm for recording offsets"));
  Serial.println(F("  stop            Immediate stop"));
}

bool canAcceptMotionCmd() {
  if (isEstop()) {
    emergencyStop(F("Command blocked: physical E-Stop is active."));
    return false;
  }
  if (faulted) {
    Serial.println(F("[BLOCKED] Fault is latched. Clear cause and type 'en' or 'home' to recover."));
    return false;
  }
  if (!steppersEnabled) {
    Serial.println(F("[BLOCKED] Actuators are disabled. Type 'en' (or 'home') first."));
    return false;
  }
  return true;
}

void handleCommand(String line) {
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) return;

  if (line == "help" || line == "?") {
    printHelp();
  } else if (line == "stop") {
    stepperX.setSpeed(0.0f);
    stepperY.setSpeed(0.0f);
    stepperX.moveTo(stepperX.currentPosition());
    stepperY.moveTo(stepperY.currentPosition());
    Serial.println(F("[STOP] Motion target cleared."));
  } else if (line == "dis") {
    stepperX.setSpeed(0.0f);
    stepperY.setSpeed(0.0f);
    stepperX.moveTo(stepperX.currentPosition());
    stepperY.moveTo(stepperY.currentPosition());
    digitalWrite(PIN_STEPPER_EN, HIGH);
    digitalWrite(PIN_SERVO_OE, HIGH);
    steppersEnabled = false;
    isHomed = false;
    Serial.println(F("[EN] Steppers & servos DISABLED; pending motion targets cleared."));
  } else if (line == "en") {
    if (isEstop()) { Serial.println(F("[BLOCKED] Release E-Stop first.")); return; }
    if (isZLimitOpen()) { Serial.println(F("[BLOCKED] Clear Z limit switch first.")); return; }
    stepperX.setSpeed(0.0f);
    stepperY.setSpeed(0.0f);
    stepperX.moveTo(stepperX.currentPosition());
    stepperY.moveTo(stepperY.currentPosition());
    faulted = false;
    pca.setPWM(SERVO_Z_CH, 0, PWM_Z_RAISED);
    pca.setPWM(SERVO_GRIPPER_CH, 0, PWM_GRIP_OPEN);
    digitalWrite(PIN_SERVO_OE, LOW);
    digitalWrite(PIN_STEPPER_EN, LOW);
    steppersEnabled = true;
    Serial.println(F("[EN] Steppers ENABLED (targets synced to current pos, Z raised, gripper open)."));
  } else if (line == "home") {
    runHomeSequence();
  } else if (line == "pos") {
    printPosition();
  } else if (line == "zup") {
    if (!canAcceptMotionCmd()) return;
    pca.setPWM(SERVO_Z_CH, 0, PWM_Z_RAISED); Serial.println(F("[Z] RAISED"));
  } else if (line == "zpick") {
    if (!canAcceptMotionCmd()) return;
    pca.setPWM(SERVO_Z_CH, 0, PWM_Z_LOWER_PICK); Serial.println(F("[Z] LOWER_PICK"));
  } else if (line == "zbin") {
    if (!canAcceptMotionCmd()) return;
    pca.setPWM(SERVO_Z_CH, 0, PWM_Z_LOWER_BIN); Serial.println(F("[Z] LOWER_BIN"));
  } else if (line == "open") {
    if (!canAcceptMotionCmd()) return;
    pca.setPWM(SERVO_GRIPPER_CH, 0, PWM_GRIP_OPEN); Serial.println(F("[GRIP] OPEN"));
  } else if (line == "close") {
    if (!canAcceptMotionCmd()) return;
    pca.setPWM(SERVO_GRIPPER_CH, 0, PWM_GRIP_CLOSE); Serial.println(F("[GRIP] CLOSE"));
  } else if (line.startsWith("x ")) {
    if (!canAcceptMotionCmd()) return;
    float dx = line.substring(2).toFloat();
    if (!isfinite(dx)) { Serial.println(F("[BLOCKED] Invalid X jog value.")); return; }
    if (isHomed) {
      float curMm = (float)stepperX.currentPosition() / STEPS_PER_MM_X;
      float targetMm = constrain(curMm + dx, 0.0f, MAX_TRAVEL_X_MM);
      if (fabsf((curMm + dx) - targetMm) > 0.01f) {
        Serial.print(F("[CLAMP] X jog clamped to safe travel [0, "));
        Serial.print(MAX_TRAVEL_X_MM, 0); Serial.println(F("] mm."));
      }
      stepperX.moveTo(lroundf(targetMm * STEPS_PER_MM_X));
      Serial.print(F("[JOG X] Target = ")); Serial.print(targetMm, 2); Serial.println(F(" mm"));
    } else {
      float clampedDx = constrain(dx, -MAX_UNHOMED_JOG_MM, MAX_UNHOMED_JOG_MM);
      if (fabsf(clampedDx - dx) > 0.01f) {
        Serial.println(F("[CLAMP] Unhomed relative X jog clamped to +/-50 mm."));
      }
      stepperX.move(lroundf(clampedDx * STEPS_PER_MM_X));
      Serial.print(F("[JOG X] ")); Serial.print(clampedDx, 2); Serial.println(F(" mm (unhomed)"));
    }
  } else if (line.startsWith("y ")) {
    if (!canAcceptMotionCmd()) return;
    float dy = line.substring(2).toFloat();
    if (!isfinite(dy)) { Serial.println(F("[BLOCKED] Invalid Y jog value.")); return; }
    if (isHomed) {
      float curMm = (float)stepperY.currentPosition() / STEPS_PER_MM_Y;
      float targetMm = constrain(curMm + dy, 0.0f, MAX_TRAVEL_Y_MM);
      if (fabsf((curMm + dy) - targetMm) > 0.01f) {
        Serial.print(F("[CLAMP] Y jog clamped to safe travel [0, "));
        Serial.print(MAX_TRAVEL_Y_MM, 0); Serial.println(F("] mm."));
      }
      stepperY.moveTo(lroundf(targetMm * STEPS_PER_MM_Y));
      Serial.print(F("[JOG Y] Target = ")); Serial.print(targetMm, 2); Serial.println(F(" mm"));
    } else {
      float clampedDy = constrain(dy, -MAX_UNHOMED_JOG_MM, MAX_UNHOMED_JOG_MM);
      if (fabsf(clampedDy - dy) > 0.01f) {
        Serial.println(F("[CLAMP] Unhomed relative Y jog clamped to +/-50 mm."));
      }
      stepperY.move(lroundf(clampedDy * STEPS_PER_MM_Y));
      Serial.print(F("[JOG Y] ")); Serial.print(clampedDy, 2); Serial.println(F(" mm (unhomed)"));
    }
  } else if (line.startsWith("go ")) {
    if (!canAcceptMotionCmd()) return;
    if (!isHomed) { Serial.println(F("[BLOCKED] Run 'home' first before absolute 'go X Y'.")); return; }
    int sp = line.indexOf(' ', 3);
    if (sp < 0) { Serial.println(F("Usage: go <X_mm> <Y_mm>")); return; }
    float rawX = line.substring(3, sp).toFloat();
    float rawY = line.substring(sp + 1).toFloat();
    if (!isfinite(rawX) || !isfinite(rawY)) { Serial.println(F("[BLOCKED] Invalid go coordinates.")); return; }
    float tx = constrain(rawX, 0.0f, MAX_TRAVEL_X_MM);
    float ty = constrain(rawY, 0.0f, MAX_TRAVEL_Y_MM);
    pca.setPWM(SERVO_Z_CH, 0, PWM_Z_RAISED);
    stepperX.moveTo(lroundf(tx * STEPS_PER_MM_X));
    stepperY.moveTo(lroundf(ty * STEPS_PER_MM_Y));
    Serial.print(F("[GO] Moving to machine (")); Serial.print(tx, 2);
    Serial.print(F(", ")); Serial.print(ty, 2); Serial.println(F(") mm"));
  } else {
    Serial.print(F("[UNKNOWN] ")); Serial.println(line);
    printHelp();
  }
}

void setup() {
  pinMode(PIN_SERVO_OE, OUTPUT);   digitalWrite(PIN_SERVO_OE, HIGH);
  pinMode(PIN_STEPPER_EN, OUTPUT); digitalWrite(PIN_STEPPER_EN, HIGH);
  pinMode(PIN_X_HOME, INPUT_PULLUP);
  pinMode(PIN_Y_HOME, INPUT_PULLUP);
  pinMode(PIN_Z_HOME, INPUT_PULLUP);
  pinMode(PIN_ESTOP,  INPUT_PULLUP);

  Serial.begin(115200);
  delay(300);

  Wire.begin();
  pca.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);
  pca.setPWMFreq(50);

  stepperX.setMinPulseWidth(2);
  stepperY.setMinPulseWidth(2);
  stepperX.setMaxSpeed(JOG_SPEED_STEPS);
  stepperX.setAcceleration(JOG_ACCEL_STEPS);
  stepperY.setMaxSpeed(JOG_SPEED_STEPS);
  stepperY.setAcceleration(JOG_ACCEL_STEPS);

  Serial.println(F("=========================================================="));
  Serial.println(F(" CapSorter Stage 3: Stepper Polarity, Homing & Offset Tool"));
  Serial.println(F(" 1. Center gantry by hand, type 'en', then 'x -10' & 'y -10'."));
  Serial.println(F("    Confirm BOTH axes move TOWARD their home switches!"));
  Serial.println(F("    If an axis moves AWAY, power off and reverse 1 coil pair."));
  Serial.println(F(" 2. Once polarity is verified, type 'home'."));
  Serial.println(F("=========================================================="));
  printHelp();
}

void loop() {
  if (!faulted) {
    if (isEstop()) {
      emergencyStop(F("Physical E-Stop pressed!"));
    } else if (steppersEnabled && (isXHomeOpen() || isYHomeOpen() || isZLimitOpen())) {
      emergencyStop(F("NC limit switch tripped during jog! Type 'home' or 'en' to recover."));
    } else if (steppersEnabled) {
      stepperX.run();
      stepperY.run();
    }
  }

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      handleCommand(cmdBuf);
      cmdBuf = "";
    } else {
      cmdBuf += c;
    }
  }
}
