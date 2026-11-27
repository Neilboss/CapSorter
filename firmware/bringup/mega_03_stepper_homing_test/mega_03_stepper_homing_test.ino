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
#define STEPS_PER_MM_X     40.0f
#define STEPS_PER_MM_Y     40.0f
#define JOG_SPEED_STEPS  1000.0f  // Slow, safe 25 mm/s jog speed
#define JOG_ACCEL_STEPS  2500.0f
#define HOMING_SPEED      800.0f  // 20 mm/s homing seek speed
#define MAX_TRAVEL_X_MM   370.0f
#define MAX_TRAVEL_Y_MM   300.0f

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

  Serial.println(F("[HOME] Seeking X & Y home switches at 20 mm/s..."));
  unsigned long t0 = millis();
  while (!isXHomeOpen() || !isYHomeOpen()) {
    if (isEstop() || isZLimitOpen()) { emergencyStop(F("E-Stop/Z fault during seek")); return; }
    if (millis() - t0 > 30000UL) { emergencyStop(F("Homing seek timeout (>30s)")); return; }
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

void handleCommand(String line) {
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) return;

  if (line == "help" || line == "?") {
    printHelp();
  } else if (line == "stop") {
    stepperX.moveTo(stepperX.currentPosition());
    stepperY.moveTo(stepperY.currentPosition());
    Serial.println(F("[STOP] Motion target cleared."));
  } else if (line == "dis") {
    digitalWrite(PIN_STEPPER_EN, HIGH);
    steppersEnabled = false;
    isHomed = false;
    Serial.println(F("[EN] Steppers DISABLED (free to move by hand)."));
  } else if (line == "en") {
    if (isEstop()) { Serial.println(F("[BLOCKED] Release E-Stop first.")); return; }
    faulted = false;
    pca.setPWM(SERVO_Z_CH, 0, PWM_Z_RAISED);
    pca.setPWM(SERVO_GRIPPER_CH, 0, PWM_GRIP_OPEN);
    digitalWrite(PIN_SERVO_OE, LOW);
    digitalWrite(PIN_STEPPER_EN, LOW);
    steppersEnabled = true;
    Serial.println(F("[EN] Steppers ENABLED (Z raised, gripper open)."));
  } else if (line == "home") {
    runHomeSequence();
  } else if (line == "pos") {
    printPosition();
  } else if (line == "zup") {
    pca.setPWM(SERVO_Z_CH, 0, PWM_Z_RAISED); Serial.println(F("[Z] RAISED"));
  } else if (line == "zpick") {
    pca.setPWM(SERVO_Z_CH, 0, PWM_Z_LOWER_PICK); Serial.println(F("[Z] LOWER_PICK"));
  } else if (line == "zbin") {
    pca.setPWM(SERVO_Z_CH, 0, PWM_Z_LOWER_BIN); Serial.println(F("[Z] LOWER_BIN"));
  } else if (line == "open") {
    pca.setPWM(SERVO_GRIPPER_CH, 0, PWM_GRIP_OPEN); Serial.println(F("[GRIP] OPEN"));
  } else if (line == "close") {
    pca.setPWM(SERVO_GRIPPER_CH, 0, PWM_GRIP_CLOSE); Serial.println(F("[GRIP] CLOSE"));
  } else if (line.startsWith("x ")) {
    if (!steppersEnabled) { Serial.println(F("[WARN] Type 'en' first.")); return; }
    float dx = line.substring(2).toFloat();
    stepperX.move(lroundf(dx * STEPS_PER_MM_X));
    Serial.print(F("[JOG X] ")); Serial.print(dx, 2); Serial.println(F(" mm"));
  } else if (line.startsWith("y ")) {
    if (!steppersEnabled) { Serial.println(F("[WARN] Type 'en' first.")); return; }
    float dy = line.substring(2).toFloat();
    stepperY.move(lroundf(dy * STEPS_PER_MM_Y));
    Serial.print(F("[JOG Y] ")); Serial.print(dy, 2); Serial.println(F(" mm"));
  } else if (line.startsWith("go ")) {
    if (!isHomed) { Serial.println(F("[BLOCKED] Run 'home' first before absolute 'go X Y'.")); return; }
    int sp = line.indexOf(' ', 3);
    if (sp < 0) { Serial.println(F("Usage: go <X_mm> <Y_mm>")); return; }
    float tx = constrain(line.substring(3, sp).toFloat(), 0.0f, MAX_TRAVEL_X_MM);
    float ty = constrain(line.substring(sp + 1).toFloat(), 0.0f, MAX_TRAVEL_Y_MM);
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
