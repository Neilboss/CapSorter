// =========================================================================
// Stage 2 Bench Test: Arduino Mega 2560 — PCA9685 Z-Axis & Gripper Calibration
// Purpose:  Safely find the exact 12-bit PWM ticks for your MG90S servos
//           WITHOUT energizing XY steppers.
// Wiring:   PCA9685 I2C 0x40 -> SDA=Pin 20, SCL=Pin 21, OE=Pin 9
//           CH0 = Z-Axis Servo, CH1 = Parallel Gripper Servo
//           E-Stop Pole 2 NC Aux Contact -> Pin 25
// Usage:    Upload, open Serial Monitor @ 115200 baud (Newline ending).
//           Type commands:
//             z 180      -> Move Z servo to 180 ticks
//             z+ / z-    -> Nudge Z servo by +5 / -5 ticks
//             g 210      -> Move Gripper servo to 210 ticks
//             g+ / g-    -> Nudge Gripper servo by +5 / -5 ticks
//             mark raised | pick | bin | open | close -> Save current tick
//             print      -> Print ready-to-paste #defines for mega_motion.ino
//             off / on   -> Disable / Enable PCA9685 outputs (OE pin)
// =========================================================================
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PIN_STEPPER_EN     8   // Locked HIGH (XY steppers disabled!)
#define PIN_SERVO_OE       9   // Active LOW PCA9685 Output Enable
#define PIN_ESTOP         25   // NC E-Stop Aux Contact (HIGH = active)

#define SERVO_Z_CH         0
#define SERVO_GRIPPER_CH   1

// Hard safety clamps to prevent mechanical servo horn binding (50 Hz: ~100..520 ticks)
#define TICK_MIN         100
#define TICK_MAX         520

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

int curZ = 180;
int curG = 210;
int savedRaised = 180;
int savedPick   = 410;
int savedBin    = 320;
int savedOpen   = 210;
int savedClose  = 385;
bool outputsEnabled = false;
String cmdBuf = "";

bool isEstopActive() {
  return digitalRead(PIN_ESTOP) == HIGH;
}

void setOutputsEnabled(bool enable) {
  if (enable && isEstopActive()) {
    Serial.println(F("[BLOCKED] Release physical E-Stop before enabling servos!"));
    outputsEnabled = false;
    digitalWrite(PIN_SERVO_OE, HIGH);
    return;
  }
  outputsEnabled = enable;
  digitalWrite(PIN_SERVO_OE, enable ? LOW : HIGH);
  Serial.println(enable ? F("[OE] PCA9685 outputs ENABLED.") : F("[OE] PCA9685 outputs DISABLED."));
}

void applyZ(int ticks) {
  curZ = constrain(ticks, TICK_MIN, TICK_MAX);
  pca.setPWM(SERVO_Z_CH, 0, (uint16_t)curZ);
  Serial.print(F("[Z_CH0] -> "));
  Serial.print(curZ);
  Serial.println(F(" ticks"));
}

void applyG(int ticks) {
  curG = constrain(ticks, TICK_MIN, TICK_MAX);
  pca.setPWM(SERVO_GRIPPER_CH, 0, (uint16_t)curG);
  Serial.print(F("[GRIP_CH1] -> "));
  Serial.print(curG);
  Serial.println(F(" ticks"));
}

void printDefines() {
  Serial.println(F("\n--- Copy these into firmware/mega_motion/mega_motion.ino ---"));
  Serial.print(F("#define PWM_Z_RAISED      ")); Serial.println(savedRaised);
  Serial.print(F("#define PWM_Z_LOWER_PICK  ")); Serial.println(savedPick);
  Serial.print(F("#define PWM_Z_LOWER_BIN   ")); Serial.println(savedBin);
  Serial.print(F("#define PWM_GRIP_OPEN     ")); Serial.println(savedOpen);
  Serial.print(F("#define PWM_GRIP_CLOSE    ")); Serial.println(savedClose);
  Serial.println(F("------------------------------------------------------------\n"));
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  z <100..520>   Set Z servo ticks      | z+ / z-  Nudge +/-5 ticks"));
  Serial.println(F("  g <100..520>   Set Gripper ticks      | g+ / g-  Nudge +/-5 ticks"));
  Serial.println(F("  mark raised | mark pick | mark bin | mark open | mark close"));
  Serial.println(F("  print          Print #defines         | on / off Toggle servo OE"));
}

void handleCommand(String line) {
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) return;

  if (line == "help" || line == "?") {
    printHelp();
  } else if (line == "off") {
    setOutputsEnabled(false);
  } else if (line == "on") {
    pca.setPWM(SERVO_Z_CH, 0, (uint16_t)curZ);
    pca.setPWM(SERVO_GRIPPER_CH, 0, (uint16_t)curG);
    setOutputsEnabled(true);
  } else if (line == "z+") {
    applyZ(curZ + 5);
  } else if (line == "z-") {
    applyZ(curZ - 5);
  } else if (line == "g+") {
    applyG(curG + 5);
  } else if (line == "g-") {
    applyG(curG - 5);
  } else if (line.startsWith("z ")) {
    applyZ(line.substring(2).toInt());
  } else if (line.startsWith("g ")) {
    applyG(line.substring(2).toInt());
  } else if (line == "mark raised") {
    savedRaised = curZ; Serial.print(F("[SAVED] PWM_Z_RAISED = ")); Serial.println(savedRaised);
  } else if (line == "mark pick") {
    savedPick = curZ; Serial.print(F("[SAVED] PWM_Z_LOWER_PICK = ")); Serial.println(savedPick);
  } else if (line == "mark bin") {
    savedBin = curZ; Serial.print(F("[SAVED] PWM_Z_LOWER_BIN = ")); Serial.println(savedBin);
  } else if (line == "mark open") {
    savedOpen = curG; Serial.print(F("[SAVED] PWM_GRIP_OPEN = ")); Serial.println(savedOpen);
  } else if (line == "mark close") {
    savedClose = curG; Serial.print(F("[SAVED] PWM_GRIP_CLOSE = ")); Serial.println(savedClose);
  } else if (line == "print") {
    printDefines();
  } else {
    Serial.print(F("[UNKNOWN] ")); Serial.println(line);
    printHelp();
  }
}

void setup() {
  pinMode(PIN_STEPPER_EN, OUTPUT);
  digitalWrite(PIN_STEPPER_EN, HIGH); // Keep XY steppers disabled!
  pinMode(PIN_SERVO_OE, OUTPUT);
  digitalWrite(PIN_SERVO_OE, HIGH);
  pinMode(PIN_ESTOP, INPUT_PULLUP);

  Serial.begin(115200);
  delay(300);

  Wire.begin();
  pca.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);
  pca.setPWMFreq(50);

  // Pre-latch safe initial positions before enabling OE
  pca.setPWM(SERVO_Z_CH, 0, (uint16_t)curZ);
  pca.setPWM(SERVO_GRIPPER_CH, 0, (uint16_t)curG);

  Serial.println(F("=========================================================="));
  Serial.println(F(" CapSorter Stage 2: PCA9685 Servo & Gripper Calibrator"));
  Serial.println(F(" TIP: Remove servo horns on first power-up or start near"));
  Serial.println(F("      mid-stroke, then type 'on' to enable PCA9685 OE."));
  Serial.println(F("=========================================================="));
  printHelp();
}

void loop() {
  // Continuously enforce E-stop safety
  if (outputsEnabled && isEstopActive()) {
    digitalWrite(PIN_SERVO_OE, HIGH);
    outputsEnabled = false;
    Serial.println(F("[E-STOP] Physical E-Stop pressed! PCA9685 outputs DISABLED."));
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
