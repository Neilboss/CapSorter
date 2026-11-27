// =========================================================================
// Stage 1 Bench Test: Arduino Mega 2560 — Limit Switches & E-Stop Verifier
// Purpose:  Verify all 3 Normally-Closed (NC) limit switches and E-Stop
//           auxiliary contact BEFORE energizing any steppers or servos.
// Safety:   PIN_STEPPER_EN and PIN_SERVO_OE are locked HIGH (disabled).
// Usage:    Upload to Arduino Mega 2560, open Serial Monitor @ 115200 baud.
//           Click each microswitch and press/release E-stop one by one.
// =========================================================================

#define PIN_STEPPER_EN  8   // Locked HIGH (disabled) throughout this test
#define PIN_SERVO_OE    9   // Locked HIGH (disabled) throughout this test
#define PIN_X_HOME     22   // X-Axis NC Limit Switch to GND
#define PIN_Y_HOME     23   // Y-Axis NC Limit Switch to GND
#define PIN_Z_HOME     24   // Z-Axis NC Overtravel/Home Switch to GND
#define PIN_ESTOP      25   // Latching E-Stop Pole 2 NC Aux Contact to GND

bool prevX = false;
bool prevY = false;
bool prevZ = false;
bool prevE = false;
unsigned long lastTableMs = 0;

void printPinStatus(const char *label, uint8_t pin, bool isOpen) {
  Serial.print(label);
  Serial.print(F(" (Pin "));
  Serial.print(pin);
  Serial.print(F("): "));
  if (isOpen) {
    Serial.print(F("HIGH -> OPEN [TRIGGERED / WIRE BROKEN]"));
  } else {
    Serial.print(F("LOW  -> CLOSED [NORMAL / SAFE]"));
  }
  Serial.println();
}

void setup() {
  // Guarantee steppers and servos stay completely disabled
  pinMode(PIN_STEPPER_EN, OUTPUT);
  digitalWrite(PIN_STEPPER_EN, HIGH);
  pinMode(PIN_SERVO_OE, OUTPUT);
  digitalWrite(PIN_SERVO_OE, HIGH);

  pinMode(PIN_X_HOME, INPUT_PULLUP);
  pinMode(PIN_Y_HOME, INPUT_PULLUP);
  pinMode(PIN_Z_HOME, INPUT_PULLUP);
  pinMode(PIN_ESTOP,  INPUT_PULLUP);

  Serial.begin(115200);
  delay(300);
  Serial.println(F("=========================================================="));
  Serial.println(F(" CapSorter Stage 1: NC Limit Switch & E-Stop Bench Test"));
  Serial.println(F(" Steppers & Servos are DISABLED (safe zero-motion test)."));
  Serial.println(F(" Expected at rest (unpressed): ALL FOUR must read LOW."));
  Serial.println(F(" Expected when pressed/unplugged: pin flips to HIGH."));
  Serial.println(F("=========================================================="));

  prevX = (digitalRead(PIN_X_HOME) == HIGH);
  prevY = (digitalRead(PIN_Y_HOME) == HIGH);
  prevZ = (digitalRead(PIN_Z_HOME) == HIGH);
  prevE = (digitalRead(PIN_ESTOP)  == HIGH);
}

void loop() {
  bool curX = (digitalRead(PIN_X_HOME) == HIGH);
  bool curY = (digitalRead(PIN_Y_HOME) == HIGH);
  bool curZ = (digitalRead(PIN_Z_HOME) == HIGH);
  bool curE = (digitalRead(PIN_ESTOP)  == HIGH);

  if (curX != prevX) {
    Serial.print(F("[CHANGE] "));
    printPinStatus("X_HOME", PIN_X_HOME, curX);
    prevX = curX;
  }
  if (curY != prevY) {
    Serial.print(F("[CHANGE] "));
    printPinStatus("Y_HOME", PIN_Y_HOME, curY);
    prevY = curY;
  }
  if (curZ != prevZ) {
    Serial.print(F("[CHANGE] "));
    printPinStatus("Z_HOME", PIN_Z_HOME, curZ);
    prevZ = curZ;
  }
  if (curE != prevE) {
    Serial.print(F("[CHANGE] "));
    printPinStatus("E_STOP", PIN_ESTOP, curE);
    prevE = curE;
  }

  unsigned long now = millis();
  if (now - lastTableMs >= 1000UL) {
    lastTableMs = now;
    Serial.println(F("--- Live Snapshot (1 Hz) ---"));
    printPinStatus("X_HOME", PIN_X_HOME, curX);
    printPinStatus("Y_HOME", PIN_Y_HOME, curY);
    printPinStatus("Z_HOME", PIN_Z_HOME, curZ);
    printPinStatus("E_STOP", PIN_ESTOP, curE);
    if (!curX && !curY && !curZ && !curE) {
      Serial.println(F("[PASS CHECK] All 4 NC circuits CLOSED (ready for Stage 2)."));
    } else {
      Serial.println(F("[ATTENTION] One or more NC inputs are OPEN; check wiring or release switch."));
    }
  }
  delay(20);
}
