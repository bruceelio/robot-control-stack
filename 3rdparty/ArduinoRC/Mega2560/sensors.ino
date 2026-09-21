// =========================================================
// SENSORS
// =========================================================
//
// Sensor/input functions currently owned by this module:
//
//   - Generic digital reads
//   - Generic analog reads
//   - Lift limit switches
//   - Start button
//   - Ultrasonic range sensors
//
// Physical pin assignments remain in Mega2560.ino.
//
// Reserved / currently inactive sensor pins remain defined centrally but are
// not newly enabled by this refactor:
//
//   - PIN_CURRENT_GRIPPER_RIGHT
//   - PIN_REFLECTANCE_LEFT
//   - PIN_REFLECTANCE_CENTRE
//   - PIN_REFLECTANCE_RIGHT
//   - PIN_SELECTOR_PI_ARDUINO
//   - PIN_BUMPER_FRONT_LEFT
//   - PIN_BUMPER_FRONT_RIGHT
//
// This file intentionally preserves the current working firmware behaviour.
// Semantic REFLECTANCE / ULTRASONIC / BUMPER commands can be added when those
// devices are actually commissioned.
//
// =========================================================

#if ENABLE_SENSORS

// =========================================================
// SETUP
// =========================================================

void setupSensors() {
  // Ultrasonic pins
  pinMode(PIN_ULTRASONIC_FRONT_LEFT_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_FRONT_LEFT_ECHO, INPUT);

  pinMode(PIN_ULTRASONIC_FRONT_RIGHT_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_FRONT_RIGHT_ECHO, INPUT);

  // Lift limits
  pinMode(PIN_LIMIT_LIFT_HIGH, INPUT_PULLUP);
  pinMode(PIN_LIMIT_LIFT_LOW, INPUT_PULLUP);

  // Start button
  pinMode(PIN_BUTTON_START, INPUT_PULLUP);
}

// =========================================================
// BASIC READ HELPERS
// =========================================================

int readDigitalPin(uint8_t pin) {
  return digitalRead(pin);
}

long readAnalogSource(const char *name) {
  // Current implementation supports A0..A15 or numeric strings.
  if (name[0] == 'A' || name[0] == 'a') {
    int idx = atoi(name + 1);
    return analogRead(idx);
  }

  int pin = atoi(name);
  return analogRead(pin);
}

// =========================================================
// ULTRASONIC
// =========================================================

long readRangePair(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000UL); // timeout ~30 ms
  if (duration <= 0) return -1;

  // Distance in mm (approx): duration_us * 0.1715
  long distanceMm = (long)(duration * 0.1715f);
  return distanceMm;
}

// =========================================================
// PI COMMAND HANDLING
// =========================================================
//
// Existing commands retained:
//
//   READ DI <pin>
//   READ AI <pin-or-Ax>
//   READ LIMIT lift_high
//   READ LIMIT lift_low
//   READ BUTTON start
//   READ RANGE <trigPin> <echoPin>
//
// READ QUAD remains owned by the encoder path and is deliberately not handled
// here.
//
// Returns:
//   false -> command does not belong to this module
//   true  -> sensor command was handled
//
bool handleSensorCommand(char *line) {
  if (strncmp(line, "READ ", 5) != 0) {
    return false;
  }

  char rkind[16];
  char a1[32];
  char a2[32];

  int count = sscanf(
      line,
      "READ %15s %31s %31s",
      rkind,
      a1,
      a2
  );

  if (count < 2) {
    return false;
  }

  if (strcmp(rkind, "DI") == 0) {
    replyValue("DI", readDigitalPin((uint8_t)atoi(a1)));
    return true;
  }

  if (strcmp(rkind, "AI") == 0) {
    replyValue("AI", readAnalogSource(a1));
    return true;
  }

  if (strcmp(rkind, "LIMIT") == 0) {
    if (strcmp(a1, "lift_high") == 0) {
      replyValue("LIMIT", readDigitalPin(PIN_LIMIT_LIFT_HIGH));
      return true;
    }

    if (strcmp(a1, "lift_low") == 0) {
      replyValue("LIMIT", readDigitalPin(PIN_LIMIT_LIFT_LOW));
      return true;
    }

    return false;
  }

  if (strcmp(rkind, "BUTTON") == 0) {
    if (strcmp(a1, "start") == 0) {
      bool pressed = (digitalRead(PIN_BUTTON_START) == LOW);
      replyValue("BUTTON", pressed);
      return true;
    }

    return false;
  }

  if (strcmp(rkind, "RANGE") == 0 && count >= 3) {
    replyValue(
        "RANGE",
        readRangePair(
            (uint8_t)atoi(a1),
            (uint8_t)atoi(a2)
        )
    );
    return true;
  }

  return false;
}

#endif  // ENABLE_SENSORS
