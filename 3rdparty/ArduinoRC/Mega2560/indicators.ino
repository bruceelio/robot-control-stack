// =========================================================
// INDICATORS
// =========================================================
//
// Indicator outputs owned by this module:
//
//   - Piezo buzzer
//   - Lisiparoi PWM LED output
//
// Physical pin assignments remain in Mega2560.ino.
//
// Battery measurement, thresholds and battery state belong to battery.ino.
// This module converts the reported battery state into the appropriate
// audible warning pattern.
//
// =========================================================

#if ENABLE_INDICATORS

// =========================================================
// SETUP
// =========================================================

void setupIndicators() {
  pinMode(PIN_PIEZO_BUZZER, OUTPUT);
  noTone(PIN_PIEZO_BUZZER);

  pinMode(PIN_LED_LISIPAROI_PWM, OUTPUT);
  analogWrite(PIN_LED_LISIPAROI_PWM, 0);
}

// =========================================================
// BUZZER
// =========================================================

void setBuzzerTone(unsigned int frequencyHz) {
  tone(PIN_PIEZO_BUZZER, frequencyHz);
}

void stopBuzzer() {
  noTone(PIN_PIEZO_BUZZER);
}

// =========================================================
// BATTERY BUZZER
// =========================================================

#if ENABLE_BATTERY_MONITOR

void serviceBatteryIndicator() {
  static unsigned long lastBeepMs = 0;
  static bool beepOn = false;
  static uint8_t previousState = 255;

  uint8_t state = getBatteryState();
  unsigned long now = millis();

  if (state != previousState) {
    previousState = state;
    lastBeepMs = now;
    beepOn = false;
    stopBuzzer();
  }

  switch (state) {

    case BATTERY_STATE_SHUTDOWN:
      setBuzzerTone(2200);
      return;

    case BATTERY_STATE_SEVERE:
      setBuzzerTone(2200);
      return;

    case BATTERY_STATE_CRITICAL:
      if (now - lastBeepMs >= 200) {
        lastBeepMs = now;
        beepOn = !beepOn;

        if (beepOn) setBuzzerTone(2200);
        else stopBuzzer();
      }
      return;

    case BATTERY_STATE_LOW:
      if (now - lastBeepMs >= 800) {
        lastBeepMs = now;
        beepOn = !beepOn;

        if (beepOn) setBuzzerTone(1800);
        else stopBuzzer();
      }
      return;

    case BATTERY_STATE_NORMAL:
    default:
      stopBuzzer();
      return;
  }
}

#endif

void serviceIndicators() {
#if ENABLE_BATTERY_MONITOR
  serviceBatteryIndicator();
#endif
}


// =========================================================
// LISIPAROI LED
// =========================================================

void setLisiparoiBrightness(float brightness) {
  brightness = constrain(brightness, 0.0f, 1.0f);

  int pwmValue = (int)(brightness * 255.0f);
  analogWrite(PIN_LED_LISIPAROI_PWM, pwmValue);
}

// =========================================================
// PI COMMAND HANDLING
// =========================================================
//
// Supported command:
//
//   LED lisiparoi WRITE brightness=<value>
//
// Returns:
//   false -> command does not belong to this module
//   true  -> indicator command was handled, including errors
//
bool handleIndicatorCommand(char *line) {
  if (strncmp(line, "LED ", 4) != 0) {
    return false;
  }

  char ledName[32];
  char ledBrightnessText[32];

  if (sscanf(
          line,
          "LED %31s WRITE brightness=%31s",
          ledName,
          ledBrightnessText
      ) == 2) {

    if (strcmp(ledName, "lisiparoi") == 0) {
      float brightness = constrain(atof(ledBrightnessText), 0.0f, 1.0f);

      setLisiparoiBrightness(brightness);

      PI_SERIAL.print("OK LED lisiparoi brightness=");
      PI_SERIAL.println(brightness, 4);
      return true;
    }

    PI_SERIAL.print("ERR LED ");
    PI_SERIAL.println(ledName);
    return true;
  }

  PI_SERIAL.println("ERR LED syntax");
  return true;
}

#endif  // ENABLE_INDICATORS
