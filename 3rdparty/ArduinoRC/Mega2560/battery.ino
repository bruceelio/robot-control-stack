// =========================================================
// BATTERY MONITOR
// =========================================================
//
// Owns:
//   - Battery ADC conversion
//   - Battery voltage thresholds
//   - Current battery state
//   - Low-voltage shutdown state
//   - Battery Pi command handling
//
// Physical pin assignment remains in Mega2560.ino:
//
//   PIN_VOLTAGE_BATTERY
//
// Audible warning patterns do NOT belong here.
// indicators.ino reads the battery state and operates the buzzer.
//
// =========================================================

#if ENABLE_BATTERY_MONITOR

// =========================================================
// BATTERY CONFIGURATION
// =========================================================

// Battery voltage divider calibration.
static const float BATTERY_DIVIDER_RATIO       = 5.0f;

// FTC-style operating thresholds. These are intentionally above the
// absolute NiMH minimum because robot performance degrades as the pack
// reaches the lower part of its discharge curve.
static const float BATTERY_LOW_WARN_V          = 12.5f;
static const float BATTERY_CRITICAL_WARN_V     = 12.2f;
static const float BATTERY_SEVERE_WARN_V       = 11.5f;
static const float BATTERY_SHUTDOWN_NOW_V      = 11.0f;

// Require a low voltage to persist before escalating state. This prevents
// normal drivetrain acceleration/current spikes from causing nuisance alarms.
static const unsigned long BATTERY_LOW_CONFIRM_MS      = 3000UL;
static const unsigned long BATTERY_CRITICAL_CONFIRM_MS = 1500UL;
static const unsigned long BATTERY_SEVERE_CONFIRM_MS   = 750UL;
static const unsigned long BATTERY_SHUTDOWN_CONFIRM_MS = 1000UL;

// =========================================================
// BATTERY STATE
// =========================================================

static const uint8_t BATTERY_STATE_NORMAL      = 0;
static const uint8_t BATTERY_STATE_LOW         = 1;
static const uint8_t BATTERY_STATE_CRITICAL    = 2;
static const uint8_t BATTERY_STATE_SEVERE      = 3;
static const uint8_t BATTERY_STATE_SHUTDOWN    = 4;

float batteryVoltage = 0.0f;
uint8_t batteryState = BATTERY_STATE_NORMAL;

uint8_t batteryCandidateState = BATTERY_STATE_NORMAL;
unsigned long batteryCandidateSinceMs = 0;
bool batteryShutdownLatched = false;

// =========================================================
// BATTERY READING
// =========================================================

float readBatteryVoltage() {
  int raw = analogRead(PIN_VOLTAGE_BATTERY);
  float sensorVolts = raw * (5.0f / 1023.0f);
  return sensorVolts * BATTERY_DIVIDER_RATIO;
}

uint8_t determineBatteryState(float voltage) {
  if (voltage <= BATTERY_SHUTDOWN_NOW_V) {
    return BATTERY_STATE_SHUTDOWN;
  }

  if (voltage <= BATTERY_SEVERE_WARN_V) {
    return BATTERY_STATE_SEVERE;
  }

  if (voltage <= BATTERY_CRITICAL_WARN_V) {
    return BATTERY_STATE_CRITICAL;
  }

  if (voltage <= BATTERY_LOW_WARN_V) {
    return BATTERY_STATE_LOW;
  }

  return BATTERY_STATE_NORMAL;
}

unsigned long batteryConfirmTimeMs(uint8_t state) {
  switch (state) {
    case BATTERY_STATE_LOW:
      return BATTERY_LOW_CONFIRM_MS;

    case BATTERY_STATE_CRITICAL:
      return BATTERY_CRITICAL_CONFIRM_MS;

    case BATTERY_STATE_SEVERE:
      return BATTERY_SEVERE_CONFIRM_MS;

    case BATTERY_STATE_SHUTDOWN:
      return BATTERY_SHUTDOWN_CONFIRM_MS;

    case BATTERY_STATE_NORMAL:
    default:
      return 0UL;
  }
}

// =========================================================
// PUBLIC BATTERY STATE
// =========================================================

float getBatteryVoltage() {
  return batteryVoltage;
}

uint8_t getBatteryState() {
  return batteryState;
}

bool batteryShutdownActive() {
  return batteryShutdownLatched;
}

// =========================================================
// SETUP / SERVICE
// =========================================================

void setupBatteryMonitor() {
  batteryVoltage = readBatteryVoltage();
  batteryState = BATTERY_STATE_NORMAL;
  batteryCandidateState = BATTERY_STATE_NORMAL;
  batteryCandidateSinceMs = millis();
  batteryShutdownLatched = false;
}

void serviceBatteryMonitor() {
  batteryVoltage = readBatteryVoltage();

  // Once shutdown has been confirmed, keep it latched until the Mega is
  // power-cycled/reset. This prevents a stop -> voltage recovery -> restart loop.
  if (batteryShutdownLatched) {
    batteryState = BATTERY_STATE_SHUTDOWN;
    stopDrive();
    writeShooterMotor(0.0f);
    writeCollectorMotor(0.0f);
    return;
  }

  uint8_t measuredState = determineBatteryState(batteryVoltage);
  unsigned long now = millis();

  if (measuredState < batteryState) {
    // Voltage has recovered into a healthier band. Recovery is immediate;
    // any future worsening must again remain low for the confirmation period.
    batteryState = measuredState;
    batteryCandidateState = measuredState;
    batteryCandidateSinceMs = now;
  } else if (measuredState > batteryState) {
    // Voltage has entered a worse band. Confirm it is sustained before
    // escalating the alarm state.
    if (measuredState != batteryCandidateState) {
      batteryCandidateState = measuredState;
      batteryCandidateSinceMs = now;
    }

    if (now - batteryCandidateSinceMs >= batteryConfirmTimeMs(measuredState)) {
      batteryState = measuredState;

      if (batteryState == BATTERY_STATE_SHUTDOWN) {
        batteryShutdownLatched = true;
      }
    }
  } else {
    // Stable in the current band.
    batteryCandidateState = measuredState;
    batteryCandidateSinceMs = now;
  }

  // Battery shutdown is a safety action, not an indicator action.
  // Keep all motor outputs stopped while the shutdown state is active.
  if (batteryShutdownLatched) {
    stopDrive();
    writeShooterMotor(0.0f);
    writeCollectorMotor(0.0f);
  }
}

// =========================================================
// PI COMMAND HANDLING
// =========================================================
//
// Current semantic command:
//   VOLTAGE battery READ
//
// Existing migration/legacy command retained:
//   READ BATTERY voltage
//
// Note:
//   READ BATTERY voltage intentionally retains the current firmware's
//   legacy raw-ADC reply behaviour. It can be retired separately later.
//
bool handleBatteryCommand(char *line) {

  // VOLTAGE battery READ
  if (strncmp(line, "VOLTAGE ", 8) == 0) {
    char voltageName[32];
    char voltageCmd[32];

    if (sscanf(
            line,
            "VOLTAGE %31s %31s",
            voltageName,
            voltageCmd
        ) == 2) {

      if (strcmp(voltageName, "battery") == 0 &&
          strcmp(voltageCmd, "READ") == 0) {

        float volts = readBatteryVoltage();

        PI_SERIAL.print("OK VOLTAGE battery volts=");
        PI_SERIAL.println(volts, 2);
        return true;
      }

      PI_SERIAL.print("ERR VOLTAGE ");
      PI_SERIAL.print(voltageName);
      PI_SERIAL.print(" ");
      PI_SERIAL.println(voltageCmd);
      return true;
    }

    PI_SERIAL.println("ERR VOLTAGE syntax");
    return true;
  }

  // Existing backward-compatible READ BATTERY voltage command.
  if (strncmp(line, "READ BATTERY ", 13) == 0) {
    char property[32];

    if (sscanf(line, "READ BATTERY %31s", property) == 1) {
      if (strcmp(property, "voltage") == 0) {
        replyValue("BATTERY", analogRead(PIN_VOLTAGE_BATTERY));
        return true;
      }

      PI_SERIAL.print("ERR BATTERY ");
      PI_SERIAL.println(property);
      return true;
    }

    PI_SERIAL.println("ERR BATTERY syntax");
    return true;
  }

  return false;
}

#endif  // ENABLE_BATTERY_MONITOR
