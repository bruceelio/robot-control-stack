// 3rdparty/ArduinoRC/Mega2560/encoders.ino
//
#if ENABLE_ENCODERS

#include <util/atomic.h>

// =========================================================
// ENCODERS
// =========================================================
//
// This file owns encoder acquisition, counting, snapshots,
// diagnostics and the semantic ENCODER serial command.
//
// Physical pin assignments remain in Mega2560.ino.
//
// Current implementation:
//   - drive_front_left  : Mega pins 22 / 24 (PORTA PA0 / PA2)
//   - drive_front_right : Mega pins 26 / 28 (PORTA PA4 / PA6)
//   - x4 quadrature decoding
//   - Timer1 Compare-B sampling at 10 kHz
//   - one PINA read captures all four drive-encoder signals
//
// Future encoder groups can be added below under their own
// ENABLE_* compile-time switches.

// =========================================================
// SHARED ENCODER DEFINITIONS
// =========================================================

static const uint16_t ENC_FLAG_NONE            = 0x0000;
static const uint16_t ENC_FLAG_NOT_INITIALIZED = 0x0001;

struct EncoderState {
  volatile int32_t count;
  volatile uint32_t last_edge_us;
  volatile uint16_t invalid_transition_count;
  volatile uint8_t previous_ab;
  volatile bool initialized;
};

struct EncoderSnapshot {
  int32_t count;
  uint16_t invalid_transition_count;
  bool initialized;
  uint32_t timestamp_ms;
};

// Previous/current AB transition table for x4 decoding.
// Index = (previous_ab << 2) | current_ab
static const int8_t QUAD_TABLE[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

// =========================================================
// DRIVE MOTOR ENCODERS
// =========================================================

#if ENABLE_DRIVE_ENCODERS

EncoderState encoderDriveFrontLeft  = {0, 0, 0, 0, false};
EncoderState encoderDriveFrontRight = {0, 0, 0, 0, false};

// Timer1 runs from the 16 MHz CPU clock with prescaler 8:
//   16 MHz / 8 = 2 MHz timer clock
//   2 MHz / 10 kHz = 200 timer ticks per encoder sample
static const uint16_t DRIVE_ENCODER_SAMPLE_HZ = 10000;
static const uint16_t DRIVE_ENCODER_TIMER_TICKS = 200;

inline uint8_t readDriveFrontLeftABFromPortA(uint8_t portA) {
  const uint8_t a = (portA >> PA0) & 0x01;
  const uint8_t b = (portA >> PA2) & 0x01;
  return (a << 1) | b;
}

inline uint8_t readDriveFrontRightABFromPortA(uint8_t portA) {
  const uint8_t a = (portA >> PA4) & 0x01;
  const uint8_t b = (portA >> PA6) & 0x01;
  return (a << 1) | b;
}

inline void updateEncoderFromAB(
    volatile EncoderState &state,
    uint8_t currentAB) {

  const uint8_t previousAB = state.previous_ab;

  if (currentAB == previousAB) {
    return;
  }

  const uint8_t changed = previousAB ^ currentAB;

  // Both bits changed between samples: an intermediate state was missed
  // or the signal was noisy. Record it diagnostically but do not alter count.
  if (changed == 0b11) {
    state.invalid_transition_count++;
    state.previous_ab = currentAB;
    return;
  }

  const uint8_t transition = (previousAB << 2) | currentAB;
  const int8_t delta = QUAD_TABLE[transition];

  state.count += delta;
  state.last_edge_us = micros();
  state.previous_ab = currentAB;
}

void setupDriveEncoderSamplingTimer() {
  // Timer1 is reserved here for drive-encoder sampling.
  // Compare-B is used so the Servo library's Compare-A ISR is not replaced.
  // The current sketch's five servos are handled on Timer5 on the Mega.
  // Timer2 is deliberately avoided because tone() uses it for the buzzer.

  noInterrupts();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // Normal counting mode, prescaler = 8.
  TCCR1B = _BV(CS11);

  OCR1B = DRIVE_ENCODER_TIMER_TICKS;
  TIFR1 = _BV(OCF1B);      // clear any pending Compare-B flag
  TIMSK1 |= _BV(OCIE1B);   // enable Compare-B interrupt

  interrupts();
}

void setupDriveEncoders() {
  pinMode(PIN_ENC_DRIVE_FRONT_LEFT_A, INPUT_PULLUP);
  pinMode(PIN_ENC_DRIVE_FRONT_LEFT_B, INPUT_PULLUP);
  pinMode(PIN_ENC_DRIVE_FRONT_RIGHT_A, INPUT_PULLUP);
  pinMode(PIN_ENC_DRIVE_FRONT_RIGHT_B, INPUT_PULLUP);

  // Capture one coherent initial state before enabling the sampling timer.
  const uint8_t portA = PINA;
  const uint32_t nowUs = micros();

  encoderDriveFrontLeft.count = 0;
  encoderDriveFrontLeft.last_edge_us = nowUs;
  encoderDriveFrontLeft.invalid_transition_count = 0;
  encoderDriveFrontLeft.previous_ab = readDriveFrontLeftABFromPortA(portA);
  encoderDriveFrontLeft.initialized = true;

  encoderDriveFrontRight.count = 0;
  encoderDriveFrontRight.last_edge_us = nowUs;
  encoderDriveFrontRight.invalid_transition_count = 0;
  encoderDriveFrontRight.previous_ab = readDriveFrontRightABFromPortA(portA);
  encoderDriveFrontRight.initialized = true;

  setupDriveEncoderSamplingTimer();
}

ISR(TIMER1_COMPB_vect) {
  // Schedule the next sample. If this ISR was delayed past its intended next
  // compare point, restart the interval from the current timer count.
  uint16_t nextCompare = OCR1B + DRIVE_ENCODER_TIMER_TICKS;
  const uint16_t now = TCNT1;

  if ((int16_t)(nextCompare - now) <= 0) {
    nextCompare = now + DRIVE_ENCODER_TIMER_TICKS;
  }

  OCR1B = nextCompare;

  // One port read captures all four drive encoder channels together.
  const uint8_t portA = PINA;

  updateEncoderFromAB(
      encoderDriveFrontLeft,
      readDriveFrontLeftABFromPortA(portA));

  updateEncoderFromAB(
      encoderDriveFrontRight,
      readDriveFrontRightABFromPortA(portA));
}

#endif  // ENABLE_DRIVE_ENCODERS

// =========================================================
// FUTURE DEADWHEEL ENCODERS
// =========================================================

#if ENABLE_DEADWHEEL_ENCODERS
#error "ENABLE_DEADWHEEL_ENCODERS is not implemented yet"
#endif

// =========================================================
// FUTURE SHOOTER ENCODER
// =========================================================

#if ENABLE_SHOOTER_ENCODER
#error "ENABLE_SHOOTER_ENCODER is not implemented yet"
#endif

// =========================================================
// ENCODER MODULE SETUP / SERVICE
// =========================================================

void setupEncoders() {
#if ENABLE_DRIVE_ENCODERS
  setupDriveEncoders();
#endif
}

void serviceEncoders() {
  // Drive encoder acquisition currently occurs entirely in the timer ISR.
  // Reserved for future low-rate diagnostics or derived values.
}

// =========================================================
// ENCODER SNAPSHOTS
// =========================================================

EncoderSnapshot snapshotEncoder(const volatile EncoderState &state) {
  EncoderSnapshot snapshot;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    snapshot.count = state.count;
    snapshot.invalid_transition_count = state.invalid_transition_count;
    snapshot.initialized = state.initialized;
  }

  // Public timestamp is the time of this returned snapshot, not last edge time.
  snapshot.timestamp_ms = millis();

  return snapshot;
}

volatile EncoderState *resolveEncoderState(const char *name) {
#if ENABLE_DRIVE_ENCODERS
  if (strcmp(name, "drive_front_left") == 0) {
    return &encoderDriveFrontLeft;
  }

  if (strcmp(name, "drive_front_right") == 0) {
    return &encoderDriveFrontRight;
  }
#endif

  return nullptr;
}

void replyEncoder(
    const char *name,
    const EncoderSnapshot &snapshot) {

  const uint16_t validFlags =
      snapshot.initialized ? ENC_FLAG_NONE : ENC_FLAG_NOT_INITIALIZED;

  const uint8_t valid = snapshot.initialized ? 1 : 0;

  PI_SERIAL.print(F("OK ENCODER "));
  PI_SERIAL.print(name);
  PI_SERIAL.print(F(" count="));
  PI_SERIAL.print(snapshot.count);
  PI_SERIAL.print(F(" timestamp_ms="));
  PI_SERIAL.print(snapshot.timestamp_ms);
  PI_SERIAL.print(F(" valid="));
  PI_SERIAL.print(valid);
  PI_SERIAL.print(F(" valid_flags="));
  PI_SERIAL.println(validFlags);
}

// =========================================================
// ENCODER PI COMMANDS
// =========================================================

bool handleEncoderCommand(char *line) {
  // Only claim lines belonging to the semantic ENCODER command family.
  if (strncmp(line, "ENCODER", 7) != 0 ||
      (line[7] != '\0' && line[7] != ' ')) {
    return false;
  }

  char encoderName[32];
  char operation[16];

  if (sscanf(
          line,
          "ENCODER %31s %15s",
          encoderName,
          operation) != 2) {
    PI_SERIAL.println(F("ERR ENCODER syntax"));
    return true;
  }

  if (strcmp(operation, "READ") != 0) {
    PI_SERIAL.print(F("ERR ENCODER unknown_operation="));
    PI_SERIAL.println(operation);
    return true;
  }

  volatile EncoderState *state = resolveEncoderState(encoderName);

  if (state == nullptr) {
    PI_SERIAL.print(F("ERR ENCODER unknown_name="));
    PI_SERIAL.println(encoderName);
    return true;
  }

  const EncoderSnapshot snapshot = snapshotEncoder(*state);
  replyEncoder(encoderName, snapshot);
  return true;
}

// =========================================================
// LOW-LEVEL QUADRATURE WIRING DIAGNOSTIC
// =========================================================

long readQuadPair(uint8_t pinA, uint8_t pinB) {
  const int a = digitalRead(pinA) ? 1 : 0;
  const int b = digitalRead(pinB) ? 1 : 0;
  return (a << 1) | b;
}

#endif  // ENABLE_ENCODERS
