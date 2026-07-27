// BobBot_EncoderTemplate.ino
//
// Purpose:
//   Reusable Arduino Mega encoder template for the revised BobBot semantic IO model.
//   This sketch implements ONLY encoder IO, but it is structured so the same
//   algorithm can be reused for shooter, drive, and deadwheel encoders.
//
// Protocol, 115200 baud:
//   HELLO
//   ENCODER <name> READ
//   ENCODER <name> ZERO
//   ENCODER <name> RESET
//   ENCODER * READ
//   ENCODER * ZERO
//   ENCODER * RESET
//
// Example:
//   ENCODER shooter READ
//
// Response:
//   OK ENCODER shooter count=<count> timestamp_ms=<ms> valid=<0|1> valid_flags=<flags>
//
// Notes:
//   - This is a polling implementation. It works best when loop() stays fast.
//   - Arduino Mega pins 22..29 and 35/37 are not all native attachInterrupt pins.
//   - If encoder edge rate is too high, move A/B to interrupt-capable pins or
//     use a verified pin-change interrupt implementation.
//   - To add/remove encoders, edit the pin constants and encoder_devices[] table.

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

// =========================================================
// SERIAL / DEVICE
// =========================================================
#define PI_SERIAL Serial
static const char DEVICE_ID[] = "MEGA_ENCODER_TEMPLATE_1";

// =========================================================
// ARDUINO SEMANTIC DEVICE NAMES
// These strings must match the Pi protocol names exactly.
// =========================================================
static const char DEV_ENCODER_SHOOTER[] = "shooter";
static const char DEV_ENCODER_DRIVE_FRONT_LEFT[] = "drive_front_left";
static const char DEV_ENCODER_DRIVE_FRONT_RIGHT[] = "drive_front_right";
static const char DEV_ENCODER_DEADWHEEL_PARALLEL[] = "deadwheel_parallel";
static const char DEV_ENCODER_DEADWHEEL_PERPENDICULAR[] = "deadwheel_perpendicular";

// =========================================================
// PIN NAMES
// Current table/drawing values.
// =========================================================
static const uint8_t PIN_ENC_SHOOTER_A = 35;
static const uint8_t PIN_ENC_SHOOTER_B = 37;

static const uint8_t PIN_ENC_DRIVE_FL_A = 22;
static const uint8_t PIN_ENC_DRIVE_FL_B = 24;

static const uint8_t PIN_ENC_DRIVE_FR_A = 26;
static const uint8_t PIN_ENC_DRIVE_FR_B = 28;

static const uint8_t PIN_ENC_DW_PAR_A = 23;
static const uint8_t PIN_ENC_DW_PAR_B = 25;

static const uint8_t PIN_ENC_DW_PERP_A = 27;
static const uint8_t PIN_ENC_DW_PERP_B = 29;

// =========================================================
// VALID FLAGS
// Keep 0 as good/clean.
// =========================================================
static const uint16_t ENC_FLAGS_OK                = 0x0000;
static const uint16_t ENC_FLAG_NOT_INITIALIZED    = 0x0001;
static const uint16_t ENC_FLAG_ILLEGAL_TRANSITION = 0x0002;
static const uint16_t ENC_FLAG_STALE              = 0x0004;

// If no edge has been seen for this long, mark STALE in flags.
// This does not necessarily mean bad if the encoder is stationary; it tells Pi
// that timestamp_ms is old.
static const unsigned long ENCODER_STALE_MS = 1000UL;

// =========================================================
// REUSABLE ENCODER DEVICE STRUCTURE
// =========================================================
struct EncoderDevice {
  const char* name;
  uint8_t pinA;
  uint8_t pinB;

  // Direction multiplier lets you reverse one encoder without rewriting the
  // quadrature algorithm. Use +1 or -1.
  int8_t polarity;

  volatile long count;
  unsigned long timestamp_ms;  // millis() at most recent valid transition or reset
  uint16_t valid_flags;
  uint32_t illegal_transitions;
  uint8_t last_state;          // packed A/B: (A << 1) | B
  bool initialized;
};

// Quadrature transition table.
// Index = old_state * 4 + new_state, where state is AB as binary 0..3.
// Values:
//   +1 = forward step
//   -1 = reverse step
//    0 = no movement
//    2 = illegal transition, likely missed edge or noise
static const int8_t QUAD_TABLE[16] = {
   0, +1, -1,  2,
  -1,  0,  2, +1,
  +1,  2,  0, -1,
   2, -1, +1,  0
};

// =========================================================
// ENCODER REGISTRY
// Add/remove encoders here. This is the template part.
// =========================================================
EncoderDevice encoder_devices[] = {
  { DEV_ENCODER_SHOOTER,                 PIN_ENC_SHOOTER_A,   PIN_ENC_SHOOTER_B,   +1, 0L, 0UL, ENC_FLAG_NOT_INITIALIZED, 0UL, 0U, false },
  { DEV_ENCODER_DRIVE_FRONT_LEFT,        PIN_ENC_DRIVE_FL_A,  PIN_ENC_DRIVE_FL_B,  +1, 0L, 0UL, ENC_FLAG_NOT_INITIALIZED, 0UL, 0U, false },
  { DEV_ENCODER_DRIVE_FRONT_RIGHT,       PIN_ENC_DRIVE_FR_A,  PIN_ENC_DRIVE_FR_B,  +1, 0L, 0UL, ENC_FLAG_NOT_INITIALIZED, 0UL, 0U, false },
  { DEV_ENCODER_DEADWHEEL_PARALLEL,      PIN_ENC_DW_PAR_A,    PIN_ENC_DW_PAR_B,    +1, 0L, 0UL, ENC_FLAG_NOT_INITIALIZED, 0UL, 0U, false },
  { DEV_ENCODER_DEADWHEEL_PERPENDICULAR, PIN_ENC_DW_PERP_A,   PIN_ENC_DW_PERP_B,   +1, 0L, 0UL, ENC_FLAG_NOT_INITIALIZED, 0UL, 0U, false },
};

static const uint8_t NUM_ENCODERS = sizeof(encoder_devices) / sizeof(encoder_devices[0]);

// =========================================================
// GENERIC ENCODER ALGORITHM
// =========================================================
uint8_t readEncoderAB(const EncoderDevice &enc) {
  uint8_t a = digitalRead(enc.pinA) ? 1 : 0;
  uint8_t b = digitalRead(enc.pinB) ? 1 : 0;
  return (uint8_t)((a << 1) | b);
}

void initEncoder(EncoderDevice &enc) {
  pinMode(enc.pinA, INPUT_PULLUP);
  pinMode(enc.pinB, INPUT_PULLUP);

  enc.count = 0L;
  enc.timestamp_ms = millis();
  enc.valid_flags = ENC_FLAGS_OK;
  enc.illegal_transitions = 0UL;
  enc.last_state = readEncoderAB(enc);
  enc.initialized = true;
}

void zeroEncoder(EncoderDevice &enc) {
  noInterrupts();
  enc.count = 0L;
  enc.timestamp_ms = millis();
  enc.valid_flags = ENC_FLAGS_OK;
  enc.illegal_transitions = 0UL;
  enc.last_state = readEncoderAB(enc);
  enc.initialized = true;
  interrupts();
}

void updateEncoder(EncoderDevice &enc) {
  if (!enc.initialized) {
    initEncoder(enc);
    return;
  }

  uint8_t new_state = readEncoderAB(enc);
  if (new_state == enc.last_state) {
    return;
  }

  uint8_t index = (uint8_t)((enc.last_state << 2) | new_state);
  int8_t delta = QUAD_TABLE[index];

  if (delta == 2) {
    enc.illegal_transitions++;
    enc.valid_flags |= ENC_FLAG_ILLEGAL_TRANSITION;
    // Re-sync to current state so the next legal edge can be counted again.
    enc.last_state = new_state;
    return;
  }

  if (delta != 0) {
    enc.count += (long)(delta * enc.polarity);
    enc.timestamp_ms = millis();
  }

  enc.last_state = new_state;
}

void updateAllEncoders() {
  for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
    updateEncoder(encoder_devices[i]);
  }
}

EncoderDevice* findEncoderByName(const char* name) {
  for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
    if (strcmp(encoder_devices[i].name, name) == 0) {
      return &encoder_devices[i];
    }
  }
  return nullptr;
}

uint16_t currentEncoderFlags(const EncoderDevice &enc) {
  uint16_t flags = enc.valid_flags;

  if (!enc.initialized) {
    flags |= ENC_FLAG_NOT_INITIALIZED;
  }

  if (millis() - enc.timestamp_ms > ENCODER_STALE_MS) {
    flags |= ENC_FLAG_STALE;
  }

  return flags;
}

bool currentEncoderValid(const EncoderDevice &enc) {
  // Valid means the encoder has initialized and no illegal transition has been
  // seen since the most recent ZERO/RESET.
  uint16_t flags = currentEncoderFlags(enc);
  return (flags & (ENC_FLAG_NOT_INITIALIZED | ENC_FLAG_ILLEGAL_TRANSITION)) == 0;
}

void replyEncoder(const EncoderDevice &enc) {
  // Copy quickly in case this template later moves to interrupts.
  noInterrupts();
  long count = enc.count;
  unsigned long ts = enc.timestamp_ms;
  uint16_t flags = currentEncoderFlags(enc);
  bool valid = currentEncoderValid(enc);
  interrupts();

  PI_SERIAL.print("OK ENCODER ");
  PI_SERIAL.print(enc.name);
  PI_SERIAL.print(" count=");
  PI_SERIAL.print(count);
  PI_SERIAL.print(" timestamp_ms=");
  PI_SERIAL.print(ts);
  PI_SERIAL.print(" valid=");
  PI_SERIAL.print(valid ? 1 : 0);
  PI_SERIAL.print(" valid_flags=");
  PI_SERIAL.println(flags);
}

void replyAllEncoders() {
  for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
    replyEncoder(encoder_devices[i]);
  }
}

void zeroAllEncoders() {
  for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
    zeroEncoder(encoder_devices[i]);
  }
}

// =========================================================
// PI SERIAL COMMAND PARSER
// =========================================================
char piLineBuf[128];
uint8_t piLineIdx = 0;

void replyErr(const char *msg) {
  PI_SERIAL.print("ERR ");
  PI_SERIAL.println(msg);
}

void replyOkEncoderReset(const char* name) {
  PI_SERIAL.print("OK ENCODER ");
  PI_SERIAL.print(name);
  PI_SERIAL.println(" reset=1");
}

void handleEncoderCommand(const char* name, const char* action) {
  bool all = (strcmp(name, "*") == 0 || strcmp(name, "all") == 0 || strcmp(name, "ALL") == 0);

  if (strcmp(action, "READ") == 0) {
    if (all) {
      replyAllEncoders();
      return;
    }

    EncoderDevice* enc = findEncoderByName(name);
    if (!enc) {
      replyErr("UNKNOWN_ENCODER");
      return;
    }
    replyEncoder(*enc);
    return;
  }

  if (strcmp(action, "ZERO") == 0 || strcmp(action, "RESET") == 0) {
    if (all) {
      zeroAllEncoders();
      PI_SERIAL.println("OK ENCODER all reset=1");
      return;
    }

    EncoderDevice* enc = findEncoderByName(name);
    if (!enc) {
      replyErr("UNKNOWN_ENCODER");
      return;
    }
    zeroEncoder(*enc);
    replyOkEncoderReset(enc->name);
    return;
  }

  replyErr("UNSUPPORTED_ENCODER_ACTION");
}

void handlePiCommand(char *line) {
  while (*line == ' ') line++;

  if (strcmp(line, "HELLO") == 0) {
    PI_SERIAL.print("ID ");
    PI_SERIAL.print(DEVICE_ID);
    PI_SERIAL.print(" encoders=");
    PI_SERIAL.println(NUM_ENCODERS);
    return;
  }

  char category[20] = {0};
  char name[40] = {0};
  char action[20] = {0};

  int n = sscanf(line, "%19s %39s %19s", category, name, action);
  if (n != 3) {
    replyErr("BAD_COMMAND");
    return;
  }

  if (strcmp(category, "ENCODER") != 0) {
    replyErr("UNSUPPORTED_CATEGORY");
    return;
  }

  handleEncoderCommand(name, action);
}

void servicePiSerial() {
  while (PI_SERIAL.available()) {
    char c = (char)PI_SERIAL.read();

    if (c == '\r') continue;

    if (c == '\n') {
      piLineBuf[piLineIdx] = '\0';
      if (piLineIdx > 0) handlePiCommand(piLineBuf);
      piLineIdx = 0;
      continue;
    }

    if (piLineIdx < sizeof(piLineBuf) - 1) {
      piLineBuf[piLineIdx++] = c;
    } else {
      piLineIdx = 0;
      replyErr("LINE_TOO_LONG");
    }
  }
}

// =========================================================
// SETUP / LOOP
// =========================================================
void setup() {
  PI_SERIAL.begin(115200);

  for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
    initEncoder(encoder_devices[i]);
  }

  PI_SERIAL.print("BOOT ");
  PI_SERIAL.print(DEVICE_ID);
  PI_SERIAL.print(" encoders=");
  PI_SERIAL.println(NUM_ENCODERS);
}

void loop() {
  // Keep polling as fast as possible.
  updateAllEncoders();
  servicePiSerial();
}
