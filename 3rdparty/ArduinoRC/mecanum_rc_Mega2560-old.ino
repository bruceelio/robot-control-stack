// Mega2560_mecanum_rc_multipurpose.ino
//
// Updated with:
// - FlySky iBus on Serial2 using SERIAL_8N2
// - RoboClaw packet serial with CRC16
// - Right-stick translation mapping for mecanum:
//     CH2 = throttle (right stick up/down)
//     CH1 = strafe   (right stick left/right)
//     CH4 = rotate   (left stick left/right)
// - Signal-loss stop
// - Configurable drive scaling
//
// Output mapping for RoboClaw packet serial:
//   RoboClaw A:
//     M1 = Front Left
//     M2 = Rear Left
//   RoboClaw B:
//     M1 = Front Right
//     M2 = Rear Right

#include <Arduino.h>
#include <Servo.h>
#include <math.h>

// ------------------------- SELECT INPUT MODE -------------------------
#define USE_INPUT_PWM  0   // RC receiver PWM per channel
#define USE_INPUT_IBUS 1   // FlySky iBUS serial

// ------------------------- SELECT OUTPUT MODE ------------------------
#define USE_OUTPUT_RC_PWM        0   // Servo pulses to motor controllers / RoboClaw RC mode
#define USE_OUTPUT_ROBOCLAW_PKT  1   // Packet serial to 2x RoboClaw

// ------------------------- PIN DEFINITIONS ---------------------------

#if USE_INPUT_PWM
  const uint8_t PIN_RC_CH1_PWM = 2;   // CH1
  const uint8_t PIN_RC_CH2_PWM = 3;   // CH2
  const uint8_t PIN_RC_CH4_PWM = 21;  // CH4
#endif

#if USE_INPUT_IBUS
  // iBUS signal wire -> Mega RX2 (pin 17)
  const uint8_t PIN_RC_IBUS_RX = 17; // informational only; Serial2 uses pins 17/16
#endif

#if USE_OUTPUT_RC_PWM
  const uint8_t PIN_MTR_FL_PWM = 6;
  const uint8_t PIN_MTR_FR_PWM = 7;
  const uint8_t PIN_MTR_RL_PWM = 8;
  const uint8_t PIN_MTR_RR_PWM = 9;
#endif

#if USE_OUTPUT_ROBOCLAW_PKT
  // Serial1 = RoboClaw A (left side)
  // Serial3 = RoboClaw B (right side)
  const uint8_t ROBOCLAW_A_ADDR = 0x80;
  const uint8_t ROBOCLAW_B_ADDR = 0x81;   // change if your second RoboClaw uses a different address
#endif

// ------------------------- CONTROL PARAMETERS ------------------------
const float STOP_MS = 1.5f;
const float MAX_MS  = 1.9f;
const float MIN_MS  = 1.1f;
const float SPAN_MS = 0.4f;

// Translation / rotation scaling
const float DRIVE_SCALE  = 0.38f;   // lower top speed; adjust as needed
const float STRAFE_SCALE = 0.38f;   // match throttle by default
const float TURN_SCALE   = 0.45f;   // soften turning a bit

static inline float normalize_input(int value) {
  const int deadzone_min = 45;
  const int deadzone_max = 55;

  if (deadzone_min <= value && value <= deadzone_max) return 0.0f;
  if (value < deadzone_min) return (float)(value - deadzone_min) / (float)(deadzone_min);
  return (float)(value - deadzone_max) / (float)(100 - deadzone_max);
}

// ------------------------- OUTPUT IMPLEMENTATIONS --------------------
#if USE_OUTPUT_RC_PWM
Servo m_fl, m_fr, m_rl, m_rr;

static inline int msToUs(float ms) { return (int)(ms * 1000.0f + 0.5f); }

static inline float clampMs(float ms) {
  if (ms < MIN_MS) return MIN_MS;
  if (ms > MAX_MS) return MAX_MS;
  return ms;
}

void stop_all_motors() {
  m_fl.writeMicroseconds(msToUs(STOP_MS));
  m_fr.writeMicroseconds(msToUs(STOP_MS));
  m_rl.writeMicroseconds(msToUs(STOP_MS));
  m_rr.writeMicroseconds(msToUs(STOP_MS));
}

void output_mecanum(float fl_pwr, float fr_pwr, float rl_pwr, float rr_pwr) {
  float fl_ms = clampMs(STOP_MS + fl_pwr * SPAN_MS);
  float fr_ms = clampMs(STOP_MS + fr_pwr * SPAN_MS);
  float rl_ms = clampMs(STOP_MS + rl_pwr * SPAN_MS);
  float rr_ms = clampMs(STOP_MS + rr_pwr * SPAN_MS);

  m_fl.writeMicroseconds(msToUs(fl_ms));
  m_fr.writeMicroseconds(msToUs(fr_ms));
  m_rl.writeMicroseconds(msToUs(rl_ms));
  m_rr.writeMicroseconds(msToUs(rr_ms));
}
#endif

#if USE_OUTPUT_ROBOCLAW_PKT
// ------------------------- ROBOCLAW CRC16 ---------------------------
uint16_t crc_update(uint16_t crc, uint8_t data) {
  crc ^= (uint16_t)data << 8;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x8000) {
      crc = (crc << 1) ^ 0x1021;
    } else {
      crc <<= 1;
    }
  }
  return crc;
}

// Send a single-byte-speed command to a RoboClaw
void sendRoboClaw(HardwareSerial &port, uint8_t addr, uint8_t cmd, uint8_t val) {
  uint16_t crc = 0;

  port.write(addr);
  crc = crc_update(crc, addr);

  port.write(cmd);
  crc = crc_update(crc, cmd);

  port.write(val);
  crc = crc_update(crc, val);

  port.write((crc >> 8) & 0xFF);
  port.write(crc & 0xFF);
}

int toRoboSpeed(float pwr) {
  pwr = constrain(pwr, -1.0f, 1.0f);
  return (int)(pwr * 127.0f);
}

// RoboClaw A: FL = M1, RL = M2
void setFL(int speed) {
  speed = constrain(speed, -127, 127);
  if (speed >= 0) sendRoboClaw(Serial1, ROBOCLAW_A_ADDR, 0x00, (uint8_t)speed);   // M1 forward
  else            sendRoboClaw(Serial1, ROBOCLAW_A_ADDR, 0x01, (uint8_t)(-speed)); // M1 backward
}

void setRL(int speed) {
  speed = constrain(speed, -127, 127);
  if (speed >= 0) sendRoboClaw(Serial1, ROBOCLAW_A_ADDR, 0x04, (uint8_t)speed);   // M2 forward
  else            sendRoboClaw(Serial1, ROBOCLAW_A_ADDR, 0x05, (uint8_t)(-speed)); // M2 backward
}

// RoboClaw B: FR = M1, RR = M2
void setFR(int speed) {
  speed = constrain(speed, -127, 127);
  if (speed >= 0) sendRoboClaw(Serial3, ROBOCLAW_B_ADDR, 0x00, (uint8_t)speed);   // M1 forward
  else            sendRoboClaw(Serial3, ROBOCLAW_B_ADDR, 0x01, (uint8_t)(-speed)); // M1 backward
}

void setRR(int speed) {
  speed = constrain(speed, -127, 127);
  if (speed >= 0) sendRoboClaw(Serial3, ROBOCLAW_B_ADDR, 0x04, (uint8_t)speed);   // M2 forward
  else            sendRoboClaw(Serial3, ROBOCLAW_B_ADDR, 0x05, (uint8_t)(-speed)); // M2 backward
}

void stop_all_motors() {
  setFL(0);
  setFR(0);
  setRL(0);
  setRR(0);
}

void output_mecanum(float fl_pwr, float fr_pwr, float rl_pwr, float rr_pwr) {
  setFL(toRoboSpeed(fl_pwr));
  setFR(toRoboSpeed(fr_pwr));
  setRL(toRoboSpeed(rl_pwr));
  setRR(toRoboSpeed(rr_pwr));
}
#endif

// ------------------------- INPUT IMPLEMENTATIONS ---------------------
#if USE_INPUT_PWM
int readChannel0to100_PWM(uint8_t pin) {
  unsigned long us = pulseIn(pin, HIGH, 25000UL);
  if (us == 0) return 50; // neutral if missing signal

  if (us < 1000) us = 1000;
  if (us > 2000) us = 2000;

  long v = map((long)us, 1000L, 2000L, 0L, 100L);
  if (v < 0) v = 0;
  if (v > 100) v = 100;
  return (int)v;
}
#endif

#if USE_INPUT_IBUS
static const uint8_t IBUS_FRAME_LEN = 32;
static uint8_t ibus_buf[IBUS_FRAME_LEN];
static uint8_t ibus_idx = 0;
static unsigned long ibus_last_frame_ms = 0;
static uint16_t ibus_ch[14] = {1500};

bool ibus_read_frame() {
  while (IBUS_SERIAL.available()) {
    uint8_t b = (uint8_t)IBUS_SERIAL.read();

    if (ibus_idx == 0) {
      if (b != 0x20) continue;
      ibus_buf[ibus_idx++] = b;
      continue;
    }

    if (ibus_idx == 1) {
      if (b != 0x40) {
        ibus_idx = 0;
        continue;
      }
      ibus_buf[ibus_idx++] = b;
      continue;
    }

    ibus_buf[ibus_idx++] = b;

    if (ibus_idx >= IBUS_FRAME_LEN) {
      uint16_t sum = 0xFFFF;
      for (uint8_t i = 0; i < IBUS_FRAME_LEN - 2; i++) sum -= ibus_buf[i];
      uint16_t rxsum = (uint16_t)ibus_buf[IBUS_FRAME_LEN - 2] | ((uint16_t)ibus_buf[IBUS_FRAME_LEN - 1] << 8);

      ibus_idx = 0;
      if (sum != rxsum) return false;

      for (uint8_t ch = 0; ch < 14; ch++) {
        uint8_t lo = ibus_buf[2 + ch * 2];
        uint8_t hi = ibus_buf[3 + ch * 2];
        ibus_ch[ch] = (uint16_t)lo | ((uint16_t)hi << 8);
      }
      ibus_last_frame_ms = millis();
      return true;
    }
  }
  return false;
}

int ibus_channel0to100(uint8_t ch_index) {
  if (ch_index >= 14) return 50;
  if (millis() - ibus_last_frame_ms > 200) return 50;

  uint16_t us = ibus_ch[ch_index];
  if (us < 1000) us = 1000;
  if (us > 2000) us = 2000;

  long v = map((long)us, 1000L, 2000L, 0L, 100L);
  if (v < 0) v = 0;
  if (v > 100) v = 100;
  return (int)v;
}
#endif

// ------------------------- DRIVE LOGIC -------------------------------
void drive(int throttle_sp, int strafe_sp, int rotate_sp) {
  float fwd    = normalize_input(throttle_sp);
  float strafe = normalize_input(strafe_sp);
  float rotate = normalize_input(rotate_sp);

  if (fwd == 0.0f && strafe == 0.0f && rotate == 0.0f) {
    stop_all_motors();
    return;
  }

  // Apply scaling
  fwd    *= DRIVE_SCALE;
  strafe *= STRAFE_SCALE;
  rotate *= TURN_SCALE;

  // Mecanum mix
  float fl = fwd + strafe + rotate;
  float fr = fwd - strafe - rotate;
  float rl = fwd - strafe + rotate;
  float rr = fwd + strafe - rotate;

  float max_power = max(max(fabs(fl), fabs(fr)), max(fabs(rl), fabs(rr)));
  if (max_power > 1.0f) {
    fl /= max_power;
    fr /= max_power;
    rl /= max_power;
    rr /= max_power;
  }

  output_mecanum(fl, fr, rl, rr);
}

// ------------------------- SETUP / LOOP ------------------------------
void setup() {
#if USE_INPUT_PWM
  pinMode(PIN_RC_CH1_PWM, INPUT);
  pinMode(PIN_RC_CH2_PWM, INPUT);
  pinMode(PIN_RC_CH4_PWM, INPUT);
#endif

#if USE_INPUT_IBUS
  IBUS_SERIAL.begin(115200, SERIAL_8N2);
#endif

#if USE_OUTPUT_RC_PWM
  m_fl.attach(PIN_MTR_FL_PWM);
  m_fr.attach(PIN_MTR_FR_PWM);
  m_rl.attach(PIN_MTR_RL_PWM);
  m_rr.attach(PIN_MTR_RR_PWM);
#endif

#if USE_OUTPUT_ROBOCLAW_PKT
  Serial1.begin(38400); // RoboClaw A
  Serial3.begin(38400); // RoboClaw B
#endif

  stop_all_motors();
}

void loop() {
  int throttle = 50, strafe = 50, rotate = 50;

#if USE_INPUT_PWM
  // Original PWM mapping can remain whatever your receiver outputs are.
  throttle = readChannel0to100_PWM(PIN_RC_CH2_PWM);
  strafe   = readChannel0to100_PWM(PIN_RC_CH1_PWM);
  rotate   = readChannel0to100_PWM(PIN_RC_CH4_PWM);
#endif

#if USE_INPUT_IBUS
  ibus_read_frame();

  // Updated FlySky mapping:
  // CH2 = right stick up/down   -> throttle
  // CH1 = right stick left/right -> strafe
  // CH4 = left stick left/right  -> rotate
  throttle = ibus_channel0to100(1); // CH2
  strafe   = ibus_channel0to100(0); // CH1
  rotate   = ibus_channel0to100(3); // CH4
#endif

  drive(throttle, strafe, rotate);
  delay(20);
}