// NEED TO UPDATE WITH LATEST 2wd_Mega2560.ino configurations !!!
//
// Mega2560_mecanum_rc_multipurpose.ino
//
// Default configuration (as shipped in this file):
//   INPUT  = RC PWM per-channel (pulseIn on CH1/CH2/CH4)
//   OUTPUT = RC PWM (servo-style) outputs to motor controllers / RoboClaw in RC mode
//
// To switch modes: comment/uncomment the #define values below.

#include <Arduino.h>
#include <Servo.h>

// ------------------------- SELECT INPUT MODE -------------------------
// DEFAULT: PWM input ON, iBUS OFF
#define USE_INPUT_PWM  1   // RC receiver PWM per channel (CH1/CH2/CH4)
#define USE_INPUT_IBUS 0   // FlySky iBUS serial (single wire)

// ------------------------- SELECT OUTPUT MODE ------------------------
// DEFAULT: RC PWM output ON, RoboClaw packet serial OFF
#define USE_OUTPUT_RC_PWM        1   // Servo pulses to motor controllers / RoboClaw RC mode
#define USE_OUTPUT_ROBOCLAW_PKT  0   // Packet serial to RoboClaw (requires 2 RoboClaws for 4 motors)

// ------------------------- PIN DEFINITIONS ---------------------------
//
// Mega external interrupt pins: 2, 3, 18, 19, 20, 21
// We'll use 2,3,21 for RC PWM inputs to avoid clashing with Serial1 pins (18/19).

#if USE_INPUT_PWM
  const uint8_t PIN_RC_CH1_PWM = 2;   // CH1 (THROTTLE)  - INT4-capable not required for pulseIn, but nice
  const uint8_t PIN_RC_CH2_PWM = 3;   // CH2 (STRAFE)
  const uint8_t PIN_RC_CH4_PWM = 21;  // CH4 (ROTATE)    - avoids Serial1 pins
#endif

#if USE_INPUT_IBUS
  // iBUS signal wire -> Mega RX2 (pin 17). We'll use Serial2.
  const uint8_t PIN_RC_IBUS_RX = 17; // RX2 (informational; Serial2 uses 17/16)
#endif

#if USE_OUTPUT_RC_PWM
  // Servo-style outputs (1-2ms @ 50Hz) to motor controllers / RoboClaw RC mode.
  const uint8_t PIN_MTR_FL_PWM = 6;
  const uint8_t PIN_MTR_FR_PWM = 7;
  const uint8_t PIN_MTR_RL_PWM = 8;
  const uint8_t PIN_MTR_RR_PWM = 9;
#endif

#if USE_OUTPUT_ROBOCLAW_PKT
  // RoboClaw packet serial example uses Serial1 (RX1=19, TX1=18) - do not use those pins elsewhere.
#endif

// ------------------------- CONTROL PARAMETERS ------------------------
const float STOP_MS = 1.5f;
const float MAX_MS  = 1.9f;
const float MIN_MS  = 1.1f;
const float SPAN_MS = 0.4f;

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
static const uint8_t ROBOCLAW_ADDR = 0x80;

void stop_all_motors() {
  // TODO: implement via RoboClaw packet serial
}

void output_mecanum(float fl_pwr, float fr_pwr, float rl_pwr, float rr_pwr) {
  // TODO: implement via RoboClaw packet serial (requires 2x RoboClaw for 4 motors)
  (void)fl_pwr; (void)fr_pwr; (void)rl_pwr; (void)rr_pwr;
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
  while (Serial2.available()) {
    uint8_t b = (uint8_t)Serial2.read();

    if (ibus_idx == 0) {
      if (b != 0x20) continue;
      ibus_buf[ibus_idx++] = b;
      continue;
    }
    if (ibus_idx == 1) {
      if (b != 0x40) { ibus_idx = 0; continue; }
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
void drive(int throttle_sp, int strafe_sp, int rotate_sp, float input_scale = 0.5f) {
  float fwd    = normalize_input(throttle_sp);
  float strafe = normalize_input(strafe_sp);
  float rotate = normalize_input(rotate_sp);

  if (fwd == 0.0f && strafe == 0.0f && rotate == 0.0f) {
    stop_all_motors();
    return;
  }

  fwd    *= input_scale;
  strafe *= input_scale;
  rotate *= input_scale;

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
  Serial2.begin(115200);
#endif

#if USE_OUTPUT_RC_PWM
  m_fl.attach(PIN_MTR_FL_PWM);
  m_fr.attach(PIN_MTR_FR_PWM);
  m_rl.attach(PIN_MTR_RL_PWM);
  m_rr.attach(PIN_MTR_RR_PWM);
#endif

#if USE_OUTPUT_ROBOCLAW_PKT
  Serial1.begin(38400); // adjust to match RoboClaw setting
#endif

  stop_all_motors();
}

void loop() {
  int throttle = 50, strafe = 50, rotate = 50;

#if USE_INPUT_PWM
  throttle = readChannel0to100_PWM(PIN_RC_CH1_PWM);
  strafe   = readChannel0to100_PWM(PIN_RC_CH2_PWM);
  rotate   = readChannel0to100_PWM(PIN_RC_CH4_PWM);
#endif

#if USE_INPUT_IBUS
  ibus_read_frame();
  throttle = ibus_channel0to100(0); // CH1
  strafe   = ibus_channel0to100(1); // CH2
  rotate   = ibus_channel0to100(3); // CH4
#endif

  drive(throttle, strafe, rotate, 0.5f);
  delay(20);
}