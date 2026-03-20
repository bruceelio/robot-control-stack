// Mega2560_2WD_FlySky_iBus_RoboClaw_RIGHT_STICK.ino

#include <Arduino.h>

// ------------------------- CONFIG -------------------------
#define ROBOCLAW_ADDR 0x80

#define IBUS_SERIAL Serial2     // RX2 = pin 17
#define ROBOCLAW_SERIAL Serial1 // TX1 = pin 18

// ------------------------- IBUS ---------------------------
static const uint8_t IBUS_FRAME_LEN = 32;
uint8_t ibus_buf[IBUS_FRAME_LEN];
uint8_t ibus_idx = 0;
uint16_t ibus_ch[14] = {1500};
unsigned long ibus_last_frame_ms = 0;

bool readIbusFrame() {
  while (IBUS_SERIAL.available()) {
    uint8_t b = IBUS_SERIAL.read();

    if (ibus_idx == 0 && b != 0x20) continue;
    if (ibus_idx == 1 && b != 0x40) { ibus_idx = 0; continue; }

    ibus_buf[ibus_idx++] = b;

    if (ibus_idx == IBUS_FRAME_LEN) {
      uint16_t sum = 0xFFFF;
      for (int i = 0; i < IBUS_FRAME_LEN - 2; i++) sum -= ibus_buf[i];

      uint16_t rxsum = ibus_buf[30] | (ibus_buf[31] << 8);
      ibus_idx = 0;

      if (sum != rxsum) return false;

      for (int ch = 0; ch < 14; ch++) {
        ibus_ch[ch] = ibus_buf[2 + ch * 2] | (ibus_buf[3 + ch * 2] << 8);
      }

      ibus_last_frame_ms = millis();
      return true;
    }
  }
  return false;
}

int ibusToPercent(uint8_t ch) {
  if (ch >= 14) return 50;

  uint16_t us = ibus_ch[ch];
  if (us < 1000) us = 1000;
  if (us > 2000) us = 2000;

  return map(us, 1000, 2000, 0, 100);
}

// ------------------------- NORMALIZATION ------------------
float normalize(int val) {
  const int dead_min = 45;
  const int dead_max = 55;

  if (val >= dead_min && val <= dead_max) return 0.0f;
  if (val < dead_min) return (float)(val - dead_min) / dead_min;
  return (float)(val - dead_max) / (100 - dead_max);
}

// ------------------------- ROBOCLAW -----------------------
int toRoboSpeed(float pwr) {
  pwr = constrain(pwr, -1.0f, 1.0f);
  return (int)(pwr * 127.0f);
}

void sendRoboClaw(uint8_t cmd, uint8_t val) {
  uint16_t crc = ROBOCLAW_ADDR + cmd + val;

  ROBOCLAW_SERIAL.write(ROBOCLAW_ADDR);
  ROBOCLAW_SERIAL.write(cmd);
  ROBOCLAW_SERIAL.write(val);
  ROBOCLAW_SERIAL.write(crc >> 8);
  ROBOCLAW_SERIAL.write(crc & 0xFF);
}

void setLeft(int speed) {
  if (speed >= 0)
    sendRoboClaw(0x00, speed);   // M1 forward
  else
    sendRoboClaw(0x01, -speed);  // M1 backward
}

void setRight(int speed) {
  if (speed >= 0)
    sendRoboClaw(0x04, speed);   // M2 forward
  else
    sendRoboClaw(0x05, -speed);  // M2 backward
}

void stopMotors() {
  setLeft(0);
  setRight(0);
}

// ------------------------- SETUP --------------------------
void setup() {
  IBUS_SERIAL.begin(115200);
  ROBOCLAW_SERIAL.begin(38400);

  stopMotors();
}

// ------------------------- LOOP ---------------------------
void loop() {
  readIbusFrame();

  // Safety: stop if signal lost
  if (millis() - ibus_last_frame_ms > 200) {
    stopMotors();
    return;
  }

  // ✅ RIGHT STICK DRIVING (Mode 2)
  int throttle = ibusToPercent(1); // CH2 → Right stick up/down
  int rotate   = ibusToPercent(0); // CH1 → Right stick left/right

  float fwd  = normalize(throttle);
  float turn = normalize(rotate);

  // Optional: soften turning
  float turnScale = 0.7f;
  turn *= turnScale;

  // Differential drive
  float left  = fwd + turn;
  float right = fwd - turn;

  // Normalize
  float maxVal = max(fabs(left), fabs(right));
  if (maxVal > 1.0f) {
    left  /= maxVal;
    right /= maxVal;
  }

  // Output
  setLeft(toRoboSpeed(left));
  setRight(toRoboSpeed(right));

  delay(20);
}