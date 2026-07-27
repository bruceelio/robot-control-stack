#include <Arduino.h>

// ---------------- CONFIG ----------------
#define ROBOCLAW_ADDR 0x80
#define ROBOCLAW_SERIAL Serial1

// ---------------- CRC16 (required by RoboClaw) ----------------
uint16_t crc_update(uint16_t crc, uint8_t data) {
  crc ^= (uint16_t)data << 8;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x1021;
    else
      crc <<= 1;
  }
  return crc;
}

// ---------------- SEND PACKET ----------------
void sendCommand(uint8_t cmd, uint8_t value) {
  uint16_t crc = 0;

  ROBOCLAW_SERIAL.write(ROBOCLAW_ADDR);
  crc = crc_update(crc, ROBOCLAW_ADDR);

  ROBOCLAW_SERIAL.write(cmd);
  crc = crc_update(crc, cmd);

  ROBOCLAW_SERIAL.write(value);
  crc = crc_update(crc, value);

  // send CRC
  ROBOCLAW_SERIAL.write((crc >> 8) & 0xFF);
  ROBOCLAW_SERIAL.write(crc & 0xFF);
}

// ---------------- HELPERS ----------------
void M1Forward(uint8_t speed) { sendCommand(0x00, speed); }
void M1Backward(uint8_t speed) { sendCommand(0x01, speed); }
void M2Forward(uint8_t speed) { sendCommand(0x04, speed); }
void M2Backward(uint8_t speed) { sendCommand(0x05, speed); }

void stopAll() {
  M1Forward(0);
  M2Forward(0);
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  ROBOCLAW_SERIAL.begin(38400);

  delay(1000);
  Serial.println("RoboClaw CRC test starting...");

  stopAll();
}

// ---------------- LOOP ----------------
void loop() {
  Serial.println("M1 forward");
  M1Forward(30);
  delay(2000);

  Serial.println("M1 stop");
  stopAll();
  delay(1000);

  Serial.println("M1 backward");
  M1Backward(30);
  delay(2000);

  Serial.println("M2 forward");
  M2Forward(30);
  delay(2000);

  Serial.println("M2 backward");
  M2Backward(30);
  delay(2000);

  Serial.println("STOP");
  stopAll();
  delay(3000);
}