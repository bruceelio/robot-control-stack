// 3rdparty/ArduinoRC/2wd_Mega2560.ino

#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ------------------------- CONFIG -------------------------
#define ROBOCLAW_ADDR 0x80

#define PI_SERIAL Serial        // USB serial to Raspberry Pi
#define IBUS_SERIAL Serial2     // RX2 = pin 17
#define ROBOCLAW_SERIAL Serial1 // TX1 = pin 18

// Servo pins
static const uint8_t SERVO_GRIP_LEFT_PIN  = 11;
static const uint8_t SERVO_GRIP_RIGHT_PIN = 13;

// FlySky channel assignment (1-based for readability)
// CH1 = right stick left/right
// CH2 = right stick up/down
// CH5/CH6 are commonly switches/knobs on FlySky radios.
static const uint8_t CH_GRIP = 5;

// Grip servo calibration in microseconds.
// Keep the known-good open endpoints and extend the closed endpoints as needed.
static const int LEFT_OPEN_US    = 900;
static const int LEFT_CLOSED_US  = 2200;

static const int RIGHT_OPEN_US   = 2100;
static const int RIGHT_CLOSED_US = 800;

// ------------------------- PI CONTROL ---------------------
static const char DEVICE_ID[] = "MEGA_AUX_1";
static const unsigned long PI_HEARTBEAT_TIMEOUT_MS = 3000;

bool piAutoRequested = false;
unsigned long piLastHeartbeatMs = 0;

float piLeftCmd = 0.0f;
float piRightCmd = 0.0f;
float piGripCmd = -1.0f;   // -1=open, +1=closed

char piLineBuf[64];
uint8_t piLineIdx = 0;

// ------------------------- IBUS ---------------------------
static const uint8_t IBUS_FRAME_LEN = 32;
uint8_t ibus_buf[IBUS_FRAME_LEN];
uint8_t ibus_idx = 0;
uint16_t ibus_ch[14] = {1500};
unsigned long ibus_last_frame_ms = 0;

Servo gripLeftServo;
Servo gripRightServo;

bool readIbusFrame() {
  while (IBUS_SERIAL.available()) {
    uint8_t b = IBUS_SERIAL.read();

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

    if (ibus_idx == IBUS_FRAME_LEN) {
      uint16_t sum = 0xFFFF;
      for (int i = 0; i < IBUS_FRAME_LEN - 2; i++) {
        sum -= ibus_buf[i];
      }

      uint16_t rxsum = (uint16_t)ibus_buf[30] | ((uint16_t)ibus_buf[31] << 8);
      ibus_idx = 0;

      if (sum != rxsum) return false;

      for (int ch = 0; ch < 14; ch++) {
        ibus_ch[ch] = (uint16_t)ibus_buf[2 + ch * 2] |
                      ((uint16_t)ibus_buf[3 + ch * 2] << 8);
      }

      ibus_last_frame_ms = millis();
      return true;
    }
  }
  return false;
}

uint16_t ibusMicros(uint8_t chZeroBased) {
  if (chZeroBased >= 14) return 1500;

  uint16_t us = ibus_ch[chZeroBased];
  if (us < 1000) us = 1000;
  if (us > 2000) us = 2000;
  return us;
}

int ibusToPercent(uint8_t chZeroBased) {
  uint16_t us = ibusMicros(chZeroBased);
  return map(us, 1000, 2000, 0, 100);
}

// ------------------------- NORMALIZATION ------------------
float normalize(int val) {
  const int dead_min = 45;
  const int dead_max = 55;

  if (val >= dead_min && val <= dead_max) return 0.0f;
  if (val < dead_min) return (float)(val - dead_min) / (float)dead_min;
  return (float)(val - dead_max) / (float)(100 - dead_max);
}

// ------------------------- ROBOCLAW CRC16 -----------------
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

// ------------------------- ROBOCLAW SEND ------------------
void sendRoboClaw(uint8_t cmd, uint8_t val) {
  uint16_t crc = 0;

  ROBOCLAW_SERIAL.write(ROBOCLAW_ADDR);
  crc = crc_update(crc, ROBOCLAW_ADDR);

  ROBOCLAW_SERIAL.write(cmd);
  crc = crc_update(crc, cmd);

  ROBOCLAW_SERIAL.write(val);
  crc = crc_update(crc, val);

  ROBOCLAW_SERIAL.write((crc >> 8) & 0xFF);
  ROBOCLAW_SERIAL.write(crc & 0xFF);
}

int toRoboSpeed(float pwr) {
  pwr = constrain(pwr, -1.0f, 1.0f);
  return (int)(pwr * 127.0f);
}

void setLeft(int speed) {
  speed = constrain(speed, -127, 127);

  if (speed >= 0) {
    sendRoboClaw(0x00, (uint8_t)speed);    // M1 forward
  } else {
    sendRoboClaw(0x01, (uint8_t)(-speed)); // M1 backward
  }
}

void setRight(int speed) {
  speed = constrain(speed, -127, 127);

  if (speed >= 0) {
    sendRoboClaw(0x04, (uint8_t)speed);    // M2 forward
  } else {
    sendRoboClaw(0x05, (uint8_t)(-speed)); // M2 backward
  }
}

void stopMotors() {
  setLeft(0);
  setRight(0);
}

// ------------------------- SERVOS -------------------------
void setGripPosition(uint16_t gripUs) {
  gripUs = constrain(gripUs, 1000, 2000);

  int leftUs  = map(gripUs, 1000, 2000, LEFT_OPEN_US,  LEFT_CLOSED_US);
  int rightUs = map(gripUs, 1000, 2000, RIGHT_OPEN_US, RIGHT_CLOSED_US);

  gripLeftServo.writeMicroseconds(leftUs);
  gripRightServo.writeMicroseconds(rightUs);
}

void updateGripFromIbus() {
  // CH_GRIP is 1-based in the config block; convert to 0-based here.
  const uint8_t idx = CH_GRIP - 1;
  const uint16_t gripUs = ibusMicros(idx);

  setGripPosition(gripUs);
}

void setGripNormalized(float pos) {
  pos = constrain(pos, -1.0f, 1.0f);
  uint16_t gripUs = (uint16_t)map((int)(pos * 1000.0f), -1000, 1000, 1000, 2000);
  setGripPosition(gripUs);
}

void setDriveNormalized(float left, float right) {
  left  = constrain(left,  -1.0f, 1.0f);
  right = constrain(right, -1.0f, 1.0f);

  setLeft(toRoboSpeed(left));
  setRight(toRoboSpeed(right));
}

// ------------------------- PI SERIAL ----------------------
bool piHeartbeatFresh() {
  return (millis() - piLastHeartbeatMs) <= PI_HEARTBEAT_TIMEOUT_MS;
}

bool piHasControl() {
  return piAutoRequested && piHeartbeatFresh();
}

bool setMotorByName(const char* name, float value) {
  value = constrain(value, -1.0f, 1.0f);

  if (strcmp(name, "drive_front_left") == 0) {
    piLeftCmd = value;
    return true;
  }

  if (strcmp(name, "drive_front_right") == 0) {
    piRightCmd = value;
    return true;
  }

  return false;
}

bool setServoByName(const char* name, float value) {
  value = constrain(value, -1.0f, 1.0f);

  if (strcmp(name, "gripper") == 0) {
    piGripCmd = value;
    return true;
  }

  return false;
}

void handlePiCommand(char *line) {
  while (*line == ' ') line++;

  if (strcmp(line, "HELLO") == 0) {
    PI_SERIAL.print("ID ");
    PI_SERIAL.println(DEVICE_ID);
    return;
  }

  if (strcmp(line, "MODE AUTO") == 0) {
    piAutoRequested = true;
    piLastHeartbeatMs = millis();
    PI_SERIAL.println("OK MODE AUTO");
    return;
  }

  if (strcmp(line, "MODE TELEOP") == 0) {
    piAutoRequested = false;
    PI_SERIAL.println("OK MODE TELEOP");
    return;
  }

  if (strcmp(line, "STOP") == 0) {
    piLeftCmd = 0.0f;
    piRightCmd = 0.0f;
    stopMotors();
    PI_SERIAL.println("OK STOP");
    return;
  }

  unsigned long hbSeq = 0;
  if (sscanf(line, "HB %lu", &hbSeq) == 1) {
    piLastHeartbeatMs = millis();
    PI_SERIAL.print("OK HB ");
    PI_SERIAL.println(hbSeq);
    return;
  }

  char kind[16];
  char name[32];
  float value = 0.0f;

  if (sscanf(line, "SET %15s %31s %f", kind, name, &value) == 3) {
    if (strcmp(kind, "MOTOR") == 0) {
      if (setMotorByName(name, value)) {
        PI_SERIAL.print("OK SET MOTOR ");
        PI_SERIAL.println(name);
      } else {
        PI_SERIAL.print("ERR SET MOTOR ");
        PI_SERIAL.println(name);
      }
      return;
    }

    if (strcmp(kind, "SERVO") == 0) {
      if (setServoByName(name, value)) {
        PI_SERIAL.print("OK SET SERVO ");
        PI_SERIAL.println(name);
      } else {
        PI_SERIAL.print("ERR SET SERVO ");
        PI_SERIAL.println(name);
      }
      return;
    }
  }

  // Optional temporary backward compatibility during migration
  if (strncmp(line, "DRV ", 4) == 0) {
    char *p = line + 4;
    char *tok1 = strtok(p, " ");
    char *tok2 = strtok(nullptr, " ");

    if (tok1 && tok2) {
      float a = atof(tok1);
      float b = atof(tok2);

      piLeftCmd = constrain(a, -1.0f, 1.0f);
      piRightCmd = constrain(b, -1.0f, 1.0f);

      PI_SERIAL.println("OK DRV");
      return;
    }
  }

  if (strncmp(line, "GRIP ", 5) == 0) {
    char *p = line + 5;
    float grip = atof(p);

    piGripCmd = constrain(grip, -1.0f, 1.0f);

    PI_SERIAL.println("OK GRIP");
    return;
  }

  PI_SERIAL.print("ERR ");
  PI_SERIAL.println(line);
}

void servicePiSerial() {
  while (PI_SERIAL.available()) {
    char c = (char)PI_SERIAL.read();

    if (c == '\r') continue;

    if (c == '\n') {
      piLineBuf[piLineIdx] = '\0';
      if (piLineIdx > 0) {
        handlePiCommand(piLineBuf);
      }
      piLineIdx = 0;
      continue;
    }

    if (piLineIdx < sizeof(piLineBuf) - 1) {
      piLineBuf[piLineIdx++] = c;
    } else {
      piLineIdx = 0;
    }
  }
}

// ------------------------- SETUP --------------------------
void setup() {
  PI_SERIAL.begin(115200);
  IBUS_SERIAL.begin(115200, SERIAL_8N2);
  ROBOCLAW_SERIAL.begin(38400);

  gripLeftServo.attach(SERVO_GRIP_LEFT_PIN);
  gripRightServo.attach(SERVO_GRIP_RIGHT_PIN);
  setGripPosition(1000);  // start open

  stopMotors();

  PI_SERIAL.print("BOOT ");
  PI_SERIAL.println(DEVICE_ID);
}

// ------------------------- LOOP ---------------------------
void loop() {
  servicePiSerial();
  readIbusFrame();

  // If Pi has fresh heartbeat and requested AUTO, Pi owns the outputs.
  if (piHasControl()) {
    setDriveNormalized(piLeftCmd, piRightCmd);
    setGripNormalized(piGripCmd);
    delay(20);
    return;
  }

  // If AUTO was requested but heartbeat expired, drop back to teleop.
  if (piAutoRequested && !piHeartbeatFresh()) {
    piAutoRequested = false;
    stopMotors();
  }

  // Safety: stop drive if FlySky signal lost.
  // Keep servos at their last commanded position.
  if (millis() - ibus_last_frame_ms > 200) {
    stopMotors();
    return;
  }

  // RIGHT STICK DRIVING (Mode 2)
  // CH2 = right stick up/down
  // CH1 = right stick left/right
  int throttle = ibusToPercent(1);
  int rotate   = ibusToPercent(0);

  float fwd  = normalize(throttle);
  float turn = normalize(rotate);

  // Speed tuning
  float driveScale = 0.38f;  // target ~117 rpm feel from 312 rpm motor
  float turnScale  = 0.45f;  // soften steering

  fwd  *= driveScale;
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

  // Output drive and gripper
  setLeft(toRoboSpeed(left));
  setRight(toRoboSpeed(right));
  updateGripFromIbus();

  delay(20);
}