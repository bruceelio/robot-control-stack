// 3rdparty/ArduinoRC/2wd_Mega2560.ino
//
// Hardware-native Mega sketch with rollback-style gripper path.
// Keeps new LINK motor protocol, restores old simple gripper behavior.

#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define PI_SERIAL Serial
#define ROBOCLAW_B_SERIAL Serial3
#define IBUS_SERIAL Serial2
#define ROBOCLAW_A_SERIAL Serial1

static const uint8_t PIN_ROBOCLAW_B_TX = 14;
static const uint8_t PIN_ROBOCLAW_B_RX = 15;
static const uint8_t PIN_RC_IBUS_TX = 16;
static const uint8_t PIN_RC_IBUS_RX = 17;
static const uint8_t PIN_ROBOCLAW_A_TX = 18;
static const uint8_t PIN_ROBOCLAW_A_RX = 19;

static const uint8_t PIN_SHOOTER_PWM   = 4;
static const uint8_t PIN_COLLECTOR_PWM = 5;

static const uint8_t PIN_GRIP_LEFT  = 11;
static const uint8_t PIN_LIFT       = 12;
static const uint8_t PIN_GRIP_RIGHT = 13;

static const uint8_t PIN_DEADWHEEL_PARALLEL_A      = 22;
static const uint8_t PIN_DEADWHEEL_PARALLEL_B      = 23;
static const uint8_t PIN_DEADWHEEL_PERPENDICULAR_A = 24;
static const uint8_t PIN_DEADWHEEL_PERPENDICULAR_B = 25;
static const uint8_t PIN_LIFT_LIMIT_HIGH = 26;
static const uint8_t PIN_LIFT_LIMIT_LOW  = 27;
static const uint8_t PIN_SHOOTER_ENC_A = 28;
static const uint8_t PIN_SHOOTER_ENC_B = 29;

static const uint8_t PIN_SHOOTER_INA     = 30;
static const uint8_t PIN_SHOOTER_INB     = 31;
static const uint8_t PIN_SHOOTER_EN_DIAG = 32;
static const uint8_t PIN_COLLECTOR_INA     = 33;
static const uint8_t PIN_COLLECTOR_INB     = 34;
static const uint8_t PIN_COLLECTOR_EN_DIAG = 35;

#define ROBOCLAW_ADDR_A 0x80
#define ROBOCLAW_ADDR_B 0x81

static const uint8_t CH_DRIVE_ROTATE   = 1;
static const uint8_t CH_DRIVE_THROTTLE = 2;
static const uint8_t CH_GRIP           = 5;

static const int LEFT_OPEN_US    = 900;
static const int LEFT_CLOSED_US  = 2200;
static const int RIGHT_OPEN_US   = 2100;
static const int RIGHT_CLOSED_US = 800;

static const int LIFT_DOWN_US    = 1000;
static const int LIFT_UP_US      = 2000;

static const char BATTERY_VOLTAGE_PIN[] = "47";
static const char DEVICE_ID[] = "MEGA_AUX_1";
static const unsigned long PI_HEARTBEAT_TIMEOUT_MS = 500;

bool piAutoRequested = false;
unsigned long piLastHeartbeatMs = 0;

float piLink_18_19_M1 = 0.0f;
float piLink_18_19_M2 = 0.0f;
float piLink_14_15_M1 = 0.0f;
float piLink_14_15_M2 = 0.0f;

float piLiftCmd = 0.0f;
uint16_t piGripLeftUs = LEFT_OPEN_US;
uint16_t piGripRightUs = RIGHT_OPEN_US;

bool  directServoAttached[70];
Servo directServoObjects[70];

char piLineBuf[160];
uint8_t piLineIdx = 0;

static const uint8_t IBUS_FRAME_LEN = 32;
uint8_t ibus_buf[IBUS_FRAME_LEN];
uint8_t ibus_idx = 0;
uint16_t ibus_ch[14] = {1500};
unsigned long ibus_last_frame_ms = 0;

Servo gripLeftServo;
Servo gripRightServo;
Servo liftServo;

bool readIbusFrame() {
  while (IBUS_SERIAL.available()) {
    uint8_t b = IBUS_SERIAL.read();
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
    if (ibus_idx == IBUS_FRAME_LEN) {
      uint16_t sum = 0xFFFF;
      for (int i = 0; i < IBUS_FRAME_LEN - 2; i++) sum -= ibus_buf[i];
      uint16_t rxsum = ibus_buf[30] | (ibus_buf[31] << 8);
      ibus_idx = 0;
      if (sum != rxsum) return false;
      for (int ch = 0; ch < 14; ch++) ibus_ch[ch] = ibus_buf[2 + ch * 2] | (ibus_buf[3 + ch * 2] << 8);
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

float normalize(int val) {
  const int dead_min = 45, dead_max = 55;
  if (val >= dead_min && val <= dead_max) return 0.0f;
  if (val < dead_min) return (float)(val - dead_min) / (float)dead_min;
  return (float)(val - dead_max) / (float)(100 - dead_max);
}

uint16_t crc_update(uint16_t crc, uint8_t data) {
  crc ^= (uint16_t)data << 8;
  for (uint8_t i = 0; i < 8; i++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  return crc;
}

void sendRoboClaw(HardwareSerial &port, uint8_t addr, uint8_t cmd, uint8_t val) {
  uint16_t crc = 0;
  port.write(addr); crc = crc_update(crc, addr);
  port.write(cmd);  crc = crc_update(crc, cmd);
  port.write(val);  crc = crc_update(crc, val);
  port.write((crc >> 8) & 0xFF);
  port.write(crc & 0xFF);
}

int toRoboSpeed(float pwr) {
  pwr = constrain(pwr, -1.0f, 1.0f);
  return (int)(pwr * 127.0f);
}

void writeRoboClawM1(HardwareSerial &port, uint8_t addr, float pwr) {
  int speed = constrain(toRoboSpeed(pwr), -127, 127);
  if (speed >= 0) sendRoboClaw(port, addr, 0x00, (uint8_t)speed);
  else sendRoboClaw(port, addr, 0x01, (uint8_t)(-speed));
}

void writeRoboClawM2(HardwareSerial &port, uint8_t addr, float pwr) {
  int speed = constrain(toRoboSpeed(pwr), -127, 127);
  if (speed >= 0) sendRoboClaw(port, addr, 0x04, (uint8_t)speed);
  else sendRoboClaw(port, addr, 0x05, (uint8_t)(-speed));
}

bool writeLinkCommand(int txPin, int rxPin, const char *channel, float value) {
  value = constrain(value, -1.0f, 1.0f);
  if (txPin == 18 && rxPin == 19) {
    if (strcmp(channel, "M1") == 0) { piLink_18_19_M1 = value; return true; }
    if (strcmp(channel, "M2") == 0) { piLink_18_19_M2 = value; return true; }
  }
  if (txPin == 14 && rxPin == 15) {
    if (strcmp(channel, "M1") == 0) { piLink_14_15_M1 = value; return true; }
    if (strcmp(channel, "M2") == 0) { piLink_14_15_M2 = value; return true; }
  }
  return false;
}

void applyPiLinkOutputs() {
  writeRoboClawM1(ROBOCLAW_A_SERIAL, ROBOCLAW_ADDR_A, piLink_18_19_M1);
  writeRoboClawM2(ROBOCLAW_A_SERIAL, ROBOCLAW_ADDR_A, piLink_18_19_M2);
  writeRoboClawM1(ROBOCLAW_B_SERIAL, ROBOCLAW_ADDR_B, piLink_14_15_M1);
  writeRoboClawM2(ROBOCLAW_B_SERIAL, ROBOCLAW_ADDR_B, piLink_14_15_M2);
}

void stopLinks() {
  piLink_18_19_M1 = 0.0f; piLink_18_19_M2 = 0.0f; piLink_14_15_M1 = 0.0f; piLink_14_15_M2 = 0.0f;
  applyPiLinkOutputs();
}

void writeHBridge(uint8_t ina, uint8_t inb, uint8_t enDiag, uint8_t pwmPin, float value) {
  value = constrain(value, -1.0f, 1.0f);
  int pwm = (int)(fabs(value) * 255.0f);
  if (value > 0.001f) { digitalWrite(ina, HIGH); digitalWrite(inb, LOW); }
  else if (value < -0.001f) { digitalWrite(ina, LOW); digitalWrite(inb, HIGH); }
  else { digitalWrite(ina, LOW); digitalWrite(inb, LOW); pwm = 0; }
  digitalWrite(enDiag, HIGH);
  analogWrite(pwmPin, pwm);
}

void ensureServoAttached(uint8_t pin) {
  if (pin >= 70 || directServoAttached[pin]) return;
  if (pin == PIN_GRIP_LEFT || pin == PIN_GRIP_RIGHT || pin == PIN_LIFT) {
    directServoAttached[pin] = true;
    return;
  }
  directServoObjects[pin].attach(pin);
  directServoAttached[pin] = true;
}

void writeGenericServoUs(uint8_t pin, uint16_t us) {
  if (pin == PIN_LIFT) { liftServo.writeMicroseconds(us); return; }
  ensureServoAttached(pin);
  if (pin >= 70) return;
  directServoObjects[pin].writeMicroseconds(us);
}

void writeGenericServo(uint8_t pin, float value) {
  value = constrain(value, -1.0f, 1.0f);
  if (pin == PIN_LIFT) {
    uint16_t liftUs = (uint16_t)map((int)(value * 1000.0f), -1000, 1000, LIFT_DOWN_US, LIFT_UP_US);
    liftServo.writeMicroseconds(liftUs);
    return;
  }
  ensureServoAttached(pin);
  if (pin >= 70) return;
  uint16_t us = (uint16_t)map((int)(value * 1000.0f), -1000, 1000, 1000, 2000);
  directServoObjects[pin].writeMicroseconds(us);
}

void setGripPosition(uint16_t gripUs) {
  gripUs = constrain(gripUs, 1000, 2000);
  int leftUs  = map(gripUs, 1000, 2000, LEFT_OPEN_US,  LEFT_CLOSED_US);
  int rightUs = map(gripUs, 1000, 2000, RIGHT_OPEN_US, RIGHT_CLOSED_US);
  gripLeftServo.writeMicroseconds(leftUs);
  gripRightServo.writeMicroseconds(rightUs);
  piGripLeftUs = leftUs;
  piGripRightUs = rightUs;
}

void setGripUs(uint16_t leftUs, uint16_t rightUs) {
  leftUs = constrain(leftUs, 500, 2500);
  rightUs = constrain(rightUs, 500, 2500);
  gripLeftServo.writeMicroseconds(leftUs);
  gripRightServo.writeMicroseconds(rightUs);
  piGripLeftUs = leftUs;
  piGripRightUs = rightUs;
}

void updateGripFromIbus() {
  const uint8_t idx = CH_GRIP - 1;
  const uint16_t gripUs = ibusMicros(idx);
  setGripPosition(gripUs);
}

void setLiftNormalized(float pos) {
  pos = constrain(pos, -1.0f, 1.0f);
  uint16_t liftUs = (uint16_t)map((int)(pos * 1000.0f), -1000, 1000, LIFT_DOWN_US, LIFT_UP_US);
  liftServo.writeMicroseconds(liftUs);
}

int readDigitalPin(uint8_t pin) { return digitalRead(pin); }
long readAnalogSource(const char *name) {
  if (name[0] == 'A' || name[0] == 'a') return analogRead(atoi(name + 1));
  return analogRead(atoi(name));
}
long readQuadPair(uint8_t pinA, uint8_t pinB) { return ((digitalRead(pinA) ? 1 : 0) << 1) | (digitalRead(pinB) ? 1 : 0); }
long readRangePair(uint8_t trigPin, uint8_t echoPin) {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration <= 0) return -1;
  return (long)(duration * 0.1715f);
}

bool piHeartbeatFresh() { return (millis() - piLastHeartbeatMs) <= PI_HEARTBEAT_TIMEOUT_MS; }
bool piHasControl() { return piAutoRequested && piHeartbeatFresh(); }

void replyValue(const char *kind, long value) {
  PI_SERIAL.print("OK ");
  PI_SERIAL.print(kind);
  PI_SERIAL.print(" ");
  PI_SERIAL.println(value);
}

void handlePiCommand(char *line) {
  while (*line == ' ') line++;

  if (strcmp(line, "HELLO") == 0) { PI_SERIAL.print("ID "); PI_SERIAL.println(DEVICE_ID); return; }
  if (strcmp(line, "MODE AUTO") == 0) { piAutoRequested = true; piLastHeartbeatMs = millis(); PI_SERIAL.println("OK MODE AUTO"); return; }
  if (strcmp(line, "MODE TELEOP") == 0) { piAutoRequested = false; PI_SERIAL.println("OK MODE TELEOP"); return; }
  if (strcmp(line, "STOP") == 0) {
    stopLinks();
    piLiftCmd = 0.0f;
    setGripPosition(1000);
    setLiftNormalized(piLiftCmd);
    writeHBridge(PIN_SHOOTER_INA, PIN_SHOOTER_INB, PIN_SHOOTER_EN_DIAG, PIN_SHOOTER_PWM, 0.0f);
    writeHBridge(PIN_COLLECTOR_INA, PIN_COLLECTOR_INB, PIN_COLLECTOR_EN_DIAG, PIN_COLLECTOR_PWM, 0.0f);
    PI_SERIAL.println("OK STOP");
    return;
  }

  unsigned long hbSeq = 0;
  if (sscanf(line, "HB %lu", &hbSeq) == 1) { piLastHeartbeatMs = millis(); PI_SERIAL.print("OK HB "); PI_SERIAL.println(hbSeq); return; }

  if (strncmp(line, "LINK ", 5) == 0) {
    char *p = line + 5;
    char *tokTx = strtok(p, " ");
    char *tokRx = strtok(nullptr, " ");
    char *tokCh = strtok(nullptr, " ");
    char *tokVal = strtok(nullptr, " ");
    if (tokTx && tokRx && tokCh && tokVal) {
      int txPin = atoi(tokTx), rxPin = atoi(tokRx);
      float value = atof(tokVal);
      if (writeLinkCommand(txPin, rxPin, tokCh, value)) {
        PI_SERIAL.print("OK LINK "); PI_SERIAL.print(txPin); PI_SERIAL.print(" "); PI_SERIAL.print(rxPin); PI_SERIAL.print(" "); PI_SERIAL.println(tokCh);
      } else {
        PI_SERIAL.print("ERR LINK "); PI_SERIAL.print(txPin); PI_SERIAL.print(" "); PI_SERIAL.print(rxPin); PI_SERIAL.print(" "); PI_SERIAL.println(tokCh);
      }
      return;
    }
  }

  if (strncmp(line, "GROUP_US_WRITE ", 15) == 0) {
    char *p = line + 15;
    char *tokPin1 = strtok(p, " ");
    char *tokUs1  = strtok(nullptr, " ");
    char *tokPin2 = strtok(nullptr, " ");
    char *tokUs2  = strtok(nullptr, " ");
    if (tokPin1 && tokUs1 && tokPin2 && tokUs2) {
      int pin1 = atoi(tokPin1), us1 = atoi(tokUs1), pin2 = atoi(tokPin2), us2 = atoi(tokUs2);
      if (us1 < 500) us1 = 500; if (us1 > 2500) us1 = 2500;
      if (us2 < 500) us2 = 500; if (us2 > 2500) us2 = 2500;
      if (pin1 == PIN_GRIP_LEFT && pin2 == PIN_GRIP_RIGHT) {
        setGripUs((uint16_t)us1, (uint16_t)us2);
        PI_SERIAL.print("OK GROUP_US_WRITE GRIP "); PI_SERIAL.print(pin1); PI_SERIAL.print(" "); PI_SERIAL.println(pin2);
      } else {
        writeGenericServoUs((uint8_t)pin1, (uint16_t)us1);
        writeGenericServoUs((uint8_t)pin2, (uint16_t)us2);
        PI_SERIAL.print("OK GROUP_US_WRITE GENERIC "); PI_SERIAL.print(pin1); PI_SERIAL.print(" "); PI_SERIAL.println(pin2);
      }
      return;
    }
  }

  if (strncmp(line, "SERVO_WRITE ", 12) == 0) {
    char *p = line + 12;
    char *tokPin = strtok(p, " ");
    char *tokVal = strtok(nullptr, " ");
    if (tokPin && tokVal) {
      int pin = atoi(tokPin);
      float value = atof(tokVal);
      if (pin == PIN_LIFT) {
        piLiftCmd = constrain(value, -1.0f, 1.0f);
        setLiftNormalized(piLiftCmd);
      } else {
        writeGenericServo((uint8_t)pin, value);
      }
      PI_SERIAL.print("OK SERVO_WRITE "); PI_SERIAL.println(pin);
      return;
    }
  }

  if (strncmp(line, "HBRIDGE_WRITE ", 14) == 0) {
    char *p = line + 14;
    char *tokIna = strtok(p, " ");
    char *tokInb = strtok(nullptr, " ");
    char *tokEn  = strtok(nullptr, " ");
    char *tokPwm = strtok(nullptr, " ");
    char *tokVal = strtok(nullptr, " ");
    if (tokIna && tokInb && tokEn && tokPwm && tokVal) {
      int ina = atoi(tokIna), inb = atoi(tokInb), enDiag = atoi(tokEn), pwm = atoi(tokPwm);
      float value = atof(tokVal);
      writeHBridge((uint8_t)ina, (uint8_t)inb, (uint8_t)enDiag, (uint8_t)pwm, value);
      PI_SERIAL.print("OK HBRIDGE_WRITE "); PI_SERIAL.print(ina); PI_SERIAL.print(" "); PI_SERIAL.print(inb); PI_SERIAL.print(" "); PI_SERIAL.print(enDiag); PI_SERIAL.print(" "); PI_SERIAL.println(pwm);
      return;
    }
  }

  char rkind[16], a1[32], a2[32];
  int count = sscanf(line, "READ %15s %31s %31s", rkind, a1, a2);
  if (count >= 2) {
    if (strcmp(rkind, "DI") == 0) { replyValue("DI", readDigitalPin((uint8_t)atoi(a1))); return; }
    if (strcmp(rkind, "AI") == 0) { replyValue("AI", readAnalogSource(a1)); return; }
    if (strcmp(rkind, "BATTERY") == 0 && strcmp(a1, "voltage") == 0) { replyValue("BATTERY", readAnalogSource(BATTERY_VOLTAGE_PIN)); return; }
    if (strcmp(rkind, "LIMIT") == 0) {
      if (strcmp(a1, "lift_high") == 0) { replyValue("LIMIT", readDigitalPin(PIN_LIFT_LIMIT_HIGH)); return; }
      if (strcmp(a1, "lift_low") == 0) { replyValue("LIMIT", readDigitalPin(PIN_LIFT_LIMIT_LOW)); return; }
    }
    if (strcmp(rkind, "QUAD") == 0 && count >= 3) { replyValue("QUAD", readQuadPair((uint8_t)atoi(a1), (uint8_t)atoi(a2))); return; }
    if (strcmp(rkind, "RANGE") == 0 && count >= 3) { replyValue("RANGE", readRangePair((uint8_t)atoi(a1), (uint8_t)atoi(a2))); return; }
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
      if (piLineIdx > 0) handlePiCommand(piLineBuf);
      piLineIdx = 0;
      continue;
    }
    if (piLineIdx < sizeof(piLineBuf) - 1) piLineBuf[piLineIdx++] = c;
    else piLineIdx = 0;
  }
}

void setup() {
  PI_SERIAL.begin(115200);
  IBUS_SERIAL.begin(115200, SERIAL_8N2);
  ROBOCLAW_A_SERIAL.begin(38400);
  ROBOCLAW_B_SERIAL.begin(38400);

  memset(directServoAttached, 0, sizeof(directServoAttached));

  pinMode(PIN_SHOOTER_PWM, OUTPUT);
  pinMode(PIN_COLLECTOR_PWM, OUTPUT);
  pinMode(PIN_SHOOTER_INA, OUTPUT);
  pinMode(PIN_SHOOTER_INB, OUTPUT);
  pinMode(PIN_SHOOTER_EN_DIAG, OUTPUT);
  pinMode(PIN_COLLECTOR_INA, OUTPUT);
  pinMode(PIN_COLLECTOR_INB, OUTPUT);
  pinMode(PIN_COLLECTOR_EN_DIAG, OUTPUT);

  pinMode(PIN_LIFT_LIMIT_HIGH, INPUT_PULLUP);
  pinMode(PIN_LIFT_LIMIT_LOW, INPUT_PULLUP);
  pinMode(PIN_DEADWHEEL_PARALLEL_A, INPUT_PULLUP);
  pinMode(PIN_DEADWHEEL_PARALLEL_B, INPUT_PULLUP);
  pinMode(PIN_DEADWHEEL_PERPENDICULAR_A, INPUT_PULLUP);
  pinMode(PIN_DEADWHEEL_PERPENDICULAR_B, INPUT_PULLUP);
  pinMode(PIN_SHOOTER_ENC_A, INPUT_PULLUP);
  pinMode(PIN_SHOOTER_ENC_B, INPUT_PULLUP);

  gripLeftServo.attach(PIN_GRIP_LEFT);
  gripRightServo.attach(PIN_GRIP_RIGHT);
  liftServo.attach(PIN_LIFT);

  directServoAttached[PIN_GRIP_LEFT] = true;
  directServoAttached[PIN_GRIP_RIGHT] = true;
  directServoAttached[PIN_LIFT] = true;

  setGripPosition(1000);
  setLiftNormalized(0.0f);

  stopLinks();
  writeHBridge(PIN_SHOOTER_INA, PIN_SHOOTER_INB, PIN_SHOOTER_EN_DIAG, PIN_SHOOTER_PWM, 0.0f);
  writeHBridge(PIN_COLLECTOR_INA, PIN_COLLECTOR_INB, PIN_COLLECTOR_EN_DIAG, PIN_COLLECTOR_PWM, 0.0f);

  PI_SERIAL.print("BOOT ");
  PI_SERIAL.println(DEVICE_ID);
}

void loop() {
  servicePiSerial();
  readIbusFrame();

  if (piHasControl()) {
    applyPiLinkOutputs();
    gripLeftServo.writeMicroseconds(piGripLeftUs);
    gripRightServo.writeMicroseconds(piGripRightUs);
    setLiftNormalized(piLiftCmd);
    delay(20);
    return;
  }

  if (piAutoRequested && !piHeartbeatFresh()) {
    piAutoRequested = false;
    stopLinks();
    gripLeftServo.writeMicroseconds(piGripLeftUs);
    gripRightServo.writeMicroseconds(piGripRightUs);
    setLiftNormalized(piLiftCmd);
    writeHBridge(PIN_SHOOTER_INA, PIN_SHOOTER_INB, PIN_SHOOTER_EN_DIAG, PIN_SHOOTER_PWM, 0.0f);
    writeHBridge(PIN_COLLECTOR_INA, PIN_COLLECTOR_INB, PIN_COLLECTOR_EN_DIAG, PIN_COLLECTOR_PWM, 0.0f);
  }

  if (millis() - ibus_last_frame_ms > 200) {
    stopLinks();
    return;
  }

  int throttle = ibusToPercent(CH_DRIVE_THROTTLE - 1);
  int rotate   = ibusToPercent(CH_DRIVE_ROTATE - 1);
  float fwd  = normalize(throttle);
  float turn = normalize(rotate);
  fwd *= 0.38f;
  turn *= 0.45f;

  float left  = fwd + turn;
  float right = fwd - turn;
  float maxVal = max(fabs(left), fabs(right));
  if (maxVal > 1.0f) { left /= maxVal; right /= maxVal; }

  writeRoboClawM1(ROBOCLAW_A_SERIAL, ROBOCLAW_ADDR_A, left);
  writeRoboClawM2(ROBOCLAW_A_SERIAL, ROBOCLAW_ADDR_A, right);
  updateGripFromIbus();
  delay(20);
}
