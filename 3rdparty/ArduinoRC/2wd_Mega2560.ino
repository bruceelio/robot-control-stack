// 3rdparty/ArduinoRC/2wd_Mega2560.ino
//
// (9) added lift
// BobBot / Mega reusable control sketch
//
// Design intent:
// - keep ALL important wiring assignments in one block at the top
// - keep named software points mapped in one place
// - allow the same sketch to be reused across projects by editing constants,
//   not by searching through the whole file
//
// Current active functions:
// - FlySky iBus teleop drive on CH1/CH2
// - FlySky gripper control on CH5
// - Pi USB serial AUTO mode with heartbeat
// - named motor endpoints:
//     drive_front_left
//     drive_front_right
//     shooter
//     collector
// - named servo endpoints:
//     gripper
//     lift
// - read helpers:
//     DI / AI / LIMIT / QUAD / BATTERY
//
// Notes:
// - RoboClaw A (pins 18/19) is currently the active drive link
// - drive_front_left  -> RoboClaw A M1
// - drive_front_right -> RoboClaw A M2
// - RoboClaw B (pins 14/15) is wired but not currently used in this sketch
// - gripper is a paired mirrored servo group on pins 11 and 13
// - lift is a single servo on pin 12
// - shooter / collector use Cytron MDD20A wiring defined below

#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// =========================================================
// PIN / LINK ASSIGNMENT BLOCK
// Edit this block first when reusing the sketch.
// =========================================================

// ------------------------- USB / SERIAL -------------------
#define PI_SERIAL Serial          // USB serial to Raspberry Pi
#define ROBOCLAW_B_SERIAL Serial3 // pins 14/15
#define IBUS_SERIAL Serial2       // pins 16/17
#define ROBOCLAW_A_SERIAL Serial1 // pins 18/19

static const uint8_t PIN_USB_RX0 = 0;   // reserved internal USB serial
static const uint8_t PIN_USB_TX0 = 1;   // reserved internal USB serial

static const uint8_t PIN_RC_IBUS_TX = 16; // reserved (FlySky iBus TX)
static const uint8_t PIN_RC_IBUS_RX = 17; // active iBus input

static const uint8_t PIN_ROBOCLAW_B_TX = 14;
static const uint8_t PIN_ROBOCLAW_B_RX = 15;

static const uint8_t PIN_ROBOCLAW_A_TX = 18;
static const uint8_t PIN_ROBOCLAW_A_RX = 19;

// ------------------------- DIRECT PWM / SERVO ------------
static const uint8_t PIN_SHOOTER_PWM       = 4;
static const uint8_t PIN_COLLECTOR_PWM     = 5;
static const uint8_t PIN_SPARE_PWM_6       = 6;
static const uint8_t PIN_SPARE_PWM_7       = 7;
static const uint8_t PIN_SPARE_PWM_8       = 8;

static const uint8_t PIN_SHOOTER_FEED_LEFT  = 9;
static const uint8_t PIN_SHOOTER_FEED_RIGHT = 10;

static const uint8_t PIN_GRIP_LEFT  = 11;
static const uint8_t PIN_LIFT       = 12;
static const uint8_t PIN_GRIP_RIGHT = 13;

// ------------------------- I2C ----------------------------
static const uint8_t PIN_I2C_SDA = 20;
static const uint8_t PIN_I2C_SCL = 21;

// ------------------------- QUADRATURE / LIMITS -----------
static const uint8_t PIN_DEADWHEEL_PARALLEL_A      = 23;
static const uint8_t PIN_DEADWHEEL_PARALLEL_B      = 25;
static const uint8_t PIN_DEADWHEEL_PERPENDICULAR_A = 27;
static const uint8_t PIN_DEADWHEEL_PERPENDICULAR_B = 29;

static const uint8_t PIN_LIFT_LIMIT_HIGH = 31;
static const uint8_t PIN_LIFT_LIMIT_LOW  = 33;

static const uint8_t PIN_SHOOTER_ENC_A = 35;
static const uint8_t PIN_SHOOTER_ENC_B = 37;

// ------------------------- CYTRON MDD20A ------------------
static const uint8_t PIN_SHOOTER_INA     = 39;
static const uint8_t PIN_SHOOTER_INB     = 41;
static const uint8_t PIN_SHOOTER_EN_DIAG = 43;

static const uint8_t PIN_COLLECTOR_INA     = 45;
static const uint8_t PIN_COLLECTOR_INB     = 47;
static const uint8_t PIN_COLLECTOR_EN_DIAG = 49;

// ------------------------- LOGICAL LINK MAPPING ----------
#define ROBOCLAW_ADDR_A 0x80
#define ROBOCLAW_ADDR_B 0x80

// Current logical mapping:
static const char MOTOR_NAME_DRIVE_FRONT_LEFT[]  = "drive_front_left";
static const char MOTOR_NAME_DRIVE_FRONT_RIGHT[] = "drive_front_right";
static const char MOTOR_NAME_DRIVE_REAR_LEFT[]   = "drive_rear_left";
static const char MOTOR_NAME_DRIVE_REAR_RIGHT[]  = "drive_rear_right";
static const char MOTOR_NAME_SHOOTER[]           = "shooter";
static const char MOTOR_NAME_COLLECTOR[]         = "collector";

static const char SERVO_NAME_GRIPPER[]           = "gripper";
static const char SERVO_NAME_LIFT[]              = "lift";

// ------------------------- FLYSKY CHANNELS ---------------
static const uint8_t CH_DRIVE_ROTATE = 1; // right stick left/right
static const uint8_t CH_DRIVE_THROTTLE = 2; // right stick up/down
static const uint8_t CH_GRIP = 5; // currently assigned gripper control
static const uint8_t CH_LIFT = 6; // knob to the right of gripper knob

// ------------------------- SERVO CALIBRATION -------------
static const int GRIP_LEFT_OPEN_US      = 900;
static const int GRIP_LEFT_CLOSED_US    = 2200;
static const int GRIP_RIGHT_OPEN_US     = 2100;
static const int GRIP_RIGHT_CLOSED_US   = 800;

static const int LIFT_DOWN_US           = 800;
static const int LIFT_UP_US             = 2250;

// ------------------------- BATTERY INPUT -----------------
static const char BATTERY_VOLTAGE_PIN[] = "A0"; // update if/when battery source changes

// ------------------------- SYSTEM ------------------------
static const char DEVICE_ID[] = "MEGA_AUX_1";
static const unsigned long PI_HEARTBEAT_TIMEOUT_MS = 86400000UL; // 24 hours; (500 ms)

// =========================================================
// STATE
// =========================================================

bool piAutoRequested = false;
unsigned long piLastHeartbeatMs = 0;

// Named outputs controlled by Pi AUTO mode
float piDriveFrontLeftCmd  = 0.0f;
float piDriveFrontRightCmd = 0.0f;
float piShooterCmd         = 0.0f;
float piCollectorCmd       = 0.0f;
float piGripCmd            = -1.0f;   // -1=open, +1=closed
float piLiftCmd            = 0.0f;    // -1=down, +1=up

char piLineBuf[128];
uint8_t piLineIdx = 0;

// ------------------------- iBus --------------------------
static const uint8_t IBUS_FRAME_LEN = 32;
uint8_t ibus_buf[IBUS_FRAME_LEN];
uint8_t ibus_idx = 0;
uint16_t ibus_ch[14] = {1500};
unsigned long ibus_last_frame_ms = 0;

// ------------------------- servos ------------------------
Servo gripLeftServo;
Servo gripRightServo;
Servo liftServo;

// =========================================================
// IBUS
// =========================================================

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
  const int dead_min = 45;
  const int dead_max = 55;

  if (val >= dead_min && val <= dead_max) return 0.0f;
  if (val < dead_min) return (float)(val - dead_min) / (float)dead_min;
  return (float)(val - dead_max) / (float)(100 - dead_max);
}

// =========================================================
// ROBOCLAW
// =========================================================

uint16_t crc_update(uint16_t crc, uint8_t data) {
  crc ^= (uint16_t)data << 8;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
    else crc <<= 1;
  }
  return crc;
}

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

// Drive mapping:
//   drive_front_left  -> RoboClaw A M1, Serial1 pins 18/19
//   drive_front_right -> RoboClaw A M2, Serial1 pins 18/19
//   drive_rear_left   -> RoboClaw B M1, Serial3 pins 14/15
//   drive_rear_right  -> RoboClaw B M2, Serial3 pins 14/15

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

void writeDriveFrontLeft(float pwr) {
  writeRoboClawM1(ROBOCLAW_A_SERIAL, ROBOCLAW_ADDR_A, pwr);
}

void writeDriveFrontRight(float pwr) {
  writeRoboClawM2(ROBOCLAW_A_SERIAL, ROBOCLAW_ADDR_A, pwr);
}

void writeDriveRearLeft(float pwr) {
  writeRoboClawM1(ROBOCLAW_B_SERIAL, ROBOCLAW_ADDR_B, pwr);
}

void writeDriveRearRight(float pwr) {
  writeRoboClawM2(ROBOCLAW_B_SERIAL, ROBOCLAW_ADDR_B, pwr);
}

void stopDrive() {
  writeDriveFrontLeft(0.0f);
  writeDriveFrontRight(0.0f);
  writeDriveRearLeft(0.0f);
  writeDriveRearRight(0.0f);
}

// =========================================================
// DIRECT MOTOR DRIVER (CYTRON MDD20A)
// =========================================================

void writeHBridge(uint8_t ina, uint8_t inb, uint8_t enDiag, uint8_t pwmPin, float value) {
  value = constrain(value, -1.0f, 1.0f);
  int pwm = (int)(fabs(value) * 255.0f);

  // Simple direction convention:
  //   +value => INA high, INB low
  //   -value => INA low,  INB high
  //   0      => coast
  if (value > 0.001f) {
    digitalWrite(ina, HIGH);
    digitalWrite(inb, LOW);
  } else if (value < -0.001f) {
    digitalWrite(ina, LOW);
    digitalWrite(inb, HIGH);
  } else {
    digitalWrite(ina, LOW);
    digitalWrite(inb, LOW);
    pwm = 0;
  }

  // EN/DIAG is treated as an enable line here.
  digitalWrite(enDiag, HIGH);
  analogWrite(pwmPin, pwm);
}

void writeShooterMotor(float value) {
  writeHBridge(PIN_SHOOTER_INA, PIN_SHOOTER_INB, PIN_SHOOTER_EN_DIAG, PIN_SHOOTER_PWM, value);
}

void writeCollectorMotor(float value) {
  writeHBridge(PIN_COLLECTOR_INA, PIN_COLLECTOR_INB, PIN_COLLECTOR_EN_DIAG, PIN_COLLECTOR_PWM, value);
}

// =========================================================
// SERVOS
// =========================================================

void setGripPositionUs(uint16_t gripUs) {
  gripUs = constrain(gripUs, 1000, 2000);

  int leftUs  = map(gripUs, 1000, 2000, GRIP_LEFT_OPEN_US,  GRIP_LEFT_CLOSED_US);
  int rightUs = map(gripUs, 1000, 2000, GRIP_RIGHT_OPEN_US, GRIP_RIGHT_CLOSED_US);

  gripLeftServo.writeMicroseconds(leftUs);
  gripRightServo.writeMicroseconds(rightUs);
}

void setGripNormalized(float pos) {
  pos = constrain(pos, -1.0f, 1.0f);
  uint16_t gripUs = (uint16_t)map((int)(pos * 1000.0f), -1000, 1000, 1000, 2000);
  setGripPositionUs(gripUs);
}

void setLiftNormalized(float pos) {
  pos = constrain(pos, -1.0f, 1.0f);
  uint16_t liftUs = (uint16_t)map((int)(pos * 1000.0f), -1000, 1000, LIFT_DOWN_US, LIFT_UP_US);
  liftServo.writeMicroseconds(liftUs);
}

void updateGripFromIbus() {
  const uint8_t idx = CH_GRIP - 1;
  const uint16_t gripUs = ibusMicros(idx);
  setGripPositionUs(gripUs);
}

void updateLiftFromIbus() {
  const uint8_t idx = CH_LIFT - 1;
  const uint16_t liftUs = ibusMicros(idx);

  // Map knob range directly to normalized lift command:
  // 1000us -> -1.0 (down)
  // 1500us ->  0.0 (mid)
  // 2000us -> +1.0 (up)
  float pos = (float)map(liftUs, 1000, 2000, -1000, 1000) / 1000.0f;

  setLiftNormalized(pos);
}

// =========================================================
// READ HELPERS
// =========================================================

int readDigitalPin(uint8_t pin) {
  return digitalRead(pin);
}

long readAnalogSource(const char *name) {
  // Current implementation supports A0..A15 or numeric strings.
  if (name[0] == 'A' || name[0] == 'a') {
    int idx = atoi(name + 1);
    return analogRead(idx);
  }

  int pin = atoi(name);
  return analogRead(pin);
}

long readQuadPair(uint8_t pinA, uint8_t pinB) {
  // Placeholder/simple snapshot. Replace with proper counter logic when encoder
  // accumulation is added.
  int a = digitalRead(pinA) ? 1 : 0;
  int b = digitalRead(pinB) ? 1 : 0;
  return (a << 1) | b;
}

long readRangePair(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000UL); // timeout ~30ms
  if (duration <= 0) return -1;

  // Distance in mm (approx): duration_us * 0.1715
  long distanceMm = (long)(duration * 0.1715f);
  return distanceMm;
}

// =========================================================
// PI SERIAL CONTROL
// =========================================================

bool piHeartbeatFresh() {
  return (millis() - piLastHeartbeatMs) <= PI_HEARTBEAT_TIMEOUT_MS;
}

bool piHasControl() {
  return piAutoRequested && piHeartbeatFresh();
}

bool setMotorByName(const char *name, float value) {
  value = constrain(value, -1.0f, 1.0f);

  if (strcmp(name, MOTOR_NAME_DRIVE_FRONT_LEFT) == 0) {
    writeDriveFrontLeft(value);
    return true;
  }

  if (strcmp(name, MOTOR_NAME_DRIVE_FRONT_RIGHT) == 0) {
    writeDriveFrontRight(value);
    return true;
  }

  if (strcmp(name, MOTOR_NAME_DRIVE_REAR_LEFT) == 0) {
    writeDriveRearLeft(value);
    return true;
  }

  if (strcmp(name, MOTOR_NAME_DRIVE_REAR_RIGHT) == 0) {
    writeDriveRearRight(value);
    return true;
  }

  if (strcmp(name, MOTOR_NAME_SHOOTER) == 0) {
    writeShooterMotor(value);
    return true;
  }

  if (strcmp(name, MOTOR_NAME_COLLECTOR) == 0) {
    writeCollectorMotor(value);
    return true;
  }

  return false;
}

bool setServoByName(const char *name, float value) {
  value = constrain(value, -1.0f, 1.0f);

  if (strcmp(name, SERVO_NAME_GRIPPER) == 0) {
    piGripCmd = value;
    return true;
  }

  if (strcmp(name, SERVO_NAME_LIFT) == 0) {
    piLiftCmd = value;
    return true;
  }

  return false;
}

void replyValue(const char *kind, long value) {
  PI_SERIAL.print("OK ");
  PI_SERIAL.print(kind);
  PI_SERIAL.print(" ");
  PI_SERIAL.println(value);
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
    piDriveFrontLeftCmd = 0.0f;
    piDriveFrontRightCmd = 0.0f;
    piShooterCmd = 0.0f;
    piCollectorCmd = 0.0f;
    stopDrive();
    writeShooterMotor(0.0f);
    writeCollectorMotor(0.0f);
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

  // ---------------------------------------------------------
  // Hardware-native pin/link protocol (preferred)
  // ---------------------------------------------------------

  if (strncmp(line, "LINK ", 5) == 0) {
    char *p = line + 5;
    char *tokTx  = strtok(p, " ");
    char *tokRx  = strtok(nullptr, " ");
    char *tokCh  = strtok(nullptr, " ");
    char *tokVal = strtok(nullptr, " ");

    if (tokTx && tokRx && tokCh && tokVal) {
      int txPin = atoi(tokTx);
      int rxPin = atoi(tokRx);
      float value = constrain(atof(tokVal), -1.0f, 1.0f);

      bool ok = false;

      // Active drive link: RoboClaw A on 18/19
      if (txPin == PIN_ROBOCLAW_A_TX && rxPin == PIN_ROBOCLAW_A_RX) {
        if (strcmp(tokCh, "M1") == 0) {
          piDriveFrontLeftCmd = value;
          ok = true;
        } else if (strcmp(tokCh, "M2") == 0) {
          piDriveFrontRightCmd = value;
          ok = true;
        }
      }

      if (ok) {
        PI_SERIAL.print("OK LINK ");
        PI_SERIAL.print(txPin);
        PI_SERIAL.print(" ");
        PI_SERIAL.print(rxPin);
        PI_SERIAL.print(" ");
        PI_SERIAL.println(tokCh);
      } else {
        PI_SERIAL.print("ERR LINK ");
        PI_SERIAL.print(txPin);
        PI_SERIAL.print(" ");
        PI_SERIAL.print(rxPin);
        PI_SERIAL.print(" ");
        PI_SERIAL.println(tokCh);
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
      float value = constrain(atof(tokVal), -1.0f, 1.0f);

      bool ok = true;

      if (pin == PIN_LIFT) {
        piLiftCmd = value;
      } else if (pin == PIN_GRIP_LEFT || pin == PIN_GRIP_RIGHT) {
        // One logical gripper command; Mega mirrors internally.
        piGripCmd = value;
      } else {
        ok = false;
      }

      if (ok) {
        PI_SERIAL.print("OK SERVO_WRITE ");
        PI_SERIAL.println(pin);
      } else {
        PI_SERIAL.print("ERR SERVO_WRITE ");
        PI_SERIAL.println(pin);
      }
      return;
    }
  }

  if (strncmp(line, "GROUP_WRITE ", 12) == 0) {
    char *p = line + 12;
    char *tokPin1 = strtok(p, " ");
    char *tokVal1 = strtok(nullptr, " ");
    char *tokPin2 = strtok(nullptr, " ");
    char *tokVal2 = strtok(nullptr, " ");

    if (tokPin1 && tokVal1 && tokPin2 && tokVal2) {
      int pin1 = atoi(tokPin1);
      float val1 = constrain(atof(tokVal1), -1.0f, 1.0f);
      int pin2 = atoi(tokPin2);
      float val2 = constrain(atof(tokVal2), -1.0f, 1.0f);

      bool ok = true;

      if ((pin1 == PIN_GRIP_LEFT && pin2 == PIN_GRIP_RIGHT) ||
          (pin1 == PIN_GRIP_RIGHT && pin2 == PIN_GRIP_LEFT)) {
        // Prefer one logical gripper command. Use the first value.
        (void)val2;
        piGripCmd = val1;
      } else {
        ok = false;
      }

      if (ok) {
        PI_SERIAL.print("OK GROUP_WRITE ");
        PI_SERIAL.print(pin1);
        PI_SERIAL.print(" ");
        PI_SERIAL.println(pin2);
      } else {
        PI_SERIAL.print("ERR GROUP_WRITE ");
        PI_SERIAL.print(pin1);
        PI_SERIAL.print(" ");
        PI_SERIAL.println(pin2);
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
      int pin1 = atoi(tokPin1);
      int us1  = atoi(tokUs1);
      int pin2 = atoi(tokPin2);
      int us2  = atoi(tokUs2);

      bool ok = true;

      if ((pin1 == PIN_GRIP_LEFT && pin2 == PIN_GRIP_RIGHT) ||
          (pin1 == PIN_GRIP_RIGHT && pin2 == PIN_GRIP_LEFT)) {
        us1 = constrain(us1, 500, 2500);
        us2 = constrain(us2, 500, 2500);
        gripLeftServo.writeMicroseconds(us1);
        gripRightServo.writeMicroseconds(us2);
      } else if (pin1 == PIN_LIFT && pin2 == PIN_LIFT) {
        us1 = constrain(us1, 500, 2500);
        liftServo.writeMicroseconds(us1);
      } else {
        ok = false;
      }

      if (ok) {
        PI_SERIAL.print("OK GROUP_US_WRITE ");
        PI_SERIAL.print(pin1);
        PI_SERIAL.print(" ");
        PI_SERIAL.println(pin2);
      } else {
        PI_SERIAL.print("ERR GROUP_US_WRITE ");
        PI_SERIAL.print(pin1);
        PI_SERIAL.print(" ");
        PI_SERIAL.println(pin2);
      }
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
      int ina = atoi(tokIna);
      int inb = atoi(tokInb);
      int enDiag = atoi(tokEn);
      int pwm = atoi(tokPwm);
      float value = constrain(atof(tokVal), -1.0f, 1.0f);

      bool ok = true;

      if (ina == PIN_SHOOTER_INA && inb == PIN_SHOOTER_INB &&
          enDiag == PIN_SHOOTER_EN_DIAG && pwm == PIN_SHOOTER_PWM) {
        piShooterCmd = value;
      } else if (ina == PIN_COLLECTOR_INA && inb == PIN_COLLECTOR_INB &&
                 enDiag == PIN_COLLECTOR_EN_DIAG && pwm == PIN_COLLECTOR_PWM) {
        piCollectorCmd = value;
      } else {
        // Fall back to direct hardware write for unknown combinations.
        writeHBridge((uint8_t)ina, (uint8_t)inb, (uint8_t)enDiag, (uint8_t)pwm, value);
      }

      PI_SERIAL.print("OK HBRIDGE_WRITE ");
      PI_SERIAL.print(ina);
      PI_SERIAL.print(" ");
      PI_SERIAL.print(inb);
      PI_SERIAL.print(" ");
      PI_SERIAL.print(enDiag);
      PI_SERIAL.print(" ");
      PI_SERIAL.println(pwm);
      return;
    }
  }

  // ---------------------------------------------------------
  // Semantic protocol (kept for compatibility)
  // ---------------------------------------------------------

  char kind[16];
  char name[32];
  float value = 0.0f;

  char motorName[32];
  float motorPower = 0.0f;

    if (sscanf(line, "MOTOR %31s WRITE power=%f", motorName, &motorPower) == 2 ||
        sscanf(line, "MOTOR %31s WRITE %f", motorName, &motorPower) == 2) {
    if (setMotorByName(motorName, motorPower)) {
        PI_SERIAL.print("OK MOTOR ");
        PI_SERIAL.print(motorName);
        PI_SERIAL.print(" power=");
        PI_SERIAL.println(motorPower, 4);
    } else {
        PI_SERIAL.print("ERR MOTOR ");
        PI_SERIAL.println(motorName);
    }
    return;
  }


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

  // READ DI 26
  // READ AI A1
  // READ LIMIT lift_high
  // READ QUAD 22 23
  // READ RANGE 2 3
  char rkind[16];
  char a1[32];
  char a2[32];
  int count = sscanf(line, "READ %15s %31s %31s", rkind, a1, a2);

  if (count >= 2) {
    if (strcmp(rkind, "DI") == 0) {
      replyValue("DI", readDigitalPin((uint8_t)atoi(a1)));
      return;
    }

    if (strcmp(rkind, "AI") == 0) {
      replyValue("AI", readAnalogSource(a1));
      return;
    }

    if (strcmp(rkind, "BATTERY") == 0) {
      if (strcmp(a1, "voltage") == 0) {
        replyValue("BATTERY", readAnalogSource(BATTERY_VOLTAGE_PIN));
        return;
      }
    }

    if (strcmp(rkind, "LIMIT") == 0) {
      if (strcmp(a1, "lift_high") == 0) {
        replyValue("LIMIT", readDigitalPin(PIN_LIFT_LIMIT_HIGH));
        return;
      }
      if (strcmp(a1, "lift_low") == 0) {
        replyValue("LIMIT", readDigitalPin(PIN_LIFT_LIMIT_LOW));
        return;
      }
    }

    if (strcmp(rkind, "QUAD") == 0 && count >= 3) {
      replyValue("QUAD", readQuadPair((uint8_t)atoi(a1), (uint8_t)atoi(a2)));
      return;
    }

    if (strcmp(rkind, "RANGE") == 0 && count >= 3) {
      replyValue("RANGE", readRangePair((uint8_t)atoi(a1), (uint8_t)atoi(a2)));
      return;
    }
  }

  // Backward compatibility during migration
  if (strncmp(line, "DRV ", 4) == 0) {
    char *p = line + 4;
    char *tok1 = strtok(p, " ");
    char *tok2 = strtok(nullptr, " ");

    if (tok1 && tok2) {
      piDriveFrontLeftCmd = constrain(atof(tok1), -1.0f, 1.0f);
      piDriveFrontRightCmd = constrain(atof(tok2), -1.0f, 1.0f);
      PI_SERIAL.println("OK DRV");
      return;
    }
  }

  if (strncmp(line, "GRIP ", 5) == 0) {
    piGripCmd = constrain(atof(line + 5), -1.0f, 1.0f);
    PI_SERIAL.println("OK GRIP");
    return;
  }

  if (strncmp(line, "LIFT ", 5) == 0) {
    piLiftCmd = constrain(atof(line + 5), -1.0f, 1.0f);
    PI_SERIAL.println("OK LIFT");
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
      if (piLineIdx > 0) handlePiCommand(piLineBuf);
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

// =========================================================
// SETUP
// =========================================================

void setup() {
  PI_SERIAL.begin(115200);
  IBUS_SERIAL.begin(115200, SERIAL_8N2);
  ROBOCLAW_A_SERIAL.begin(38400);
  ROBOCLAW_B_SERIAL.begin(38400);

  // Direct outputs
  pinMode(PIN_SHOOTER_PWM, OUTPUT);
  pinMode(PIN_COLLECTOR_PWM, OUTPUT);

  pinMode(PIN_SHOOTER_INA, OUTPUT);
  pinMode(PIN_SHOOTER_INB, OUTPUT);
  pinMode(PIN_SHOOTER_EN_DIAG, OUTPUT);

  pinMode(PIN_COLLECTOR_INA, OUTPUT);
  pinMode(PIN_COLLECTOR_INB, OUTPUT);
  pinMode(PIN_COLLECTOR_EN_DIAG, OUTPUT);

  // Ultrasonic pins are only examples/future use; configure on demand or here.
  pinMode(2, OUTPUT);
  pinMode(3, INPUT);

  // Limits / quadrature inputs
  pinMode(PIN_LIFT_LIMIT_HIGH, INPUT_PULLUP);
  pinMode(PIN_LIFT_LIMIT_LOW, INPUT_PULLUP);

  pinMode(PIN_DEADWHEEL_PARALLEL_A, INPUT_PULLUP);
  pinMode(PIN_DEADWHEEL_PARALLEL_B, INPUT_PULLUP);
  pinMode(PIN_DEADWHEEL_PERPENDICULAR_A, INPUT_PULLUP);
  pinMode(PIN_DEADWHEEL_PERPENDICULAR_B, INPUT_PULLUP);
  pinMode(PIN_SHOOTER_ENC_A, INPUT_PULLUP);
  pinMode(PIN_SHOOTER_ENC_B, INPUT_PULLUP);

  // Servos
  gripLeftServo.attach(PIN_GRIP_LEFT);
  gripRightServo.attach(PIN_GRIP_RIGHT);
  liftServo.attach(PIN_LIFT);

  setGripNormalized(-1.0f); // open
  setLiftNormalized(0.0f);  // neutral / midpoint

  stopDrive();
  writeShooterMotor(0.0f);
  writeCollectorMotor(0.0f);

  PI_SERIAL.print("BOOT ");
  PI_SERIAL.println(DEVICE_ID);
}

// =========================================================
// LOOP
// =========================================================

void loop() {
  servicePiSerial();
  readIbusFrame();

  // Pi AUTO owns outputs while heartbeat is fresh.
  if (piHasControl()) {
    writeDriveFrontLeft(piDriveFrontLeftCmd);
    writeDriveFrontRight(piDriveFrontRightCmd);
    writeShooterMotor(piShooterCmd);
    writeCollectorMotor(piCollectorCmd);
    setGripNormalized(piGripCmd);
    setLiftNormalized(piLiftCmd);
    delay(20);
    return;
  }

  // Drop back to teleop on timeout
  if (piAutoRequested && !piHeartbeatFresh()) {
    piAutoRequested = false;
    stopDrive();
    writeShooterMotor(0.0f);
    writeCollectorMotor(0.0f);
  }

  // Safety: if FlySky signal is lost, stop drive.
  if (millis() - ibus_last_frame_ms > 200) {
    stopDrive();
    return;
  }

  // Differential drive from FlySky
  int throttle = ibusToPercent(CH_DRIVE_THROTTLE - 1);
  int rotate   = ibusToPercent(CH_DRIVE_ROTATE - 1);

  float fwd  = normalize(throttle);
  float turn = normalize(rotate);

  float driveScale = 0.38f;
  float turnScale  = 0.45f;

  fwd  *= driveScale;
  turn *= turnScale;

  float left  = fwd + turn;
  float right = fwd - turn;

  float maxVal = max(fabs(left), fabs(right));
  if (maxVal > 1.0f) {
    left  /= maxVal;
    right /= maxVal;
  }

  writeDriveFrontLeft(left);
  writeDriveFrontRight(right);
  updateGripFromIbus();
  updateLiftFromIbus();

  delay(20);
}
