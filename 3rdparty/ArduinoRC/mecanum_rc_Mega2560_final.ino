// mecanum_rc_Mega2560_final.ino
//
// Standalone FlySky-controlled mecanum robot.
// No Raspberry Pi connection and no autonomous mode.
//
// Based on the tested non-mecanum Mega sketch:
// - FlySky iBus input on Serial2
// - RoboClaw packet serial with CRC16
// - RoboClaw A controls the FRONT motors
// - RoboClaw B controls the REAR motors
// - tested shooter control copied from CH6 behaviour
// - tested shooter-feed control copied unchanged from CH7 behaviour
//
// The only drivetrain difference is mecanum mixing:
//   CH2 = forward/reverse  (right stick up/down)
//   CH1 = strafe           (right stick left/right)
//   CH4 = rotate           (left stick left/right)

#include <Arduino.h>
#include <math.h>
#include <Servo.h>

// =========================================================
// SERIAL LINKS
// =========================================================

#define ROBOCLAW_B_SERIAL Serial3   // pins 14/15
#define ROBOCLAW_A_SERIAL Serial1   // pins 18/19
#define IBUS_SERIAL       Serial2   // FlySky iBus on RX2 pin 17

// Each RoboClaw is on its own serial link, matching the tested sketch.
#define ROBOCLAW_ADDR_A 0x80
#define ROBOCLAW_ADDR_B 0x80

// =========================================================
// HARDWARE MAPPING
// =========================================================

// RoboClaw A: front motors
//   M1 = Front Left
//   M2 = Front Right
//
// RoboClaw B: rear motors
//   M1 = Rear Left
//   M2 = Rear Right

// Shooter motor: Cytron MDD20A PWM/DIR wiring
static const uint8_t PIN_SHOOTER_PWM = 5;
static const uint8_t PIN_SHOOTER_DIR = 39;

// Shooter-feed continuous-rotation servos
static const uint8_t PIN_SERVO_SHOOTER_FEED_LEFT  = 9;
static const uint8_t PIN_SERVO_SHOOTER_FEED_RIGHT = 10;

// =========================================================
// FLYSKY CHANNELS
// =========================================================

static const uint8_t CH_DRIVE_STRAFE   = 1; // right stick left/right
static const uint8_t CH_DRIVE_THROTTLE = 2; // right stick up/down
static const uint8_t CH_DRIVE_ROTATE   = 4; // left stick left/right

// Preserved from the tested non-mecanum sketch
static const uint8_t CH_SHOOTER      = 6;
static const uint8_t CH_SHOOTER_FEED = 7;

// =========================================================
// CONTROL SETTINGS
// =========================================================

static const float DRIVE_SCALE  = 0.38f;
static const float STRAFE_SCALE = 0.38f;
static const float TURN_SCALE   = 0.45f;

// Preserved from the tested non-mecanum sketch
static const float TELEOP_SHOOTER_SCALE = 0.8f;

static const unsigned long SHOOTER_FEED_PULSE_MS = 700;
static const int SHOOTER_FEED_STOP_US      = 1500;
static const int SHOOTER_FEED_LEFT_RUN_US  = 1700;
static const int SHOOTER_FEED_RIGHT_RUN_US = 1300;

static const unsigned long IBUS_TIMEOUT_MS = 200;

// =========================================================
// STATE
// =========================================================

static const uint8_t IBUS_FRAME_LEN = 32;
uint8_t ibus_buf[IBUS_FRAME_LEN];
uint8_t ibus_idx = 0;
uint16_t ibus_ch[14] = {1500};
unsigned long ibus_last_frame_ms = 0;

Servo shooterFeedLeftServo;
Servo shooterFeedRightServo;

bool shooterFeedLastSwitchHigh = false;
bool shooterFeedInitialized = false;
bool shooterFeedPulseActive = false;
unsigned long shooterFeedPulseStartMs = 0;

// =========================================================
// FLYSKY IBUS
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
      for (int i = 0; i < IBUS_FRAME_LEN - 2; i++) {
        sum -= ibus_buf[i];
      }

      uint16_t rxsum = ibus_buf[30] | (ibus_buf[31] << 8);
      ibus_idx = 0;

      if (sum != rxsum) return false;

      for (int ch = 0; ch < 14; ch++) {
        ibus_ch[ch] = ibus_buf[2 + ch * 2] |
                      (ibus_buf[3 + ch * 2] << 8);
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
  return constrain(us, 1000, 2000);
}

int ibusToPercent(uint8_t chZeroBased) {
  uint16_t us = ibusMicros(chZeroBased);
  return map(us, 1000, 2000, 0, 100);
}

float normalize(int val) {
  const int dead_min = 45;
  const int dead_max = 55;

  if (val >= dead_min && val <= dead_max) return 0.0f;
  if (val < dead_min) {
    return (float)(val - dead_min) / (float)dead_min;
  }
  return (float)(val - dead_max) / (float)(100 - dead_max);
}

// =========================================================
// ROBOCLAW PACKET SERIAL
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

void writeRoboClawM1(HardwareSerial &port, uint8_t addr, float pwr) {
  int speed = constrain(toRoboSpeed(pwr), -127, 127);

  if (speed >= 0) {
    sendRoboClaw(port, addr, 0x00, (uint8_t)speed);
  } else {
    sendRoboClaw(port, addr, 0x01, (uint8_t)(-speed));
  }
}

void writeRoboClawM2(HardwareSerial &port, uint8_t addr, float pwr) {
  int speed = constrain(toRoboSpeed(pwr), -127, 127);

  if (speed >= 0) {
    sendRoboClaw(port, addr, 0x04, (uint8_t)speed);
  } else {
    sendRoboClaw(port, addr, 0x05, (uint8_t)(-speed));
  }
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
// SHOOTER MOTOR
// Copied from the tested non-mecanum implementation.
// =========================================================

void writePwmDirMotor(uint8_t pwmPin, uint8_t dirPin, float value) {
  value = constrain(value, -1.0f, 1.0f);

  int pwm = (int)(fabs(value) * 255.0f);

  if (value > 0.001f) {
    digitalWrite(dirPin, HIGH);
  } else if (value < -0.001f) {
    digitalWrite(dirPin, LOW);
  } else {
    pwm = 0;
  }

  analogWrite(pwmPin, pwm);
}

void writeShooterMotor(float value) {
  writePwmDirMotor(PIN_SHOOTER_PWM, PIN_SHOOTER_DIR, value);
}

void updateShooterFromIbus() {
  const uint8_t idx = CH_SHOOTER - 1;
  const uint16_t shooterUs = ibusMicros(idx);

  // Same tested CH6 calculation used by the non-mecanum sketch:
  // 1000us -> -1.0
  // 1500us ->  0.0
  // 2000us -> +1.0
  float pos =
      (float)map(shooterUs, 1000, 2000, -1000, 1000) / 1000.0f;

  // Uses absolute value so both halves of knob travel spin shooter forward.
  float shooterPower = -fabs(pos) * TELEOP_SHOOTER_SCALE;
  writeShooterMotor(shooterPower);
}

// =========================================================
// SHOOTER FEEDS
// Copied unchanged from the tested non-mecanum implementation.
// =========================================================

void stopShooterFeedServos() {
  shooterFeedLeftServo.writeMicroseconds(SHOOTER_FEED_STOP_US);
  shooterFeedRightServo.writeMicroseconds(SHOOTER_FEED_STOP_US);
}

void startShooterFeedPulse() {
  shooterFeedLeftServo.writeMicroseconds(SHOOTER_FEED_LEFT_RUN_US);
  shooterFeedRightServo.writeMicroseconds(SHOOTER_FEED_RIGHT_RUN_US);
  shooterFeedPulseStartMs = millis();
  shooterFeedPulseActive = true;
}

void updateShooterFeedFromIbus() {
  const uint8_t idx = CH_SHOOTER_FEED - 1;
  const uint16_t swUs = ibusMicros(idx);

  bool switchHigh = swUs > 1500;

  if (!shooterFeedInitialized) {
    shooterFeedLastSwitchHigh = switchHigh;
    shooterFeedInitialized = true;
    stopShooterFeedServos();
    return;
  }

  if (switchHigh != shooterFeedLastSwitchHigh) {
    shooterFeedLastSwitchHigh = switchHigh;
    startShooterFeedPulse();
  }

  if (shooterFeedPulseActive &&
      millis() - shooterFeedPulseStartMs >= SHOOTER_FEED_PULSE_MS) {
    shooterFeedPulseActive = false;
    stopShooterFeedServos();
  }
}

// =========================================================
// MECANUM DRIVE
// This is the only functional difference from differential drive.
// =========================================================

void updateMecanumDriveFromIbus() {
  int throttle = ibusToPercent(CH_DRIVE_THROTTLE - 1);
  int strafe   = ibusToPercent(CH_DRIVE_STRAFE - 1);
  int rotate   = ibusToPercent(CH_DRIVE_ROTATE - 1);

  float fwd  = normalize(throttle) * DRIVE_SCALE;
  float side = normalize(strafe)   * STRAFE_SCALE;
  float turn = normalize(rotate)   * TURN_SCALE;

  // Standard mecanum mix
  float frontLeft  = fwd + side + turn;
  float frontRight = fwd - side - turn;
  float rearLeft   = fwd - side + turn;
  float rearRight  = fwd + side - turn;

  // Preserve direction ratios when any wheel exceeds full command.
  float maxVal = max(
      max(fabs(frontLeft), fabs(frontRight)),
      max(fabs(rearLeft), fabs(rearRight))
  );

  if (maxVal > 1.0f) {
    frontLeft  /= maxVal;
    frontRight /= maxVal;
    rearLeft   /= maxVal;
    rearRight  /= maxVal;
  }

  writeDriveFrontLeft(frontLeft);
  writeDriveFrontRight(frontRight);
  writeDriveRearLeft(rearLeft);
  writeDriveRearRight(rearRight);
}

// =========================================================
// SAFETY
// =========================================================

void stopAllControlledOutputs() {
  stopDrive();
  writeShooterMotor(0.0f);
  stopShooterFeedServos();
  shooterFeedPulseActive = false;
}

// =========================================================
// SETUP
// =========================================================

void setup() {
  IBUS_SERIAL.begin(115200, SERIAL_8N2);
  ROBOCLAW_A_SERIAL.begin(38400);
  ROBOCLAW_B_SERIAL.begin(38400);

  pinMode(PIN_SHOOTER_PWM, OUTPUT);
  pinMode(PIN_SHOOTER_DIR, OUTPUT);

  shooterFeedLeftServo.attach(PIN_SERVO_SHOOTER_FEED_LEFT);
  shooterFeedRightServo.attach(PIN_SERVO_SHOOTER_FEED_RIGHT);

  stopAllControlledOutputs();
}

// =========================================================
// LOOP
// =========================================================

void loop() {
  readIbusFrame();

  // Stop every controlled output if the FlySky/iBus signal is lost.
  if (millis() - ibus_last_frame_ms > IBUS_TIMEOUT_MS) {
    stopAllControlledOutputs();
    delay(20);
    return;
  }

  updateMecanumDriveFromIbus();
  updateShooterFromIbus();
  updateShooterFeedFromIbus();

  delay(20);
}