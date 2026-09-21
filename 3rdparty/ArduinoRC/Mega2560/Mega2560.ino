// 3rdparty/ArduinoRC/Mega2560/Mega2560.ino
//
// Full Pi-compatible BobBot firmware with 4WD mecanum FlySky teleop.
// BobBot / Mega reusable control sketch
//
// Design intent:
// - keep ALL important wiring assignments in one block at the top
// - keep named software points mapped in one place
// - allow the same sketch to be reused across projects by editing constants,
//   not by searching through the whole file
//
// Current active functions:
// - FlySky iBus 4WD mecanum teleop on CH1/CH2/CH4
// - FlySky gripper control on CH5
// - Pi USB serial AUTO mode and existing Pi-facing protocol
// - named motor endpoints:
//     drive_front_left
//     drive_front_right
//     drive_rear_left
//     drive_rear_right
//     shooter
//     collector
// - named servo endpoints:
//     gripper
//     lift
// - semantic Pi protocol:
//     MOTOR <name> WRITE power=<value>
//     SERVO <name> WRITE position=<value>
//     VOLTAGE battery READ
//     CURRENT gripper_right READ
//     REFLECTANCE <name> READ
//     ULTRASONIC <name> READ
//     BUMPER <name> READ
//     LIMIT <name> READ
//     ENCODER <name> READ
//     LED lisiparoi WRITE brightness=<value>
//     AUDIO <name> PLAY ...
//
// Notes:
// - RoboClaw A (pins 18/19) controls the front motors
// - drive_front_left  -> RoboClaw A M1
// - drive_front_right -> RoboClaw A M2
// - RoboClaw B (pins 14/15) controls the rear motors
// - drive_rear_left   -> RoboClaw B M1
// - drive_rear_right  -> RoboClaw B M2
// - gripper is a paired mirrored servo group on pins 11 and 13
// - lift is a single servo on pin 12
// - shooter / collector use Cytron MDD20A wiring defined below

#include <Arduino.h>
#include <math.h>
#include <Servo.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Forward declarations for encoder module types.
// Required so Arduino's automatic .ino function prototypes can see these types.
struct EncoderState;
struct EncoderSnapshot;

// =========================================================
// PIN / LINK ASSIGNMENT BLOCK
// Edit this block first when reusing the sketch.
// =========================================================

// ------------------------- SERIAL / BUS ------------------
#define PI_SERIAL Serial
#define ROBOCLAW_B_SERIAL Serial3   // pins 14/15
#define ROBOCLAW_A_SERIAL Serial1   // pins 18/19
#define IBUS_SERIAL Serial2         // FlySky iBus on Mega RX2 pin 17

// =========================================================
// OPTIONAL FEATURES
// =========================================================

#define ENABLE_ENCODERS            1
#define ENABLE_DRIVE_ENCODERS      1
#define ENABLE_DEADWHEEL_ENCODERS  0
#define ENABLE_SHOOTER_ENCODER     0

#define ENABLE_BATTERY_MONITOR     1
#define ENABLE_INDICATORS          1
#define ENABLE_SENSORS             1

// =========================================================
// PIN ASSIGNMENTS
// =========================================================


static const uint8_t PIN_USB_RX0 = 0;
static const uint8_t PIN_USB_TX0 = 1;

static const uint8_t PIN_ROBOCLAW_B_TX = 14;
static const uint8_t PIN_ROBOCLAW_B_RX = 15;

static const uint8_t PIN_ROBOCLAW_A_TX = 18;
static const uint8_t PIN_ROBOCLAW_A_RX = 19;

static const uint8_t PIN_I2C_SDA = 20;
static const uint8_t PIN_I2C_SCL = 21;

// ------------------------- ANALOG / ADC ------------------
static const uint8_t PIN_VOLTAGE_BATTERY       = A0;
static const uint8_t PIN_CURRENT_GRIPPER_RIGHT = A1;

static const uint8_t PIN_REFLECTANCE_LEFT      = A5;
static const uint8_t PIN_REFLECTANCE_CENTRE    = A6;
static const uint8_t PIN_REFLECTANCE_RIGHT     = A7;

// ------------------------- DIRECT PWM / SERVO ------------
static const uint8_t PIN_COLLECTOR_PWM         = 4;
static const uint8_t PIN_SHOOTER_PWM           = 5;

static const uint8_t PIN_DFPLAYER_SELECT_PWM = 6;

static const uint8_t PIN_ALT_FRONT_RIGHT_PWM = 7;
static const uint8_t PIN_ALT_FRONT_LEFT_PWM  = 8;

static const uint8_t PIN_PIEZO_BUZZER          = 44;
static const uint8_t PIN_LED_LISIPAROI_PWM     = 46;

static const uint8_t PIN_SERVO_SHOOTER_FEED_LEFT  = 9;
static const uint8_t PIN_SERVO_SHOOTER_FEED_RIGHT = 10;

static const uint8_t PIN_SERVO_GRIPPER_LEFT    = 11;
static const uint8_t PIN_SERVO_LIFT            = 12;
static const uint8_t PIN_SERVO_GRIPPER_RIGHT   = 13;

// ------------------------- ENCODERS / LIMITS -------------
static const uint8_t PIN_ENC_DRIVE_FRONT_LEFT_A  = 22;
static const uint8_t PIN_ENC_DRIVE_FRONT_LEFT_B  = 24;

static const uint8_t PIN_ENC_DEADWHEEL_PARALLEL_A = 23;
static const uint8_t PIN_ENC_DEADWHEEL_PARALLEL_B = 25;

static const uint8_t PIN_ENC_DRIVE_FRONT_RIGHT_A = 26;
static const uint8_t PIN_ENC_DRIVE_FRONT_RIGHT_B = 28;

static const uint8_t PIN_ENC_DEADWHEEL_PERPENDICULAR_A = 27;
static const uint8_t PIN_ENC_DEADWHEEL_PERPENDICULAR_B = 29;

static const uint8_t PIN_LIMIT_LIFT_HIGH        = 31;
static const uint8_t PIN_LIMIT_LIFT_LOW         = 33;

static const uint8_t PIN_ENC_SHOOTER_A          = 35;
static const uint8_t PIN_ENC_SHOOTER_B          = 37;

// ------------------------- DIGITAL INPUTS ----------------
static const uint8_t PIN_SELECTOR_PI_ARDUINO    = 32;
static const uint8_t PIN_BUTTON_START = 34;

static const uint8_t PIN_BUMPER_FRONT_LEFT      = 47;
static const uint8_t PIN_BUMPER_FRONT_RIGHT     = 49;

// ------------------------- ULTRASONIC --------------------
static const uint8_t PIN_ULTRASONIC_FRONT_LEFT_TRIG  = 36;
static const uint8_t PIN_ULTRASONIC_FRONT_LEFT_ECHO  = 38;

static const uint8_t PIN_ULTRASONIC_FRONT_RIGHT_TRIG = 40;
static const uint8_t PIN_ULTRASONIC_FRONT_RIGHT_ECHO = 42;

// ------------------------- MOTOR DIR ---------------------
static const uint8_t PIN_SHOOTER_DIR            = 39;
static const uint8_t PIN_COLLECTOR_DIR          = 41;
static const uint8_t PIN_ALT_FRONT_LEFT_DIR     = 43;
static const uint8_t PIN_ALT_FRONT_RIGHT_DIR    = 45;


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
static const uint8_t CH_DRIVE_STRAFE   = 1; // right stick left/right
static const uint8_t CH_DRIVE_THROTTLE = 2; // right stick up/down
static const uint8_t CH_DRIVE_ROTATE   = 4; // left stick left/right

// Teleop drive scaling.
//
// Mecanum rotation depends on the chassis geometry term (L + W), where:
//   L = centre-to-front/rear wheel centreline distance
//   W = centre-to-left/right wheel centreline distance
//
// A square chassis has W = L, giving a reference rotation term of 2L.
// For a rectangular chassis, the geometry correction relative to square is:
//
//   TURN_GEOMETRY_FACTOR = (L + W) / (2L)
//                        = (1 + W/L) / 2
//
// Example: if W = 0.8L:
//   TURN_GEOMETRY_FACTOR = (1 + 0.8) / 2 = 0.90
//
// TELEOP_TURN_SCALE therefore includes this chassis geometry correction.
// Final value may still be adjusted experimentally for preferred handling.

static const float TELEOP_DRIVE_SCALE  = 0.80f;
static const float TELEOP_STRAFE_SCALE = 0.80f;

static const float TURN_GEOMETRY_FACTOR = 0.90f;   // Example: W = 0.8L
static const float TELEOP_TURN_SCALE =
    TELEOP_DRIVE_SCALE * TURN_GEOMETRY_FACTOR;     // 0.80 * 0.90 = 0.72

static const uint8_t CH_GRIP = 5; // currently assigned gripper control
static const uint8_t CH_LIFT = 6; // knob to the right of gripper knob
static const float TELEOP_SHOOTER_SCALE = 0.8f; // CH_LIFT also controls shooter power

static const uint8_t CH_SHOOTER_FEED = 7; // SWA, pulse shooter feed servos
static const uint8_t CH_COLLECTOR = 8;          // SWB 3-position
static const float TELEOP_COLLECTOR_SCALE = 0.8f;


static const unsigned long SHOOTER_FEED_PULSE_MS = 700;
static const int SHOOTER_FEED_STOP_US = 1500;
static const int SHOOTER_FEED_LEFT_RUN_US = 1700;
static const int SHOOTER_FEED_RIGHT_RUN_US = 1300;

// ------------------------- SERVO CALIBRATION -------------
static const int GRIP_LEFT_OPEN_US      = 900;
static const int GRIP_LEFT_CLOSED_US    = 2200;
static const int GRIP_RIGHT_OPEN_US     = 2100;
static const int GRIP_RIGHT_CLOSED_US   = 800;

static const int LIFT_DOWN_US           = 800;
static const int LIFT_UP_US             = 2250;

// ------------------------- SYSTEM ------------------------
static const char DEVICE_ID[] = "MEGA_AUX_1";
static const unsigned long PI_HEARTBEAT_TIMEOUT_MS = 86400000UL; // 24 hours; (500 ms)

// =========================================================
// STATE
// =========================================================

// Front-drive hardware routing
enum FrontDriveRoute {
  FRONT_DRIVE_ROBOCLAW,
  FRONT_DRIVE_MDD20A
};

FrontDriveRoute frontDriveRoute = FRONT_DRIVE_MDD20A;

// Pi / AUTO state

bool piAutoRequested = false;
unsigned long piLastHeartbeatMs = 0;

// Semantic output state controlled by AUTO mode
float motorDriveFrontLeftPower  = 0.0f;
float motorDriveFrontRightPower = 0.0f;
float motorDriveRearLeftPower   = 0.0f;
float motorDriveRearRightPower  = 0.0f;
float motorShooterPower         = 0.0f;
float motorCollectorPower       = 0.0f;

float servoGripperPosition      = -1.0f;   // -1=open, +1=closed
float servoLiftPosition         = 0.0f;    // -1=down, +1=up

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
Servo shooterFeedLeftServo;
Servo shooterFeedRightServo;

bool shooterFeedLastSwitchHigh = false;
bool shooterFeedInitialized = false;
bool shooterFeedPulseActive = false;
unsigned long shooterFeedPulseStartMs = 0;

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

void writePwmDirMotor(uint8_t pwmPin, uint8_t dirPin, float value);

void writeDriveFrontLeft(float pwr) {
  if (frontDriveRoute == FRONT_DRIVE_MDD20A) {
    writePwmDirMotor(
      PIN_ALT_FRONT_LEFT_PWM,
      PIN_ALT_FRONT_LEFT_DIR,
      pwr
    );
    return;
  }

  writeRoboClawM1(
    ROBOCLAW_A_SERIAL,
    ROBOCLAW_ADDR_A,
    pwr
  );
}

void writeDriveFrontRight(float pwr) {
  if (frontDriveRoute == FRONT_DRIVE_MDD20A) {
    writePwmDirMotor(
      PIN_ALT_FRONT_RIGHT_PWM,
      PIN_ALT_FRONT_RIGHT_DIR,
      pwr
    );
    return;
  }

  writeRoboClawM2(
    ROBOCLAW_A_SERIAL,
    ROBOCLAW_ADDR_A,
    pwr
  );
}

void writeDriveRearLeft(float pwr) {
  writeRoboClawM1(ROBOCLAW_B_SERIAL, ROBOCLAW_ADDR_B, pwr);
}

void writeDriveRearRight(float pwr) {
  writeRoboClawM2(ROBOCLAW_B_SERIAL, ROBOCLAW_ADDR_B, pwr);
}

void stopDrive() {
  // Normal front RoboClaw
  writeRoboClawM1(
    ROBOCLAW_A_SERIAL,
    ROBOCLAW_ADDR_A,
    0.0f
  );
  writeRoboClawM2(
    ROBOCLAW_A_SERIAL,
    ROBOCLAW_ADDR_A,
    0.0f
  );

  // Alternate front MDD20A
  writePwmDirMotor(
    PIN_ALT_FRONT_LEFT_PWM,
    PIN_ALT_FRONT_LEFT_DIR,
    0.0f
  );
  writePwmDirMotor(
    PIN_ALT_FRONT_RIGHT_PWM,
    PIN_ALT_FRONT_RIGHT_DIR,
    0.0f
  );

  // Rear drive
  writeDriveRearLeft(0.0f);
  writeDriveRearRight(0.0f);
}

// =========================================================
// DIRECT MOTOR DRIVER (CYTRON MDD20A)
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

void writeCollectorMotor(float value) {
  writePwmDirMotor(PIN_COLLECTOR_PWM, PIN_COLLECTOR_DIR, value);
}

// =========================================================
// COLLECTOR
// =========================================================

void updateCollectorFromIbus() {
  const uint8_t idx = CH_COLLECTOR - 1;
  const uint16_t swUs = ibusMicros(idx);

  // SWB is a 3-position switch:
  // low  -> reverse
  // mid  -> stop
  // high -> forward

  if (swUs < 1250) {
    writeCollectorMotor(-TELEOP_COLLECTOR_SCALE);
  }
  else if (swUs > 1750) {
    writeCollectorMotor(TELEOP_COLLECTOR_SCALE);
  }
  else {
    writeCollectorMotor(0.0f);
  }
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
  // 1000us -> -1.0
  // 1500us ->  0.0
  // 2000us -> +1.0
  float pos = (float)map(liftUs, 1000, 2000, -1000, 1000) / 1000.0f;

  setLiftNormalized(pos);

  // Same CH6 knob also controls shooter power in teleop.
  // Uses absolute value so both halves of knob travel spin shooter forward.
  float shooterPower = -fabs(pos) * TELEOP_SHOOTER_SCALE;
  writeShooterMotor(shooterPower);
}

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
    motorDriveFrontLeftPower = value;
    return true;
  }

  if (strcmp(name, MOTOR_NAME_DRIVE_FRONT_RIGHT) == 0) {
    motorDriveFrontRightPower = value;
    return true;
  }

  if (strcmp(name, MOTOR_NAME_DRIVE_REAR_LEFT) == 0) {
    motorDriveRearLeftPower = value;
    return true;
  }

  if (strcmp(name, MOTOR_NAME_DRIVE_REAR_RIGHT) == 0) {
    motorDriveRearRightPower = value;
    return true;
  }

  if (strcmp(name, MOTOR_NAME_SHOOTER) == 0) {
    motorShooterPower = value;
    return true;
  }

  if (strcmp(name, MOTOR_NAME_COLLECTOR) == 0) {
    motorCollectorPower = value;
    return true;
  }

  return false;
}

bool setServoByName(const char *name, float value) {
  value = constrain(value, -1.0f, 1.0f);

  if (strcmp(name, SERVO_NAME_GRIPPER) == 0) {
    servoGripperPosition = value;
    return true;
  }

  if (strcmp(name, SERVO_NAME_LIFT) == 0) {
    servoLiftPosition = value;
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

#if ENABLE_ENCODERS
  if (handleEncoderCommand(line)) {
    return;
  }
#endif

#if ENABLE_BATTERY_MONITOR
  if (handleBatteryCommand(line)) {
    return;
  }
#endif

#if ENABLE_INDICATORS
  if (handleIndicatorCommand(line)) {
    return;
  }
#endif

#if ENABLE_SENSORS
  if (handleSensorCommand(line)) {
    return;
  }
#endif

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


if (strncmp(line, "FRONT_ROUTE ", 12) == 0) {
  const char *routeName = line + 12;

  if (strcmp(routeName, "ROBOCLAW") == 0) {
    if (frontDriveRoute != FRONT_DRIVE_ROBOCLAW) {
      stopDrive();
      motorDriveFrontLeftPower = 0.0f;
      motorDriveFrontRightPower = 0.0f;
      frontDriveRoute = FRONT_DRIVE_ROBOCLAW;
    }

    PI_SERIAL.println("OK FRONT_ROUTE ROBOCLAW");
    return;
  }

  if (strcmp(routeName, "MDD20A") == 0) {
    if (frontDriveRoute != FRONT_DRIVE_MDD20A) {
      stopDrive();
      motorDriveFrontLeftPower = 0.0f;
      motorDriveFrontRightPower = 0.0f;
      frontDriveRoute = FRONT_DRIVE_MDD20A;
    }

    PI_SERIAL.println("OK FRONT_ROUTE MDD20A");
    return;
  }

  PI_SERIAL.print("ERR FRONT_ROUTE ");
  PI_SERIAL.println(routeName);
  return;
}


  if (strcmp(line, "STOP") == 0) {
    motorDriveFrontLeftPower = 0.0f;
    motorDriveFrontRightPower = 0.0f;
    motorDriveRearLeftPower = 0.0f;
    motorDriveRearRightPower = 0.0f;
    motorShooterPower = 0.0f;
    motorCollectorPower = 0.0f;

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
          motorDriveFrontLeftPower = value;
          ok = true;
        } else if (strcmp(tokCh, "M2") == 0) {
          motorDriveFrontRightPower = value;
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

      if (pin == PIN_SERVO_LIFT) {
        servoLiftPosition = value;
      } else if (pin == PIN_SERVO_GRIPPER_LEFT || pin == PIN_SERVO_GRIPPER_RIGHT) {
        // One logical gripper command; Mega mirrors internally.
        servoGripperPosition = value;
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

      if ((pin1 == PIN_SERVO_GRIPPER_LEFT && pin2 == PIN_SERVO_GRIPPER_RIGHT) ||
          (pin1 == PIN_SERVO_GRIPPER_RIGHT && pin2 == PIN_SERVO_GRIPPER_LEFT)) {
        // Prefer one logical gripper command. Use the first value.
        (void)val2;
        servoGripperPosition = val1;
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

      if ((pin1 == PIN_SERVO_GRIPPER_LEFT && pin2 == PIN_SERVO_GRIPPER_RIGHT) ||
          (pin1 == PIN_SERVO_GRIPPER_RIGHT && pin2 == PIN_SERVO_GRIPPER_LEFT)) {
        us1 = constrain(us1, 500, 2500);
        us2 = constrain(us2, 500, 2500);
        gripLeftServo.writeMicroseconds(us1);
        gripRightServo.writeMicroseconds(us2);
      } else if (pin1 == PIN_SERVO_LIFT && pin2 == PIN_SERVO_LIFT) {
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

  if (strncmp(line, "PWM_DIR_WRITE ", 14) == 0) {
    char *p = line + 14;
    char *tokPwm = strtok(p, " ");
    char *tokDir = strtok(nullptr, " ");
    char *tokVal = strtok(nullptr, " ");

    if (tokPwm && tokDir && tokVal) {
      int pwmPin = atoi(tokPwm);
      int dirPin = atoi(tokDir);
      float value = constrain(atof(tokVal), -1.0f, 1.0f);

      if (pwmPin == PIN_SHOOTER_PWM && dirPin == PIN_SHOOTER_DIR) {
        motorShooterPower = value;
      } else if (pwmPin == PIN_COLLECTOR_PWM && dirPin == PIN_COLLECTOR_DIR) {
        motorCollectorPower = value;
      } else {
        writePwmDirMotor((uint8_t)pwmPin, (uint8_t)dirPin, value);
      }

      PI_SERIAL.print("OK PWM_DIR_WRITE ");
      PI_SERIAL.print(pwmPin);
      PI_SERIAL.print(" ");
      PI_SERIAL.println(dirPin);
      return;
    }
  }

  // ---------------------------------------------------------
  // Semantic protocol (kept for compatibility)
  // ---------------------------------------------------------

  char kind[16];
  char name[32];
  float value = 0.0f;

  // Coordinated drive protocol:
  //   DRIVE front WRITE left=0.3000 right=0.3000
  //   DRIVE rear WRITE left=0.3000 right=0.3000
  char driveName[16];
  char driveLeftText[32];
  char driveRightText[32];

  if (sscanf(
      line,
      "DRIVE %15s WRITE left=%31s right=%31s",
      driveName,
      driveLeftText,
      driveRightText
    ) == 3) {

    float leftPower = constrain(atof(driveLeftText), -1.0f, 1.0f);
    float rightPower = constrain(atof(driveRightText), -1.0f, 1.0f);

    if (strcmp(driveName, "front") == 0) {
      motorDriveFrontLeftPower = leftPower;
      motorDriveFrontRightPower = rightPower;
    } else if (strcmp(driveName, "rear") == 0) {
      motorDriveRearLeftPower = leftPower;
      motorDriveRearRightPower = rightPower;
    } else {
      PI_SERIAL.print("ERR DRIVE ");
      PI_SERIAL.println(driveName);
      return;
    }

    PI_SERIAL.print("OK DRIVE ");
    PI_SERIAL.print(driveName);
    PI_SERIAL.print(" left=");
    PI_SERIAL.print(leftPower, 4);
    PI_SERIAL.print(" right=");
    PI_SERIAL.println(rightPower, 4);
    return;
  }

  char motorName[32];
  char motorValueText[32];

  if (sscanf(line, "MOTOR %31s WRITE power=%31s", motorName, motorValueText) == 2 ||
      sscanf(line, "MOTOR %31s WRITE %31s", motorName, motorValueText) == 2) {

    float motorPower = atof(motorValueText);

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

  // New semantic servo protocol:
  //   SERVO gripper WRITE position=0.5
  //   SERVO lift WRITE position=0.5
  //   SERVO gripper WRITE 0.5
  char servoName[32];
  char servoValueText[32];

  if (sscanf(line, "SERVO %31s WRITE position=%31s", servoName, servoValueText) == 2 ||
      sscanf(line, "SERVO %31s WRITE %31s", servoName, servoValueText) == 2) {

    float servoPosition = atof(servoValueText);

    if (setServoByName(servoName, servoPosition)) {
      PI_SERIAL.print("OK SERVO ");
      PI_SERIAL.print(servoName);
      PI_SERIAL.print(" position=");
      PI_SERIAL.println(servoPosition, 4);
    } else {
      PI_SERIAL.print("ERR SERVO ");
      PI_SERIAL.println(servoName);
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


    #if ENABLE_ENCODERS
      if (strcmp(rkind, "QUAD") == 0 && count >= 3) {
        replyValue("QUAD", readQuadPair((uint8_t)atoi(a1), (uint8_t)atoi(a2)));
        return;
      }
    #endif
  }

  // Backward compatibility during migration
  if (strncmp(line, "DRV ", 4) == 0) {
    char *p = line + 4;
    char *tok1 = strtok(p, " ");
    char *tok2 = strtok(nullptr, " ");

    if (tok1 && tok2) {
      motorDriveFrontLeftPower = constrain(atof(tok1), -1.0f, 1.0f);
      motorDriveFrontRightPower = constrain(atof(tok2), -1.0f, 1.0f);
      PI_SERIAL.println("OK DRV");
      return;
    }
  }

  if (strncmp(line, "GRIP ", 5) == 0) {
    servoGripperPosition = constrain(atof(line + 5), -1.0f, 1.0f);
    PI_SERIAL.println("OK GRIP");
    return;
  }

  if (strncmp(line, "LIFT ", 5) == 0) {
    servoLiftPosition = constrain(atof(line + 5), -1.0f, 1.0f);
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

  // Direct PWM/DIR motor outputs
  pinMode(PIN_SHOOTER_PWM, OUTPUT);
  pinMode(PIN_SHOOTER_DIR, OUTPUT);

  pinMode(PIN_COLLECTOR_PWM, OUTPUT);
  pinMode(PIN_COLLECTOR_DIR, OUTPUT);

  // Alternate front-drive MDD20A
  pinMode(PIN_ALT_FRONT_LEFT_PWM, OUTPUT);
  pinMode(PIN_ALT_FRONT_LEFT_DIR, OUTPUT);

  pinMode(PIN_ALT_FRONT_RIGHT_PWM, OUTPUT);
  pinMode(PIN_ALT_FRONT_RIGHT_DIR, OUTPUT);


  // Indicators
  #if ENABLE_INDICATORS
    setupIndicators();
  #endif

  // Battery monitor
  #if ENABLE_BATTERY_MONITOR
    setupBatteryMonitor();
  #endif

  // Sensors
  #if ENABLE_SENSORS
    setupSensors();
  #endif

  // Encoders
  #if ENABLE_ENCODERS
    setupEncoders();
  #endif



  // Servos
  gripLeftServo.attach(PIN_SERVO_GRIPPER_LEFT);
  gripRightServo.attach(PIN_SERVO_GRIPPER_RIGHT);
  liftServo.attach(PIN_SERVO_LIFT);
  shooterFeedLeftServo.attach(PIN_SERVO_SHOOTER_FEED_LEFT);
  shooterFeedRightServo.attach(PIN_SERVO_SHOOTER_FEED_RIGHT);

  setGripNormalized(-1.0f); // open
  setLiftNormalized(0.0f);  // neutral / midpoint
  stopShooterFeedServos();

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

  #if ENABLE_BATTERY_MONITOR
    serviceBatteryMonitor();
   #endif

  #if ENABLE_INDICATORS
    serviceIndicators();
  #endif

  #if ENABLE_BATTERY_MONITOR
    if (batteryShutdownActive()) {
      delay(20);
      return;
    }
  #endif

  // AUTO owns outputs while heartbeat is fresh.
  if (piHasControl()) {
    writeDriveFrontLeft(motorDriveFrontLeftPower);
    writeDriveFrontRight(motorDriveFrontRightPower);
    writeDriveRearLeft(motorDriveRearLeftPower);
    writeDriveRearRight(motorDriveRearRightPower);
    writeShooterMotor(motorShooterPower);
    writeCollectorMotor(motorCollectorPower);
    setGripNormalized(servoGripperPosition);
    setLiftNormalized(servoLiftPosition);
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

    // Safety: if FlySky signal is lost, stop all teleop-controlled motion.
  if (ibus_last_frame_ms == 0 ||
    millis() - ibus_last_frame_ms > 200) {
    stopDrive();
    writeShooterMotor(0.0f);
    writeCollectorMotor(0.0f);
    stopShooterFeedServos();
    shooterFeedPulseActive = false;
    return;
  }

  // Select front motor controller from FlySky.
  // Route changes are accepted only while drive controls are neutral.


  // 4WD mecanum drive from FlySky.
  // CH2 = forward/reverse, CH1 = strafe, CH4 = rotate.
  int throttle = ibusToPercent(CH_DRIVE_THROTTLE - 1);
  int strafe   = ibusToPercent(CH_DRIVE_STRAFE - 1);
  int rotate   = ibusToPercent(CH_DRIVE_ROTATE - 1);

  float fwd  = normalize(throttle) * TELEOP_DRIVE_SCALE;
  float side = normalize(strafe)   * TELEOP_STRAFE_SCALE;
  float turn = normalize(rotate)   * TELEOP_TURN_SCALE;

  // Standard mecanum mix.
  float frontLeft  = fwd + side + turn;
  float frontRight = fwd - side - turn;
  float rearLeft   = fwd - side + turn;
  float rearRight  = fwd + side - turn;

  // Preserve the wheel-command ratios when any command exceeds full scale.
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
  updateGripFromIbus();
  updateLiftFromIbus();
  updateCollectorFromIbus();
  updateShooterFeedFromIbus();

  delay(20);
}
