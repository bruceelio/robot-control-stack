# Arduino Mega Firmware File Structure

## Purpose

This document defines the recommended file structure for the Arduino Mega 2560 firmware.

The objective is to keep `Mega2560.ino` as the robot-level overview while moving each complete functional capability into its own `.ino` tab.

The goal is not to make every file completely standalone. All physical pin assignments remain in the main file so the complete robot wiring can be inspected in one place.

The intended result is:

- a short, readable main sketch;
- one file per major robot function;
- minimal integration points between the main sketch and each feature;
- optional features that can be enabled or disabled at compile time;
- functional sections that can be developed, tested or removed without navigating through unrelated code.

A programmer should be able to:

1. open `Mega2560.ino` to understand the whole robot;
2. open one functional `.ino` tab to work on one capability.

---

## 1. Proposed Master File Layout

A practical first-stage layout is:

```text
Mega2560/
├── Mega2560.ino
├── battery.ino
├── drive_roboclaw.ino
├── encoders.ino
├── flysky_ibus.ino
├── motors_pwm.ino
├── pi_protocol.ino
├── servos.ino
└── sensors.ino
```

An optional later expansion is:

```text
Mega2560/
├── Mega2560.ino
├── battery.ino
├── drive_roboclaw.ino
├── encoders.ino
├── flysky_ibus.ino
├── motors_pwm.ino
├── pi_protocol.ino
├── servos.ino
├── digital_inputs.ino
├── reflectance.ino
├── ultrasonics.ino
└── indicators.ino
```

The first-stage layout is recommended initially. Small sensor functions can remain grouped in `sensors.ino` until they become large enough to justify separate files.

---

## 2. Design Principle

`Mega2560.ino` remains the robot definition and coordinator.

Each functional `.ino` file owns one complete capability.

```text
Mega2560.ino
    defines what hardware exists
    defines which features are enabled
    defines the physical pin allocation
    initializes enabled features
    schedules enabled features
    coordinates control modes

Functional .ino files
    contain the implementation
    contain feature-specific state
    contain feature-specific calibration
    contain feature-specific command handling
    contain feature-specific diagnostics
```

The functional files are not required to be standalone Arduino programs.

They may use:

- pin constants declared in `Mega2560.ino`;
- serial objects declared in `Mega2560.ino`;
- logical endpoint names declared in `Mega2560.ino`;
- system-wide control state declared in `Mega2560.ino`.

This is intentional.

---

## 3. Feature Selection

Near the top of `Mega2560.ino`, define which capabilities are present on the current robot.

```cpp
// =========================================================
// OPTIONAL FEATURES
// =========================================================

#define ENABLE_PI_PROTOCOL           1
#define ENABLE_FLYSKY_IBUS           1

#define ENABLE_DRIVE                 1
#define ENABLE_PWM_MOTORS            1
#define ENABLE_SERVOS                1

#define ENABLE_ENCODERS              1
#define ENABLE_DRIVE_ENCODERS        1
#define ENABLE_DEADWHEEL_ENCODERS    0
#define ENABLE_SHOOTER_ENCODER       0

#define ENABLE_BATTERY_MONITOR       1
#define ENABLE_SENSORS               1
#define ENABLE_INDICATORS            0
```

For a robot without encoders:

```cpp
#define ENABLE_ENCODERS              0
#define ENABLE_DRIVE_ENCODERS        0
#define ENABLE_DEADWHEEL_ENCODERS    0
#define ENABLE_SHOOTER_ENCODER       0
```

These are compile-time settings.

Code enclosed by:

```cpp
#if ENABLE_ENCODERS

// Encoder implementation

#endif
```

is omitted from the compiled firmware when the feature is disabled.

This may reduce:

- flash usage;
- RAM usage;
- setup work;
- loop processing;
- protocol comparisons;
- timer interrupts;
- interrupt service routine workload.

The most important runtime saving occurs when a disabled feature would otherwise run continuously, such as timer-driven encoder sampling.

---

## 4. What Remains in `Mega2560.ino`

`Mega2560.ino` should remain the authoritative robot overview.

It should contain the following sections.

### 4.1 File Header and Architecture Notes

Keep the introductory documentation describing:

- the purpose of the Mega;
- its relationship with the Raspberry Pi;
- manual control through FlySky;
- RoboClaw motor control;
- logical device naming;
- the serial command model;
- high-level safety and control behaviour.

### 4.2 Common Library Includes

For example:

```cpp
#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
```

Libraries used exclusively by one feature may be included in that functional file where practical.

### 4.3 Feature Selection

All compile-time feature switches should remain together near the top of the main file.

### 4.4 Physical Pin Assignments

All physical pin assignments remain in `Mega2560.ino`.

This preserves one authoritative wiring map for the robot.

Examples include:

```text
SERIAL / BUS
ANALOG / ADC
DIRECT PWM
SERVO OUTPUTS
ENCODERS
LIMIT SWITCHES
DIGITAL INPUTS
ULTRASONIC
MOTOR DIRECTION
INDICATORS
```

Example:

```cpp
// ------------------------- ENCODERS -----------------------

static const uint8_t PIN_ENC_DRIVE_FRONT_LEFT_A  = 22;
static const uint8_t PIN_ENC_DRIVE_FRONT_LEFT_B  = 24;

static const uint8_t PIN_ENC_DRIVE_FRONT_RIGHT_A = 26;
static const uint8_t PIN_ENC_DRIVE_FRONT_RIGHT_B = 28;

static const uint8_t PIN_ENC_DEADWHEEL_PARALLEL_A = 23;
static const uint8_t PIN_ENC_DEADWHEEL_PARALLEL_B = 25;

static const uint8_t PIN_ENC_DEADWHEEL_PERPENDICULAR_A = 27;
static const uint8_t PIN_ENC_DEADWHEEL_PERPENDICULAR_B = 29;

static const uint8_t PIN_ENC_SHOOTER_A = 35;
static const uint8_t PIN_ENC_SHOOTER_B = 37;
```

The feature implementation is separated, but the complete pin allocation remains visible centrally.

### 4.5 Serial Assignments

Keep the serial-port mappings in the main file.

Examples:

```cpp
#define PI_SERIAL Serial
#define IBUS_SERIAL Serial1
#define ROBOCLAW_FRONT_SERIAL Serial2
#define ROBOCLAW_REAR_SERIAL Serial3
```

### 4.6 Logical Endpoint Names

Logical device names form part of the robot-facing IO contract and may remain centrally visible.

For example:

```cpp
static const char MOTOR_NAME_DRIVE_FRONT_LEFT[]  = "drive_front_left";
static const char MOTOR_NAME_DRIVE_FRONT_RIGHT[] = "drive_front_right";
static const char MOTOR_NAME_DRIVE_REAR_LEFT[]   = "drive_rear_left";
static const char MOTOR_NAME_DRIVE_REAR_RIGHT[]  = "drive_rear_right";

static const char MOTOR_NAME_SHOOTER[]   = "shooter";
static const char MOTOR_NAME_COLLECTOR[] = "collector";

static const char SERVO_NAME_GRIPPER[] = "gripper";
static const char SERVO_NAME_LIFT[]    = "lift";
```

Feature-specific names may instead remain in their functional file when removing the feature should also remove its semantic names.

For example, encoder names can be defined in `encoders.ino`.

### 4.7 System-Wide State

Only state that coordinates several modules should remain in the main file.

For example:

```cpp
bool piAutoRequested = false;
unsigned long piLastHeartbeatMs = 0;
```

A clearer future form may be:

```cpp
enum ControlSource {
  CONTROL_NONE,
  CONTROL_PI,
  CONTROL_FLYSKY
};

ControlSource activeControlSource = CONTROL_NONE;
```

Feature-specific state should move to the feature file.

Examples:

```text
iBus buffers                  → flysky_ibus.ino
servo objects                 → servos.ino
encoder counts                → encoders.ino
battery alarm state           → battery.ino
motor command state           → drive_roboclaw.ino or motors_pwm.ino
```

### 4.8 `setup()`

The main setup function should become a readable list of enabled capabilities.

```cpp
void setup() {
  setupCoreCommunications();

#if ENABLE_DRIVE
  setupDrive();
#endif

#if ENABLE_PWM_MOTORS
  setupPwmMotors();
#endif

#if ENABLE_SERVOS
  setupServos();
#endif

#if ENABLE_FLYSKY_IBUS
  setupFlySkyIbus();
#endif

#if ENABLE_ENCODERS
  setupEncoders();
#endif

#if ENABLE_BATTERY_MONITOR
  setupBatteryMonitor();
#endif

#if ENABLE_SENSORS
  setupSensors();
#endif

#if ENABLE_INDICATORS
  setupIndicators();
#endif
}
```

The feature-specific pin setup and implementation details move into the corresponding functional file.

### 4.9 `loop()`

The main loop should contain only high-level scheduling and control decisions.

```cpp
void loop() {
#if ENABLE_PI_PROTOCOL
  servicePiSerial();
#endif

#if ENABLE_FLYSKY_IBUS
  serviceFlySkyIbus();
#endif

#if ENABLE_ENCODERS
  serviceEncoders();
#endif

#if ENABLE_BATTERY_MONITOR
  serviceBatteryMonitor();
#endif

#if ENABLE_SERVOS
  serviceServos();
#endif

#if ENABLE_INDICATORS
  serviceIndicators();
#endif

  updateControlSource();
  applyRobotControl();
}
```

Raw sensor acquisition, motor calculations, servo timing and command parsing should not dominate the main loop.

---

## 5. `pi_protocol.ino`

This file owns the common Raspberry Pi serial transport and top-level command dispatch.

### Responsibilities

It should contain:

- input-line buffering;
- complete-line detection;
- Pi heartbeat handling;
- Pi/manual control selection;
- generic reply helpers;
- top-level command dispatch;
- generic unknown-command errors.

Typical functions:

```cpp
void setupPiProtocol();
void servicePiSerial();

bool piHeartbeatFresh();
bool piHasControl();

void handlePiCommand(char* line);
void replyValue(...);
```

### Command Dispatch

`handlePiCommand()` should become a dispatcher rather than contain all hardware logic.

Conceptually:

```cpp
void handlePiCommand(char* line) {
  if (handleSystemCommand(line)) {
    return;
  }

#if ENABLE_DRIVE
  if (handleDriveCommand(line)) {
    return;
  }
#endif

#if ENABLE_PWM_MOTORS
  if (handlePwmMotorCommand(line)) {
    return;
  }
#endif

#if ENABLE_SERVOS
  if (handleServoCommand(line)) {
    return;
  }
#endif

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

#if ENABLE_SENSORS
  if (handleSensorCommand(line)) {
    return;
  }
#endif

  PI_SERIAL.println(F("ERR unknown_command"));
}
```

Each functional file should preferably own its own semantic command parsing.

This keeps:

```text
encoder commands    in encoders.ino
battery commands    in battery.ino
servo commands      in servos.ino
drive commands      in drive_roboclaw.ino
sensor commands     in sensors.ino
```

---

## 6. `drive_roboclaw.ino`

This file owns the complete RoboClaw drivetrain output implementation.

### Move Into This File

- RoboClaw CRC calculation;
- packet transmission;
- RoboClaw M1/M2 writes;
- conversion from normalized power to RoboClaw command values;
- individual drive-motor writes;
- drive-motor command state;
- drivetrain stopping;
- mecanum mixing currently performed in `loop()`.

Typical functions:

```cpp
void setupDrive();

void writeDriveFrontLeft(float power);
void writeDriveFrontRight(float power);
void writeDriveRearLeft(float power);
void writeDriveRearRight(float power);

void setMecanumDrive(
    float forward,
    float strafe,
    float rotation
);

bool setDriveMotorByName(
    const char* name,
    float power
);

bool handleDriveCommand(char* line);

void stopDrive();
```

### Main File Retains

- RoboClaw serial assignments;
- RoboClaw addresses;
- drive-motor logical names;
- drive feature switch;
- high-level control-source selection.

---

## 7. `motors_pwm.ino`

This file owns motors controlled by direct PWM and direction outputs rather than RoboClaw.

Current examples include:

- shooter motor;
- collector motor.

### Move Into This File

- generic PWM/direction motor writes;
- shooter power state;
- collector power state;
- shooter motor control;
- collector motor control;
- PWM motor command parsing;
- PWM motor stop behaviour.

Typical functions:

```cpp
void setupPwmMotors();

void setShooterPower(float power);
void setCollectorPower(float power);

bool setPwmMotorByName(
    const char* name,
    float power
);

bool handlePwmMotorCommand(char* line);

void stopPwmMotors();
```

Keeping direct PWM motors separate from RoboClaw drive motors is useful because the hardware and software interfaces are different.

---

## 8. `servos.ino`

This file owns all servo objects, calibration, positions and timed servo behaviour.

### Move Into This File

Servo objects:

```cpp
Servo gripLeftServo;
Servo gripRightServo;
Servo liftServo;
Servo shooterFeedLeftServo;
Servo shooterFeedRightServo;
```

Servo state:

```cpp
float servoGripperPosition;
float servoLiftPosition;

bool shooterFeedLastSwitchHigh;
bool shooterFeedInitialized;
bool shooterFeedPulseActive;

unsigned long shooterFeedPulseStartMs;
```

Servo calibration:

```text
GRIP_LEFT_OPEN_US
GRIP_LEFT_CLOSED_US
GRIP_RIGHT_OPEN_US
GRIP_RIGHT_CLOSED_US

LIFT_DOWN_US
LIFT_UP_US

SHOOTER_FEED_PULSE_MS
SHOOTER_FEED_STOP_US
SHOOTER_FEED_LEFT_RUN_US
SHOOTER_FEED_RIGHT_RUN_US
```

Servo functions:

```cpp
void setupServos();
void serviceServos();

void setGripPositionUs(uint16_t pulseUs);
void setGripNormalized(float position);
void setLiftNormalized(float position);

void stopShooterFeedServos();
void startShooterFeedPulse();

void updateGripFromIbus();
void updateLiftFromIbus();
void updateShooterFeedFromIbus();

bool setServoByName(
    const char* name,
    float position
);

bool handleServoCommand(char* line);

void stopServos();
```

### Main File Retains

- physical servo pin assignments;
- servo feature switch;
- logical servo endpoint names, if they remain part of the central IO map.

---

## 9. `flysky_ibus.ino`

This file owns the complete FlySky receiver interface.

### Move Into This File

- iBus receive buffer;
- receive index;
- decoded channel array;
- last-valid-frame time;
- frame parsing;
- channel pulse access;
- channel-to-percent conversion;
- normalized control conversion;
- manual-control channel mapping;
- teleoperation scale values;
- manual command extraction.

Typical state:

```cpp
static const uint8_t IBUS_FRAME_LEN = 32;

uint8_t ibusBuffer[IBUS_FRAME_LEN];
uint8_t ibusIndex = 0;

uint16_t ibusChannels[14];

unsigned long ibusLastFrameMs = 0;
```

Typical functions:

```cpp
void setupFlySkyIbus();
void serviceFlySkyIbus();

bool readIbusFrame();
bool flySkyIsFresh();

uint16_t ibusMicros(uint8_t channel);
int ibusToPercent(uint8_t channel);
float normalizeIbusValue(int value);

float getFlySkyDriveForward();
float getFlySkyDriveStrafe();
float getFlySkyDriveRotation();

float getFlySkyGripCommand();
float getFlySkyLiftCommand();

bool getFlySkyShooterFeedCommand();
```

Other modules should preferably use these semantic functions instead of directly reading `ibusChannels[]`.

### Main File Retains

- the iBus serial assignment;
- the feature switch;
- the high-level decision about whether FlySky has control.

---

## 10. `encoders.ino`

This file owns every encoder capability.

It may contain several encoder groups:

```text
encoders.ino
├── shared encoder types
├── shared quadrature decoder
├── drive-motor encoders
├── deadwheel encoders
├── shooter encoder
├── atomic snapshots
├── diagnostics
├── name resolution
└── encoder protocol commands
```

### Feature Structure

```cpp
#if ENABLE_ENCODERS

// Shared encoder definitions

#if ENABLE_DRIVE_ENCODERS
// Drive-motor encoder implementation
#endif

#if ENABLE_DEADWHEEL_ENCODERS
// Deadwheel encoder implementation
#endif

#if ENABLE_SHOOTER_ENCODER
// Shooter encoder implementation
#endif

#endif
```

### Main File Retains

- encoder feature switches;
- all encoder pin assignments;
- one `setupEncoders()` call;
- one optional `serviceEncoders()` call;
- one protocol delegation point.

### Encoder File Owns

- encoder state structures;
- quadrature decoding;
- direct-port or interrupt acquisition;
- timer configuration;
- interrupt service routines;
- cumulative count storage;
- invalid-transition diagnostics;
- atomic snapshots;
- encoder validity;
- encoder semantic names;
- encoder command parsing;
- encoder serial responses.

### Unified Setup

```cpp
void setupEncoders() {
#if ENABLE_DRIVE_ENCODERS
  setupDriveEncoders();
#endif

#if ENABLE_DEADWHEEL_ENCODERS
  setupDeadwheelEncoders();
#endif

#if ENABLE_SHOOTER_ENCODER
  setupShooterEncoder();
#endif
}
```

### Unified Service

```cpp
void serviceEncoders() {
#if ENABLE_DRIVE_ENCODERS
  serviceDriveEncoders();
#endif

#if ENABLE_DEADWHEEL_ENCODERS
  serviceDeadwheelEncoders();
#endif

#if ENABLE_SHOOTER_ENCODER
  serviceShooterEncoder();
#endif
}
```

For timer-driven drive encoders, raw counting occurs in the timer interrupt rather than in `serviceEncoders()`.

The service function can later handle:

- deferred diagnostics;
- velocity calculations;
- reset requests;
- low-rate health reporting.

### Unified Name Resolution

```cpp
EncoderState* resolveEncoderState(const char* name) {
#if ENABLE_DRIVE_ENCODERS
  if (strcmp(name, "drive_front_left") == 0) {
    return &encoderDriveFrontLeft;
  }

  if (strcmp(name, "drive_front_right") == 0) {
    return &encoderDriveFrontRight;
  }
#endif

#if ENABLE_DEADWHEEL_ENCODERS
  if (strcmp(name, "deadwheel_parallel") == 0) {
    return &encoderDeadwheelParallel;
  }

  if (strcmp(name, "deadwheel_perpendicular") == 0) {
    return &encoderDeadwheelPerpendicular;
  }
#endif

#if ENABLE_SHOOTER_ENCODER
  if (strcmp(name, "shooter") == 0) {
    return &encoderShooter;
  }
#endif

  return nullptr;
}
```

### Unified Protocol Handler

```cpp
bool handleEncoderCommand(char* line);
```

This handler should own the complete encoder command family, such as:

```text
ENCODER drive_front_left READ
ENCODER drive_front_right READ
ENCODER deadwheel_parallel READ
ENCODER deadwheel_perpendicular READ
ENCODER shooter READ
```

---

## 11. `battery.ino`

This file owns everything required to read, interpret, test and report battery voltage.

### Move Into This File

- battery ADC conversion;
- voltage-divider ratio;
- filtering or averaging;
- last sampled voltage;
- warning thresholds;
- critical thresholds;
- shutdown warning state;
- battery alarm level;
- battery-specific command parsing.

Typical constants:

```cpp
static const float BATTERY_DIVIDER_RATIO = 5.0f;

static const float BATTERY_LOW_WARN_V = 12.0f;
static const float BATTERY_CRITICAL_WARN_V = 11.5f;
static const float BATTERY_SHUTDOWN_NOW_V = 11.0f;
```

Typical functions:

```cpp
void setupBatteryMonitor();
void serviceBatteryMonitor();

float readBatteryVoltage();
uint8_t getBatteryAlarmLevel();

bool handleBatteryCommand(char* line);
```

### Main File Retains

- battery ADC pin assignment;
- feature switch.

The Pi protocol should call `readBatteryVoltage()` rather than repeat the ADC conversion.

---

## 12. `sensors.ino`

Initially, this file can own the remaining sensor and input functions.

Recommended internal sections:

```text
DIGITAL INPUTS
LIMIT SWITCHES
BUMPERS
REFLECTANCE
ULTRASONIC
OTHER ANALOG INPUTS
SENSOR COMMAND HANDLING
```

This prevents the first refactor from creating too many tiny files.

### Digital Inputs

Examples:

- lift high limit;
- lift low limit;
- start button;
- Pi/Arduino selector;
- front-left bumper;
- front-right bumper.

Prefer named functions:

```cpp
bool isLiftHighLimitActive();
bool isLiftLowLimitActive();

bool isStartButtonPressed();

bool isFrontLeftBumperActive();
bool isFrontRightBumperActive();

bool isPiControlSelected();
```

This keeps electrical polarity decisions inside the sensor module.

### Reflectance Sensors

Typical functions:

```cpp
uint16_t readReflectanceLeft();
uint16_t readReflectanceCentre();
uint16_t readReflectanceRight();

bool readReflectanceByName(
    const char* name,
    uint16_t& value
);
```

### Ultrasonic Sensors

Typical functions:

```cpp
void setupUltrasonics();

long readRangePair(
    uint8_t triggerPin,
    uint8_t echoPin
);

long readUltrasonicByName(const char* name);
```

Later, this section may grow to include:

- non-blocking trigger sequencing;
- echo timing state;
- validity;
- timeout handling;
- filtering;
- scheduled updates.

### Sensor Command Handling

```cpp
bool handleSensorCommand(char* line);
```

This handler should route named sensor reads to the correct functional section.

---

## 13. Optional Later Splits

When a section becomes substantial, split it into its own file.

### `digital_inputs.ino`

Possible contents:

- limit switches;
- bumpers;
- start button;
- control selector;
- active-high and active-low interpretation;
- digital input command handling.

### `reflectance.ino`

Possible contents:

- raw ADC readings;
- calibration;
- thresholds;
- normalized reflectance values;
- line detection;
- reflectance command handling.

### `ultrasonics.ino`

Possible contents:

- trigger scheduling;
- echo capture;
- non-blocking state machines;
- distance conversion;
- filtering;
- validity;
- ultrasonic command handling.

### `indicators.ino`

Possible contents:

- piezo buzzer;
- status LEDs;
- Lisiparoi;
- DFPlayer selection or control;
- battery warning indication;
- control-source indication;
- fault indication.

A clean responsibility boundary is:

```text
battery.ino
    determines battery state

indicators.ino
    displays or sounds battery state
```

---

## 14. Minimal Module Integration Contract

Each functional file should expose only the functions it actually requires from this common pattern:

```cpp
setupFeature();
serviceFeature();
handleFeatureCommand(char* line);
stopFeature();
```

Examples:

```cpp
// encoders.ino
void setupEncoders();
void serviceEncoders();
bool handleEncoderCommand(char* line);
```

```cpp
// battery.ino
void setupBatteryMonitor();
void serviceBatteryMonitor();
bool handleBatteryCommand(char* line);
```

```cpp
// servos.ino
void setupServos();
void serviceServos();
bool handleServoCommand(char* line);
void stopServos();
```

```cpp
// drive_roboclaw.ino
void setupDrive();
bool handleDriveCommand(char* line);
void stopDrive();
```

Not every feature requires every function.

Examples:

- timer-driven encoders may need setup and command handling, while acquisition runs in an ISR;
- battery monitoring may need setup, service and command handling;
- requested ultrasonic reads may initially need setup and command handling only;
- drive control needs setup, command handling and stop behaviour;
- Pi protocol needs setup and service but does not dispatch to itself.

---

## 15. Ownership Summary

| File | Owns |
|---|---|
| `Mega2560.ino` | Feature selection, all pins, serial assignments, robot composition, system-wide state, `setup()`, `loop()` |
| `pi_protocol.ino` | Pi serial line handling, heartbeat, control mode and command dispatch |
| `drive_roboclaw.ino` | RoboClaw protocol, drive motor outputs, mecanum mixing and drive stop |
| `motors_pwm.ino` | Shooter and collector motor control |
| `servos.ino` | Gripper, lift and shooter-feed servo behaviour |
| `flysky_ibus.ino` | iBus frame reception, channel conversion and manual command extraction |
| `encoders.ino` | All encoder groups, acquisition, counting, snapshots, diagnostics and encoder commands |
| `battery.ino` | Battery ADC conversion, thresholds, state and battery commands |
| `sensors.ino` | Ultrasonic, reflectance, limit switches, bumpers and remaining sensor commands |
| `indicators.ino` | Optional later split for buzzer, lighting and status indication |

---

## 16. Intended Final Appearance of `Mega2560.ino`

The main file should eventually be short enough to scan as a robot overview.

```cpp
// =========================================================
// DOCUMENTATION AND LIBRARIES
// =========================================================

// =========================================================
// FEATURE SELECTION
// =========================================================

// =========================================================
// SERIAL ASSIGNMENTS
// =========================================================

// =========================================================
// PHYSICAL PIN ASSIGNMENTS
// =========================================================

// =========================================================
// LOGICAL ENDPOINT NAMES
// =========================================================

// =========================================================
// SYSTEM-WIDE STATE
// =========================================================

void setup() {
  setupCoreCommunications();

#if ENABLE_DRIVE
  setupDrive();
#endif

#if ENABLE_PWM_MOTORS
  setupPwmMotors();
#endif

#if ENABLE_SERVOS
  setupServos();
#endif

#if ENABLE_FLYSKY_IBUS
  setupFlySkyIbus();
#endif

#if ENABLE_ENCODERS
  setupEncoders();
#endif

#if ENABLE_BATTERY_MONITOR
  setupBatteryMonitor();
#endif

#if ENABLE_SENSORS
  setupSensors();
#endif

#if ENABLE_INDICATORS
  setupIndicators();
#endif
}

void loop() {
#if ENABLE_PI_PROTOCOL
  servicePiSerial();
#endif

#if ENABLE_FLYSKY_IBUS
  serviceFlySkyIbus();
#endif

#if ENABLE_ENCODERS
  serviceEncoders();
#endif

#if ENABLE_BATTERY_MONITOR
  serviceBatteryMonitor();
#endif

#if ENABLE_SERVOS
  serviceServos();
#endif

#if ENABLE_INDICATORS
  serviceIndicators();
#endif

  updateControlSource();
  applyRobotControl();
}
```

This provides two levels of understanding:

```text
Open Mega2560.ino
    understand the complete robot

Open one functional .ino file
    develop, test, disable or remove one capability
```

---

## 17. Recommended Refactoring Sequence

Refactor incrementally rather than moving the entire sketch at once.

Recommended sequence:

1. Add the feature-selection section.
2. Create `battery.ino`.
3. Create `encoders.ino`.
4. Create `flysky_ibus.ino`.
5. Create `drive_roboclaw.ino`.
6. Create `motors_pwm.ino`.
7. Create `servos.ino`.
8. Create `sensors.ino`.
9. Reduce `pi_protocol.ino` to transport and dispatch.
10. Simplify `setup()`.
11. Simplify `loop()`.
12. Compile and test after every extraction.

For each feature:

1. move its state;
2. move its calibration;
3. move its setup;
4. move its implementation;
5. move its command handling;
6. add its guarded integration calls;
7. compile;
8. run the existing hardware checkout or diagnostic;
9. only then move to the next feature.

This avoids changing architecture and behaviour across the whole firmware simultaneously.

---

## 18. Verification

After each module is extracted, verify:

- the sketch still compiles;
- no duplicate function or variable definitions exist;
- no required state was left behind;
- no feature code runs when its flag is disabled;
- all existing serial commands still respond correctly;
- the feature still passes its physical checkout;
- disabling the feature removes its setup, servicing and command handling;
- unrelated robot functions remain unchanged.

For a disabled module, verify both:

```cpp
#define ENABLE_FEATURE 0
```

and, where intended, complete deletion of the feature `.ino` file.

Every reference to a deletable module must be enclosed by the matching compile-time guard.

Example:

```cpp
#if ENABLE_ENCODERS
  setupEncoders();
#endif
```

```cpp
#if ENABLE_ENCODERS
  serviceEncoders();
#endif
```

```cpp
#if ENABLE_ENCODERS
  if (handleEncoderCommand(line)) {
    return;
  }
#endif
```

---

## 19. Next Step

Use this document as the master guide while splitting the current `Mega2560.ino`.

The immediate next task is to establish the feature-selection section and create the first functional tab without changing behaviour.

A suitable first extraction is `battery.ino`, followed by `encoders.ino`.

The encoder implementation itself should continue to follow the dedicated drive-wheel encoder design document.
