# HiWonder Board A Controller Reference Manual

**Document Version:** 0.1 (Draft)

**Status:** In Development

---

## Revision History

| Version | Date | Description |
|----------|------|-------------|
| 0.1 | Initial Draft | Hardware architecture, platform overview and engineering reference established. |

---

# 1. Scope

This document provides an engineering reference for the HiWonder **Board A Controller**, the embedded controller used throughout HiWonder's Raspberry Pi robotics ecosystem.

Unlike the official tutorials, which are organised around individual demonstrations, this document describes the controller as an engineering platform, including:

- Hardware architecture
- Communication architecture
- Electrical interfaces
- UART protocol
- STM32 controller functions
- Python SDK
- Bus servo interface
- Motor interface
- GPIO and I²C architecture
- Reverse engineering observations
- Cross-platform compatibility

The intention is to provide a single reference suitable for developing custom robotics software independently of the supplied demonstration programs.

---

# 2. Board Overview

Board A is a general-purpose robotics controller designed by HiWonder for Raspberry Pi based robots.

Rather than directly controlling motors and servos from Linux, the Raspberry Pi communicates with an onboard 32-bit STM32 microcontroller over a high-speed UART.

The STM32 performs all deterministic real-time control while the Raspberry Pi executes high-level software including:

- Python applications
- OpenCV
- AI inference
- ROS
- Motion planning
- User applications

This architecture separates deterministic hardware timing from non-real-time Linux execution.

---

## 2.1 Design Philosophy

The controller should be viewed as a reusable robotics platform rather than an ArmPi-specific controller.

Evidence from the official documentation shows that Board A is used across multiple HiWonder products including:

- ArmPi FPV
- ArmPi Pro
- ArmPi Mini
- TurboPi
- MasterPi
- GoGoPi
- additional Raspberry Pi based products

Each robot uses the same fundamental controller architecture while differing primarily in:

- mechanical structure
- sensors
- software
- attached peripherals

---

## 2.2 Raspberry Pi Compatibility

Current documentation states that Board A is compatible with both Raspberry Pi 4B and Raspberry Pi 5.

The electrical interfaces remain identical.

The primary physical difference is mechanical accommodation of the Raspberry Pi 5 active cooling solution.

Engineering observations indicate that the underlying STM32 controller architecture appears unchanged.

---

# 3. Hardware Architecture

The controller architecture is shown below.

                    Raspberry Pi

                High-Level Software
               (Python / ROS / AI)

                        │

                 UART (1 Mbit/s)

                        │

          ┌──────────────────────────────┐
          │      STM32 Controller        │
          └──────────────────────────────┘

             │        │        │
             │        │        │
        Bus Servos  PWM     DC Motors

             │
        RGB LEDs

             │
          Buzzer

             │
          Buttons

             │
      Battery Monitoring

──────────────────────────────────────────────

Direct Raspberry Pi Interfaces

GPIO
I²C

These interfaces bypass the STM32 entirely and are connected directly to the Raspberry Pi.

---

## 3.1 Functional Separation

The controller intentionally separates hardware into two categories.

### STM32 Controlled

The following peripherals are managed by the STM32 firmware:

- Bus servos
- PWM servos
- DC motors
- RGB LEDs
- Buzzer
- User buttons
- Battery voltage
- IMU (where fitted)
- Gamepad interface
- SBUS receiver
- OLED display

Communication with these peripherals always occurs via the UART protocol.

---

### Raspberry Pi Controlled

The following interfaces are directly connected to the Raspberry Pi.

- GPIO
- I²C

Applications communicate with these interfaces using standard Linux libraries such as:

- gpiod
- smbus
- smbus2

The STM32 is not involved in these transactions.

---

# 4. Hardware Resources

Board A integrates the following resources.

| Resource | Quantity | Controller | Notes |
|----------|---------:|------------|------|
| Bus Servo Ports | 2 | STM32 | TTL serial bus |
| PWM Servo Outputs | 6 | STM32 | Standard RC PWM |
| DC Motor Outputs | 4 | STM32 | PWM motor driver |
| RGB LEDs | 2 | STM32 | Individually addressable |
| Buzzer | 1 | STM32 | Frequency programmable |
| User Buttons | 2 | STM32 | Event driven |
| I²C Connectors | 3 | Raspberry Pi | Direct Pi I²C |
| GPIO Connector | 1 | Raspberry Pi | Direct GPIO |
| Power Input | 1 | Board | Screw terminal |
| Power Switch | 1 | Board | Master power |
| Status LED | 1 | STM32 | Controller status |

---

## 4.1 Controller MCU

Board A uses a 32-bit ARM STM32 microcontroller.

Responsibilities include:

- UART protocol processing
- Motor PWM generation
- Bus servo communication
- PWM servo generation
- RGB LED control
- Buzzer generation
- Button event detection
- Battery monitoring
- Optional IMU acquisition
- Optional gamepad handling

The Raspberry Pi never directly generates motor or servo timing.

All deterministic timing is delegated to the STM32.

# 5. Hardware Description

## 5.1 Controller Overview

Board A is centred around a 32-bit STM32 ARM microcontroller acting as a dedicated real-time peripheral controller.

Unlike many Raspberry Pi robotics platforms, the Raspberry Pi does not directly generate servo PWM signals or motor control waveforms. Instead, all real-time hardware control is delegated to the STM32, allowing Linux to concentrate on high-level software.

Typical responsibilities of the Raspberry Pi include:

- Python applications
- Artificial intelligence
- Computer vision
- Motion planning
- Inverse kinematics
- User interface
- Networking

Typical responsibilities of the STM32 include:

- DC motor PWM generation
- Bus servo communication
- PWM servo timing
- RGB LED generation
- Buzzer generation
- Button event detection
- Battery monitoring
- Optional IMU acquisition
- Optional gamepad processing

This separation greatly reduces timing jitter and removes the need for Linux to perform deterministic real-time tasks.

---

## 5.2 Data Flow

A typical command sequence is illustrated below.

Application

↓

Python SDK

↓

UART Packet

↓

STM32 Firmware

↓

Peripheral Driver

↓

Motor / Servo / LED / Buzzer

Likewise, hardware events follow the reverse path.

Button Press

↓

STM32 Firmware

↓

UART Report Packet

↓

Python SDK

↓

Application

The Raspberry Pi therefore interacts with peripherals at the command level rather than directly manipulating hardware registers.

---

## 5.3 Controller Responsibilities

The STM32 should be viewed as a hardware abstraction layer rather than a simple motor driver.

Its firmware performs:

- packet parsing
- command validation
- checksum verification
- peripheral scheduling
- servo protocol generation
- PWM generation
- event reporting

The Raspberry Pi communicates exclusively through the published packet interface.

---

# 6. Hardware Resources

This section describes each hardware resource integrated into Board A.

---

## 6.1 Bus Servo Interface

Quantity

- Two connectors

Controller

- STM32

Purpose

Provides communication with HiWonder serial bus servos including:

- LX-15D
- LX-225
- HTS-16L
- HTS-25L
- HX-06L

Capabilities

- Position control
- Torque enable/disable
- Servo ID programming
- Position feedback
- Voltage feedback
- Temperature feedback
- Offset calibration
- Angle limits
- Voltage limits
- Temperature limits

Engineering Notes

The STM32 performs all bus timing.

The Raspberry Pi does not communicate directly with the servos.

---

## 6.2 PWM Servo Outputs

Quantity

- Six outputs

Controller

- STM32

Purpose

Provides standard hobby servo PWM outputs.

Capabilities

- Absolute pulse width positioning
- Offset calibration
- Position feedback (where supported)

Typical use

- Pan/tilt mechanisms
- Grippers
- Auxiliary actuators

---

## 6.3 DC Motor Outputs

Quantity

- Four outputs

Controller

- STM32

Purpose

Provides PWM motor control for brushed DC motors.

SDK Functions

- Duty-cycle control
- Velocity control (firmware supported)

Typical applications

- Differential drive
- Mecanum drive
- Omni drive

Engineering Note

The SDK contains interfaces for both duty-cycle control and floating-point speed commands, suggesting the firmware supports more than simple PWM output.

---

## 6.4 RGB LEDs

Quantity

- Two

Controller

- STM32

Capabilities

- Independent RGB colour
- Multiple colours
- Software control

Typical uses

- Robot status
- Diagnostics
- User feedback

---

## 6.5 Buzzer

Quantity

- One

Controller

- STM32

Capabilities

- Programmable frequency
- Programmable on-time
- Programmable off-time
- Continuous repeat mode

Applications

- Status indication
- Warnings
- Audible diagnostics

---

## 6.6 User Buttons

Quantity

- Two

Controller

- STM32

Capabilities

- Press detection
- Click
- Double click
- Triple click
- Long press
- Long press repeat

Engineering Note

The SDK exposes button events rather than GPIO state, indicating the STM32 performs event detection internally.

---

## 6.7 Battery Monitoring

Controller

- STM32

Capabilities

- Battery voltage measurement
- Voltage reporting to host

Engineering Note

The battery voltage is reported through the System packet interface rather than via an analogue input on the Raspberry Pi.

---

## 6.8 GPIO Interface

Controller

- Raspberry Pi

Access Method

Linux gpiod library

Characteristics

The GPIO connector is a direct breakout of Raspberry Pi GPIO pins.

No STM32 firmware involvement is required.

Applications communicate directly with Linux GPIO drivers.

---

## 6.9 I²C Interface

Controller

- Raspberry Pi

Access Method

Linux smbus / smbus2

Characteristics

The I²C connectors are direct Raspberry Pi I²C interfaces.

Example peripherals include:

- Ultrasonic module
- Four-channel line sensor
- OLED displays
- IMUs
- Additional sensors

Engineering Note

Example software accesses these devices directly using Linux I²C libraries without passing through the STM32 firmware.

---

## 6.10 Optional Peripherals

Depending upon the robot platform, Board A firmware may also support:

- IMU
- Gamepad receiver
- SBUS receiver
- OLED display

These devices are accessed using the same UART packet protocol as the remaining STM32 peripherals.

---

# Part III - Communication Architecture

# 7. Communication Architecture

## 7.1 Overview

Board A uses a layered communication architecture that separates high-level application software from real-time peripheral control.

The Raspberry Pi never directly manipulates servo buses, motor drivers or PWM outputs. Instead, all communication occurs through a packet-based UART protocol implemented by the STM32 firmware.

The overall architecture is illustrated below.

```
                User Application
                     │
                     ▼
             Python Application
                     │
                     ▼
      ros_robot_controller_sdk.py
                     │
                     ▼
             UART (1,000,000 baud)
                     │
                     ▼
             STM32 Firmware
                     │
      ┌──────────────┼──────────────┐
      ▼              ▼              ▼
 Bus Servos      PWM Servos     DC Motors
      │              │              │
      ├──────────────┼──────────────┤
      ▼              ▼              ▼
 RGB LEDs        Buzzer        Board Services
```

This architecture provides a clear separation between:

- Non-deterministic Linux software
- Deterministic real-time hardware control

---

## 7.2 Why an STM32?

Linux is not a hard real-time operating system.

Tasks such as:

- Servo pulse generation
- Motor PWM
- Serial bus timing
- RGB LED timing

require microsecond-level timing guarantees that cannot reliably be achieved from user-space Python.

Board A therefore delegates these functions to the STM32.

The Raspberry Pi instead issues high-level commands such as:

```

Move Servo 6 to Position 500

```

rather than generating the individual timing waveforms.

---

## 7.3 Communication Layers

The software stack can be viewed as five distinct layers.

| Layer | Responsibility |
|---------|----------------|
| Application | Robot logic |
| SDK | Packet construction and parsing |
| UART | Reliable byte transport |
| STM32 Firmware | Command execution |
| Hardware | Physical peripherals |

Each layer has a clearly defined responsibility.

---

## 7.4 UART Configuration

Communication between the Raspberry Pi and STM32 uses the Raspberry Pi's primary UART.

| Parameter | Value |
|-----------|-------|
| Interface | UART |
| Device | `/dev/ttyAMA0` |
| Baud Rate | 1,000,000 bit/s |
| Data Bits | 8 |
| Parity | None |
| Stop Bits | 1 |
| Flow Control | None |

This configuration is established automatically by the Python SDK.

---

## 7.5 Packet-Oriented Protocol

The UART is **not** a stream of ASCII commands.

Instead, every transaction consists of a binary packet.

General format:

```

AA 55
Function
Length
Payload
CRC8

```

The fixed header allows packet synchronisation even if bytes are lost.

The CRC provides packet integrity.

Payload interpretation depends upon the Function ID.

---

## 7.6 Request / Response Model

Board A implements both command packets and report packets.

Examples of commands include:

- Move servo
- Set RGB colour
- Enable buzzer
- Set motor duty cycle

Examples of reports include:

- Battery voltage
- Button events
- IMU measurements
- Servo feedback
- Gamepad status

This allows communication to be bi-directional.

---

## 7.7 Packet Lifetime

A typical command follows the sequence below.

```

Python Application

↓

SDK constructs payload

↓

CRC calculated

↓

UART transmission

↓

STM32 validates CRC

↓

STM32 executes command

↓

Peripheral updated

```

For commands requiring feedback, an additional packet is generated by the STM32 and returned to the Raspberry Pi.

---

# 8. Packet Structure

## 8.1 Overview

Every UART transaction uses the same packet format.

```

Byte 0    0xAA

Byte 1    0x55

Byte 2    Function ID

Byte 3    Payload Length

Byte 4... Payload

Final     CRC8

```

The protocol therefore has a fixed framing mechanism regardless of command type.

---

## 8.2 Packet Header

The first two bytes are constant.

| Byte | Value | Purpose |
|------|-------|----------|
| 0 | 0xAA | Synchronisation |
| 1 | 0x55 | Synchronisation |

These values allow the receiver to locate the beginning of a packet.

If communication becomes misaligned, the parser searches for this header before attempting to decode additional packets.

---

## 8.3 Function ID

The third byte identifies the subsystem.

| Function ID | Peripheral |
|-------------|------------|
| 0 | System |
| 1 | LED |
| 2 | Buzzer |
| 3 | Motor |
| 4 | PWM Servo |
| 5 | Bus Servo |
| 6 | Key |
| 7 | IMU |
| 8 | Gamepad |
| 9 | SBUS |
| 10 | OLED |
| 11 | RGB |

Each function defines its own payload format.

---

## 8.4 Payload Length

The Length byte specifies the number of payload bytes that follow.

This allows packets of varying size while maintaining a common framing format.

---

## 8.5 Payload

Payload contents are determined entirely by the Function ID.

Examples include:

- Motor speeds
- Servo positions
- RGB values
- Voltage readings
- IMU data

No payload interpretation is performed until after the Function ID has been decoded.

---

## 8.6 CRC8

Every packet terminates with an 8-bit cyclic redundancy check.

The CRC is calculated over all packet bytes excluding the CRC byte itself.

The STM32 rejects packets with an invalid CRC.

Likewise, the SDK ignores received packets whose CRC validation fails.

This mechanism provides protection against corrupted UART transmissions.

---

# Part IV - SDK API Reference

# 9. SDK Architecture

## 9.1 Overview

The official Python SDK provides a high-level interface between user applications and the Board A controller.

Applications do not construct UART packets directly. Instead, they interact with the `Board` class, which encapsulates packet construction, transmission, reception and response parsing.

The overall software hierarchy is shown below.

```
Application

↓

Board Class

↓

Packet Encoder

↓

UART Driver

↓

STM32 Firmware

↓

Hardware
```

This layered architecture isolates user applications from the underlying communication protocol.

---

## 9.2 The Board Class

Almost all interaction with Board A begins by creating an instance of the `Board` class.

Example:

```python
import ros_robot_controller_sdk as rrc

board = rrc.Board()
```

During initialisation the SDK:

- Opens the UART interface.
- Configures the serial port.
- Starts the receive thread.
- Initialises internal packet queues.
- Synchronises communication with the STM32.

Once initialised, the `Board` object becomes the primary interface for all hardware resources.

---

## 9.3 Thread Architecture

The SDK creates a dedicated receive thread responsible for continuously monitoring the UART.

Responsibilities include:

- Receiving incoming bytes.
- Detecting packet headers.
- Verifying packet integrity.
- Parsing payloads.
- Routing reports to the appropriate queues.

Applications therefore interact with complete messages rather than raw serial data.

---

# 10. System Functions

Function ID

```
0
```

Purpose

Provides access to controller-level information including battery monitoring and board status.

---

## 10.1 Battery Voltage

Description

Returns the measured battery supply voltage.

Typical use cases include:

- Low battery warning.
- Telemetry.
- Safe shutdown.
- Power diagnostics.

Return type

Floating point voltage value.

Engineering Note

Battery voltage is measured by the STM32 and transmitted to the Raspberry Pi as a report packet.

The Raspberry Pi does not measure battery voltage directly.

---

# 11. Status LED

Function ID

```
1
```

Purpose

Controls the onboard status LED.

Typical uses

- Startup indication.
- Error reporting.
- Heartbeat indication.
- User feedback.

---

# 12. Buzzer

Function ID

```
2
```

Purpose

Controls the onboard piezoelectric buzzer.

---

## 12.1 SDK Function

```python
board.set_buzzer(
    frequency,
    on_time,
    off_time,
    repeat
)
```

### Parameters

| Parameter | Description |
|------------|-------------|
| frequency | Frequency in Hertz |
| on_time | On duration (seconds) |
| off_time | Off duration (seconds) |
| repeat | Number of repetitions |

---

### Special Behaviour

A repeat value of

```
0
```

causes the buzzer sequence to repeat continuously until another command disables it.

---

### Stop Command

```python
board.set_buzzer(
    1000,
    0.0,
    0.0,
    1
)
```

is used to silence the buzzer.

---

### Example

Single confirmation beep.

```python
board.set_buzzer(
    1900,
    0.1,
    0.9,
    1
)
```

Continuous warning tone.

```python
board.set_buzzer(
    1000,
    0.5,
    0.5,
    0
)
```

---

### Engineering Observation

The SDK converts time values from seconds into integer milliseconds before packet transmission.

---

# 13. RGB LEDs

Function ID

```
11
```

Purpose

Controls the onboard WS2812 RGB LEDs.

---

## 13.1 SDK Function

The SDK provides methods for independently controlling each RGB LED.

Typical parameters include:

- LED index.
- Red intensity.
- Green intensity.
- Blue intensity.

---

### Typical Uses

- Robot state indication.
- Debugging.
- User interaction.
- Battery indication.
- Operating mode display.

---

### Engineering Observation

Colour generation is performed entirely by the STM32.

The Raspberry Pi simply transmits colour values.

---

# 14. DC Motors

Function ID

```
3
```

Purpose

Controls up to four brushed DC motor channels.

---

## 14.1 SDK Functions

The SDK exposes interfaces for:

- Duty-cycle control.
- Speed control.

---

### Typical Applications

- Differential drive.
- Mecanum drive.
- Omni drive.
- Conveyor mechanisms.
- Linear actuators.

---

### Engineering Observation

The SDK supports floating-point speed commands in addition to raw PWM values.

This suggests firmware support beyond simple open-loop PWM generation.

---

# 15. PWM Servos

Function ID

```
4
```

Purpose

Controls standard hobby PWM servos.

---

## Supported Features

- Absolute position.
- Multiple servo updates.
- Offset adjustment.

---

### Typical Applications

- Camera pan.
- Camera tilt.
- Grippers.
- Auxiliary mechanisms.

---

### Engineering Observation

Servo pulse generation is entirely performed by the STM32.

Linux timing has no effect on PWM quality.

---

# 16. Bus Servos

Function ID

```
5
```

Purpose

Controls HiWonder intelligent serial bus servos.

This is the most feature-rich subsystem provided by Board A.

Supported operations include:

- Position control.
- Position readback.
- Temperature monitoring.
- Voltage monitoring.
- ID programming.
- Offset calibration.
- Angle limits.
- Voltage limits.
- Temperature limits.
- Torque enable/disable.
- Servo stop.
- Servo unload.

The Bus Servo subsystem is documented in detail in Appendix B.

---

# Part V - Protocol Reference

# 17. Protocol Function Reference

## 17.1 Overview

Communication between the Raspberry Pi and the Board A controller is organised into functional subsystems.

Each UART packet contains a **Function ID** that identifies the target subsystem. The STM32 firmware dispatches incoming packets according to this identifier before interpreting the packet payload.

This design provides a modular protocol in which each peripheral is responsible only for decoding its own command set.

The currently identified Function IDs are shown below.

| Function ID | Subsystem | Direction | Purpose |
|------------:|-----------|-----------|---------|
| 0 | System | Request / Response | Board status and battery monitoring |
| 1 | Status LED | Command | On-board LED control |
| 2 | Buzzer | Command | Audible indication |
| 3 | DC Motors | Command | Brushed DC motor control |
| 4 | PWM Servos | Command / Response | Standard RC servo control |
| 5 | Bus Servos | Command / Response | Intelligent serial servo interface |
| 6 | Keys | Report | User button events |
| 7 | IMU | Report | Inertial measurement data |
| 8 | Gamepad | Report | Wireless controller interface |
| 9 | SBUS | Report | Radio receiver interface |
| 10 | OLED | Command | OLED display control |
| 11 | RGB LEDs | Command | Addressable RGB LEDs |

The following sections describe each subsystem individually.

---

# 18. System Functions

## Function ID

```
0
```

### Purpose

Provides access to controller-level services and system status.

Unlike other Function IDs, System commands are not associated with a physical peripheral. Instead they provide information describing the operating condition of the controller itself.

---

## Current Capabilities

The SDK currently exposes:

- Battery voltage measurement
- System status reporting

Future firmware revisions may extend this function with additional controller diagnostics.

---

## Direction

Request

↓

STM32

↓

Response

---

## Engineering Notes

Battery voltage is measured by analogue circuitry connected to the STM32.

The Raspberry Pi does not directly measure battery voltage.

Applications should therefore regard the reported value as the authoritative system supply voltage.

---

# 19. Status LED

## Function ID

```
1
```

### Purpose

Controls the on-board status LED.

---

### Typical Applications

- System startup
- Heartbeat indication
- Error signalling
- User feedback

---

### Communication Direction

Command only

---

### Engineering Notes

LED timing is generated locally by the STM32.

The Raspberry Pi merely issues the desired operating state.

---

# 20. Buzzer

## Function ID

```
2
```

### Purpose

Controls the on-board piezoelectric sounder.

---

### Supported Operations

- Start tone
- Stop tone
- Continuous tone
- Repeating tone

---

### Parameters

| Parameter | Description |
|------------|-------------|
| Frequency | Tone frequency (Hz) |
| On Time | Tone duration |
| Off Time | Silent interval |
| Repeat | Number of repetitions |

---

### Continuous Mode

A repeat count of

```
0
```

produces continuous operation until another command disables the buzzer.

---

### Typical Applications

- Startup confirmation
- Error indication
- Low battery warning
- Debug notifications

---

### Engineering Notes

The SDK converts floating-point seconds into integer milliseconds before transmission.

Timing accuracy is therefore determined by the STM32 firmware.

---

# 21. DC Motor Controller

## Function ID

```
3
```

### Purpose

Provides control of up to four brushed DC motors.

---

### Supported Operations

The firmware supports:

- Duty-cycle control
- Speed control
- Simultaneous motor updates

---

### Typical Applications

- Differential drive
- Omni drive
- Mecanum drive
- Conveyor systems
- Linear actuators

---

### Engineering Notes

Motor PWM generation is completely independent of Linux timing.

This architecture avoids the timing jitter that would otherwise occur if PWM were generated directly by the Raspberry Pi.

---

# 22. PWM Servo Controller

## Function ID

```
4
```

### Purpose

Controls standard hobby RC servos.

---

### Supported Operations

- Absolute positioning
- Multi-servo update
- Position offset
- Position feedback

---

### Typical Applications

- Camera pan/tilt
- Auxiliary mechanisms
- Small manipulators
- Grippers

---

### Engineering Notes

Servo pulse generation occurs entirely within the STM32.

The Raspberry Pi issues position commands only.

---

# 23. Bus Servo Controller

## Function ID

```
5
```

### Purpose

Provides communication with HiWonder intelligent serial bus servos.

This subsystem represents the most capable component of the Board A controller.

---

### Supported Operations

Motion

- Move
- Stop

Configuration

- Set ID
- Set Offset
- Save Offset

Monitoring

- Position
- Temperature
- Supply Voltage
- Load Status

Protection

- Angle Limits
- Voltage Limits
- Temperature Limits

Power

- Torque Enable
- Torque Disable

---

### Supported Servo Families

Current SDK support includes:

- LX-15D
- LX-225
- HTS-16L
- HTS-25L
- HX-06L

Future compatible servos may also operate provided they implement the same communication protocol.

---

### Engineering Notes

Unlike PWM servos, Bus Servos provide closed-loop feedback.

This enables the Raspberry Pi to query actual servo state rather than assuming commanded motion has completed.

The complete Bus Servo command set is documented in Appendix B.

---

# 24. Key Controller

## Function ID

```
6
```

### Purpose

Reports user button activity.

---

### Event Types

The firmware recognises multiple button actions including:

- Press
- Release
- Click
- Double Click
- Triple Click
- Long Press
- Long Press Repeat

Applications receive decoded events rather than raw switch states.

---

### Engineering Notes

The STM32 performs all timing required to distinguish between click patterns.

Consequently the Raspberry Pi does not implement button debouncing.

---

# 25. IMU

## Function ID

```
7
```

### Purpose

Provides inertial measurement data from the onboard IMU when fitted.

---

### Typical Measurements

- Acceleration
- Angular velocity
- Orientation

---

### Engineering Notes

Support depends upon the specific robot configuration.

Not all Board A based robots include an IMU.

---

# 26. Gamepad

## Function ID

```
8
```

### Purpose

Provides communication with supported wireless game controllers.

The STM32 reports decoded controller state to the Raspberry Pi.

---

# 27. SBUS

## Function ID

```
9
```

### Purpose

Provides support for SBUS-compatible radio receivers.

---

### Applications

- Manual robot control
- Mixed autonomy
- Radio override

---

# 28. OLED Display

## Function ID

```
10
```

### Purpose

Provides firmware support for optional OLED displays.

Support is dependent upon the target robot platform.

---

# 29. RGB LEDs

## Function ID

```
11
```

### Purpose

Controls individually addressable RGB LEDs connected to the controller.

---

### Supported Operations

- Set colour
- Update individual LEDs
- Multiple LED updates

---

### Typical Applications

- Status indication
- Battery level
- Operating mode
- Diagnostics
- User interaction

---

### Engineering Notes

RGB timing is generated entirely by the STM32 firmware.

The Raspberry Pi transmits only colour values.

---

# Part VI - Intelligent Bus Servo System

# 30. Intelligent Bus Servo Architecture

## 30.1 Overview

One of the defining features of Board A is its support for intelligent serial bus servos.

Unlike conventional hobby servos, which receive only a PWM position signal, intelligent bus servos incorporate an onboard microcontroller and communicate digitally over a shared serial bus.

This architecture enables:

- Closed-loop position control
- Position feedback
- Temperature monitoring
- Supply voltage monitoring
- Configuration storage
- Servo identification
- Software protection limits
- Multi-servo communication

Each servo operates as an intelligent node on a common communication bus.

---

## 30.2 System Architecture

The complete control path is illustrated below.

```
Python Application

↓

Board SDK

↓

UART (1 Mbit/s)

↓

STM32 Controller

↓

TTL Bus Servo Interface

↓

Shared Servo Bus

├── Servo ID 1
├── Servo ID 2
├── Servo ID 3
├── Servo ID 4
└── ...
```

Only one communication cable is required regardless of the number of servos connected.

---

## 30.3 Servo Intelligence

Each intelligent servo contains:

- DC motor
- Reduction gearbox
- Position sensor
- Motor driver
- Microcontroller
- EEPROM configuration memory
- Protection circuitry

The servo therefore performs its own closed-loop position control internally.

The STM32 issues motion commands but does not directly regulate motor current or position.

---

## 30.4 Addressing

Every servo possesses a unique numerical identifier.

Commands are directed to individual servos using this ID.

Example

```
Move Servo 6

↓

Servo 6 executes

↓

All other servos ignore the command
```

This allows multiple servos to share a common communication bus without conflict.

---

## 30.5 Multi-Drop Network

Unlike PWM servos, intelligent servos are connected in parallel.

```
STM32
   │
   ├────────────── Servo 1
   │
   ├────────────── Servo 2
   │
   ├────────────── Servo 3
   │
   └────────────── Servo 4
```

The STM32 acts as the master.

Servos operate exclusively as slave devices.

Only the addressed servo responds.

---

## 30.6 Advantages over PWM Servos

| Feature | PWM Servo | Intelligent Bus Servo |
|----------|-----------|----------------------|
| Position Command | Yes | Yes |
| Position Feedback | No | Yes |
| Temperature | No | Yes |
| Supply Voltage | No | Yes |
| Unique Address | No | Yes |
| EEPROM Configuration | No | Yes |
| Software Limits | No | Yes |
| Shared Bus | No | Yes |
| Diagnostic Information | No | Yes |

The intelligent bus architecture significantly reduces wiring complexity while providing considerably greater diagnostic capability.

---

## 30.7 Supported Servo Families

The current SDK supports the following HiWonder servo families.

| Family | Typical Application |
|----------|--------------------|
| LX-15D | General purpose |
| LX-225 | High torque |
| HTS-16L | Improved intelligent servo |
| HTS-25L | High torque intelligent servo |
| HX-06L | Compact wrist servo |

All of these appear to implement a common communication protocol.

---

## 30.8 Engineering Observations

Analysis of the SDK indicates that Board A communicates with all supported servo families through a unified command interface.

Servo-specific behaviour is therefore implemented internally by the servo rather than within the STM32 firmware.

This significantly simplifies controller software and allows new compatible servo models to be introduced without major firmware changes.

# 31. Bus Servo Command Categories

The intelligent servo protocol can be divided into six logical groups.

## Motion Commands

Commands responsible for servo movement.

Examples:

- Move
- Stop

---

## Configuration Commands

Commands that permanently modify servo behaviour.

Examples:

- Set ID
- Set Offset
- Save Offset

---

## Status Commands

Commands that retrieve operating information.

Examples:

- Read Position
- Read Temperature
- Read Voltage
- Read Load Status

---

## Protection Commands

Commands that define safe operating limits.

Examples:

- Angle Limits
- Voltage Limits
- Temperature Limits

---

## Power Commands

Commands controlling the servo power state.

Examples:

- Torque Enable
- Torque Disable

---

## Diagnostic Commands

Commands used during maintenance and troubleshooting.

Examples:

- Read ID
- Read Offset
- Read Limits
- Read Firmware Parameters

# 36. Power Management

## 36.1 Servo Torque

The servo motor may be electronically enabled or disabled.

When torque is disabled:

- The motor is de-energised.
- The output shaft rotates freely.
- Position is no longer actively maintained.

---

## Typical Applications

Torque disable is useful during:

- Mechanical assembly.
- Manual positioning.
- Calibration.
- Transportation.
- Maintenance.

---

## Torque Enable

Re-enabling torque restores closed-loop position control.

The servo resumes normal operation and maintains the commanded position.

---

## Engineering Notes

Disabling torque does not remove power from the servo electronics.

The servo remains operational and continues to communicate with the controller.

Only the motor drive stage is disabled.

# 37. Engineering Use of Intelligent Bus Servos

## 37.1 Introduction

Intelligent bus servos provide significantly more functionality than conventional PWM hobby servos. While it is possible to use them simply as position-controlled actuators, doing so ignores many of the capabilities that distinguish them from traditional servo systems.

This chapter discusses recommended engineering practices for integrating intelligent bus servos into robotic systems.

The objective is not to describe the communication protocol, but to explain how the available features should be applied during robot design, calibration and operation.

---

# 37.2 Separate Configuration from Operation

A useful engineering principle is to divide servo usage into three distinct phases.

1. Configuration
2. Calibration
3. Normal Operation

These phases have different objectives and should rarely overlap.

---

## Configuration

Configuration establishes permanent properties of the servo.

Typical activities include:

- Assigning the Servo ID.
- Setting angle limits.
- Setting voltage limits.
- Setting temperature limits.
- Saving EEPROM parameters.

Configuration normally occurs only once during the lifetime of the servo.

---

## Calibration

Calibration adapts the servo to the mechanical assembly.

Typical activities include:

- Setting the zero offset.
- Installing the servo horn.
- Verifying mechanical alignment.
- Confirming joint travel.

Calibration is normally repeated only after maintenance or component replacement.

---

## Operation

Operation consists solely of commanding motion and monitoring servo status.

Typical runtime commands include:

- Move servo.
- Read position.
- Read temperature.
- Read voltage.
- Enable or disable torque.

Normal robot software should avoid modifying configuration parameters during operation.

---

# 37.3 Prefer Mechanical Alignment

Servo offsets are intended to compensate for small manufacturing tolerances rather than poor mechanical assembly.

Recommended procedure:

1. Install the servo in its mechanical centre.
2. Install the output horn as accurately as possible.
3. Use the offset parameter only for fine adjustment.

Large software offsets generally indicate that the mechanism should be reassembled.

Mechanical alignment simplifies:

- Inverse kinematics.
- Maintenance.
- Replacement of damaged servos.

---

# 37.4 Use Protection Limits

Protection parameters should be regarded as safety features rather than motion-planning tools.

Angle limits should prevent:

- Self-collision.
- Cable strain.
- Mechanical over-travel.

Voltage limits should protect against:

- Battery failure.
- Wiring faults.
- Incorrect power supplies.

Temperature limits should protect:

- Servo electronics.
- Gearboxes.
- Motors.

Once configured, these limits should rarely require modification.

---

# 37.5 Use Feedback Rather Than Delays

Many example programs use fixed delays after commanding servo movement.

For example:

Move servo

↓

Wait 1000 ms

↓

Continue

While adequate for demonstrations, this approach is unsuitable for more complex robotic systems.

Where possible, applications should instead verify:

- Position reached.
- Motion completed.
- Servo still responding.

Closed-loop feedback is one of the principal advantages of intelligent servos and should be utilised whenever practical.

---

# 37.6 Monitor Servo Health

The intelligent servo continuously measures several operating parameters.

These should be monitored where appropriate.

Position

Unexpected position errors may indicate:

- Mechanical obstruction.
- Collision.
- Gear damage.

Temperature

Increasing temperature may indicate:

- Excessive loading.
- Continuous stall.
- Mechanical binding.

Supply Voltage

Low voltage may indicate:

- Battery discharge.
- Wiring resistance.
- Connector failure.

Monitoring these parameters enables faults to be detected before complete failure occurs.

---

# 37.7 Torque Control

Disabling torque should be regarded as a maintenance function rather than a normal operating mode.

Typical uses include:

- Manual positioning.
- Mechanical assembly.
- Transport.
- Calibration.

Applications should avoid repeatedly enabling and disabling torque during routine operation unless specifically required by the mechanism.

---

# 37.8 Multi-Servo Motion

Where possible, coordinated joints should be commanded simultaneously.

Advantages include:

- Smoother motion.
- Improved trajectory accuracy.
- Reduced overall movement time.
- More natural robot motion.

Examples include:

- Robotic arms.
- Walking robots.
- Pan-tilt systems.
- Parallel mechanisms.

---

# 37.9 Predictive Maintenance

Unlike conventional servos, intelligent bus servos provide sufficient diagnostic information to support predictive maintenance.

Examples include:

Repeated temperature increases

→ Inspect gearbox.

Repeated voltage drops

→ Inspect power distribution.

Increasing position error

→ Inspect couplers or gears.

Unexpected torque disable

→ Investigate overload conditions.

Periodic monitoring of these parameters can significantly improve long-term reliability.

---

# 37.10 Engineering Summary

The principal advantages of intelligent bus servos are not increased torque or speed, but increased observability and configurability.

A well-designed robot should therefore make use of:

- Position feedback.
- Voltage monitoring.
- Temperature monitoring.
- Protection limits.
- Configuration memory.
- Coordinated multi-servo motion.

These capabilities distinguish intelligent bus servos from conventional PWM hobby servos and enable more reliable, maintainable and fault-tolerant robotic systems.

---

# Part VII - Engineering Notes and Design Considerations

# 38. Design Philosophy

## 38.1 Controller Architecture

Board A should not be viewed as a simple Raspberry Pi expansion board.

Instead, it is a distributed control system consisting of two independent processors with clearly defined responsibilities.

| Processor | Primary Responsibilities |
|------------|--------------------------|
| Raspberry Pi | High-level software, computer vision, AI, networking, motion planning |
| STM32 | Real-time control, peripheral management, communication timing |

This architecture combines the computational capability of Linux with the deterministic behaviour of an embedded microcontroller.

---

## 38.2 Separation of Responsibilities

A useful way to understand Board A is to divide the software into three layers.

Application Layer

- Robot logic
- Vision
- AI
- ROS
- Motion planning

↓

Board SDK

↓

Embedded Controller

- Motors
- Servos
- LEDs
- Sensors

Applications should avoid bypassing these layers unless developing replacement firmware.

---

# 39. Deterministic Control

Many robotics platforms attempt to generate servo timing directly from Linux.

Board A deliberately avoids this.

Instead:

Python

↓

UART

↓

STM32

↓

PWM

This provides:

- Stable servo timing.
- Stable motor PWM.
- Reduced CPU utilisation.
- Reduced software complexity.

---

# 40. Direct Raspberry Pi Interfaces

Not every peripheral is controlled by the STM32.

The SDK examples demonstrate two distinct hardware models.

STM32 Managed

- Bus servos
- PWM servos
- Motors
- RGB LEDs
- Buzzer
- Buttons

Direct Raspberry Pi

- GPIO
- I²C

Applications therefore communicate directly with Linux when using GPIO or I²C devices.

Typical libraries include:

- gpiod
- smbus
- smbus2

This distinction is important when designing custom peripherals.

---

# 41. Platform Architecture

Evidence from the SDK and official documentation indicates that Board A is intended as a common robotics platform rather than an application-specific controller.

Current documented users include:

- ArmPi FPV
- ArmPi Pro
- ArmPi Mini
- TurboPi
- MasterPi
- GoGoPi

The primary differences between these robots are:

- Mechanical construction.
- Sensors.
- Example software.

The controller architecture remains substantially unchanged.

---

# 42. Raspberry Pi Compatibility

Current documentation identifies compatibility with:

- Raspberry Pi 4B
- Raspberry Pi 5

The Python SDK is identical across both platforms.

Engineering observations indicate that the STM32 firmware interface is unchanged.

The principal hardware difference is accommodation of the Raspberry Pi 5 active cooling system.

---

# 43. Software Development

Applications should normally be written against the Board SDK.

Typical development sequence:

Application

↓

Board SDK

↓

STM32

↓

Hardware

Developers wishing to use another programming language need only implement the documented UART protocol.

The STM32 firmware remains unchanged.

---

# 44. Future Development

The protocol architecture lends itself to alternative host implementations.

Potential future software includes:

- Native C library.
- C++ SDK.
- Rust SDK.
- Go SDK.
- Java implementation.
- ROS 2 interface.
- MicroPython implementation.
- Embedded microcontroller host.

The protocol is sufficiently simple that additional language bindings should require only packet encoding and decoding.

---

# 45. Engineering Conclusions

The investigation documented in this manual leads to several important conclusions.

1. Board A is a reusable robotics controller rather than an ArmPi-specific expansion board.

2. The STM32 functions as a dedicated real-time robotics co-processor.

3. The Raspberry Pi and STM32 have clearly separated responsibilities.

4. Intelligent bus servos provide significantly greater capability than conventional PWM servos.

5. GPIO and I²C devices bypass the STM32 and communicate directly with Linux.

6. The published SDK effectively documents the underlying communication protocol.

7. Custom host software can be developed independently of the official Python SDK provided the documented UART protocol is implemented.

# 46. Software Development Guidelines

## 46.1 Introduction

The Board A architecture is intended to separate high-level robotics software from deterministic hardware control.

Applications should therefore be written to utilise the services provided by the STM32 rather than attempting to replace them.

Following the guidelines in this chapter will generally produce software that is simpler, more reliable and easier to maintain.

---

# 46.2 Use the Highest Appropriate Level of Abstraction

The communication stack consists of several layers.

```
Robot Application

↓

Board SDK

↓

UART Protocol

↓

STM32 Firmware

↓

Hardware
```

Most applications should interact only with the Board SDK.

Only projects requiring support for another programming language or operating system should communicate directly with the UART protocol.

Firmware modification should be regarded as a last resort.

---

# 46.3 Keep Real-Time Logic Inside the STM32

The STM32 exists specifically to perform deterministic timing.

Applications should therefore avoid attempting to generate:

- Servo PWM
- Motor PWM
- RGB timing
- Bus servo timing

These functions are already implemented by the controller firmware.

Instead, applications should issue high-level commands describing the desired behaviour.

---

# 46.4 Design Around Feedback

One of the principal advantages of Board A is that many peripherals can report their operating state.

Whenever practical, software should verify hardware state rather than assuming successful execution.

Examples include:

Instead of

Move Servo

↓

Wait 1000 ms

↓

Continue

Prefer

Move Servo

↓

Read Position

↓

Continue when Position Reached

Likewise, battery voltage, servo temperature and other available status information should be incorporated into robot decision making where appropriate.

---

# 46.5 Separate Configuration from Runtime

Applications should distinguish between:

Configuration

- Servo IDs
- Protection limits
- Calibration
- EEPROM settings

Runtime

- Motion
- Status monitoring
- User interaction

Configuration should normally be performed during commissioning or maintenance rather than during routine operation.

---

# 46.6 Prefer Coordinated Commands

Where supported by the protocol, multiple devices should be updated together.

Examples include:

- Multiple servo movements
- Simultaneous motor updates
- RGB LED updates

This produces smoother and more predictable robot behaviour.

---

# 46.7 Handle Communication Failures

Although UART communication is generally reliable, applications should assume that communication failures are possible.

Recommended responses include:

- Retry transient failures.
- Detect missing responses.
- Validate returned data.
- Notify the operator where appropriate.
- Place the robot into a safe state if communication cannot be restored.

---

# 46.8 Avoid Hard-Coded Timing

Robot software should avoid unnecessary delays.

Fixed timing assumptions often become invalid after:

- Battery discharge.
- Mechanical wear.
- Payload changes.
- Firmware updates.

Whenever possible, software should use measured state rather than elapsed time.

---

# 46.9 Design for Maintainability

Applications should be structured so that hardware-specific code is isolated from robot behaviour.

For example:

```
Robot Behaviour

↓

Manipulator Controller

↓

Board Interface

↓

Board SDK
```

This allows the hardware interface to be replaced without modifying higher-level application logic.

---

# 46.10 Future Compatibility

The protocol architecture suggests that future Board A firmware may introduce additional Function IDs or extend existing payloads.

Applications should therefore:

- Ignore unknown report packets.
- Validate packet lengths.
- Avoid assumptions about unused fields.
- Prefer symbolic constants over literal values.

Following these practices improves compatibility with future firmware revisions.

---

# 46.11 Engineering Summary

The most successful Board A applications typically follow four guiding principles.

1. Use the highest practical level of abstraction.
2. Delegate deterministic timing to the STM32.
3. Use feedback wherever possible.
4. Separate configuration, calibration and operation.

Applications developed according to these principles are generally simpler, more robust and easier to maintain than those which attempt to bypass the controller architecture.

# 47. Troubleshooting and Diagnostics

## 47.1 Introduction

This chapter describes common faults that may be encountered when using the Board A Controller and provides a systematic approach to diagnosing hardware and software problems.

Where possible, troubleshooting should proceed from the highest system level towards the individual hardware components.

---

# 47.2 System Startup

Before investigating individual peripherals, verify that the controller has started correctly.

Recommended checks include:

- Raspberry Pi boots successfully.
- Linux operating system loads normally.
- Board SDK starts without errors.
- UART communication can be established.
- STM32 controller responds.

Failure at this stage indicates a system-level problem that should be resolved before testing individual peripherals.

---

# 47.3 Communication Problems

## Symptoms

- SDK reports timeout.
- Commands have no effect.
- No response packets received.
- Intermittent operation.

## Possible Causes

- Incorrect UART device.
- Incorrect baud rate.
- Loose connector.
- STM32 firmware not running.
- Damaged cable.

## Recommended Checks

- Verify UART device.
- Confirm 1 Mbit/s configuration.
- Inspect wiring.
- Restart both processors.
- Test with a known working SDK example.

---

# 47.4 Bus Servo Problems

## Symptoms

- Servo does not move.
- Servo moves unexpectedly.
- Incorrect servo responds.
- Servo overheats.
- Position feedback incorrect.

## Possible Causes

- Duplicate Servo IDs.
- Incorrect wiring.
- Mechanical obstruction.
- Excessive load.
- Incorrect calibration.

## Recommended Checks

- Read Servo ID.
- Read position.
- Read voltage.
- Read temperature.
- Verify offset calibration.

---

# 47.5 PWM Servo Problems

## Symptoms

- No movement.
- Incorrect movement.
- Servo jitter.

## Possible Causes

- Incorrect channel.
- Insufficient power.
- Damaged servo.
- Excessive mechanical load.

---

# 47.6 Motor Problems

## Symptoms

- Motor does not rotate.
- Incorrect direction.
- Reduced speed.
- Uneven operation.

## Possible Causes

- Wiring fault.
- Motor driver configuration.
- Mechanical binding.
- Low battery voltage.

---

# 47.7 RGB LED Problems

## Symptoms

- No illumination.
- Incorrect colour.
- Flickering.

## Possible Causes

- Incorrect LED index.
- Invalid colour values.
- Loose connection.

---

# 47.8 Buzzer Problems

## Symptoms

- No sound.
- Continuous sound.
- Incorrect tone.

## Possible Causes

- Incorrect frequency.
- Invalid timing parameters.
- Hardware fault.

---

# 47.9 Battery Related Problems

Low battery voltage can affect multiple subsystems simultaneously.

Possible symptoms include:

- Servo instability.
- Motor power reduction.
- Unexpected resets.
- Communication failures.

Battery voltage should therefore be verified whenever multiple unrelated faults appear simultaneously.

---

# 47.10 Engineering Approach

Faults should be isolated methodically.

Recommended order:

1. Verify power.
2. Verify Raspberry Pi.
3. Verify STM32 communication.
4. Verify SDK operation.
5. Verify peripheral.
6. Verify mechanical assembly.

Replacing components without identifying the root cause is discouraged.

---

# 47.11 Diagnostic Summary

Many reported software problems are ultimately caused by:

- Power distribution.
- Wiring.
- Mechanical faults.
- Configuration errors.

A structured diagnostic procedure significantly reduces troubleshooting time.

# 48. Performance Considerations

## 48.1 Introduction

The performance of the Board A Controller is determined by the interaction between the Raspberry Pi, the STM32 microcontroller and the connected peripherals.

For most robotics applications the limiting factor is not processor performance but communication latency, peripheral response time and mechanical movement.

Understanding these characteristics enables developers to design responsive and reliable applications.

---

# 48.2 Distributed Processing

Board A distributes computation between two processors.

| Processor | Typical Responsibilities |
|------------|--------------------------|
| Raspberry Pi | Vision, AI, user applications, networking |
| STM32 | Deterministic I/O, servo control, motor control, timing |

This separation allows each processor to operate within its area of strength.

---

# 48.3 UART Communication

Communication between the Raspberry Pi and STM32 occurs over a dedicated UART operating at approximately 1 Mbit/s.

This bandwidth is sufficient for typical robotics control tasks including:

- Servo commands
- Motor updates
- LED control
- Status requests
- Sensor polling

Applications should avoid transmitting unnecessary or repetitive commands.

---

# 48.4 Bus Servo Performance

Bus servos contain their own embedded controllers.

Movement commands therefore require only:

- Servo ID
- Target position
- Movement time

The servo performs trajectory generation and closed-loop position control internally.

This greatly reduces the processing requirements of both the Raspberry Pi and STM32.

---

# 48.5 Polling Frequency

Status information may be requested periodically.

Examples include:

- Position
- Voltage
- Temperature

Polling frequency should be selected according to application requirements.

Very high polling rates generally provide little additional benefit while increasing communication traffic.

---

# 48.6 Motion Latency

A complete movement consists of several stages.

```
Application

↓

SDK

↓

UART

↓

STM32

↓

Servo

↓

Mechanical Motion
```

For most applications, the dominant contributor to response time is the physical movement of the mechanism rather than communication overhead.

---

# 48.7 Linux Scheduling

Applications execute under the Linux operating system.

Consequently:

- Thread scheduling is non-deterministic.
- CPU load may vary.
- Background processes may introduce latency.

Board A mitigates these effects by delegating timing-critical tasks to the STM32.

---

# 48.8 Efficient Communication

Applications should minimise unnecessary communication.

Recommended practices include:

- Combine multiple servo movements into a single command.
- Read only required status information.
- Avoid repeated configuration writes.
- Cache static information where practical.

These techniques reduce communication overhead and improve responsiveness.

---

# 48.9 Scalability

The architecture is well suited to robots containing multiple intelligent peripherals.

Typical examples include:

- Multi-axis robotic arms
- Mobile robots
- Quadrupeds
- Pan-tilt camera systems

As the number of peripherals increases, application design becomes increasingly important in maintaining efficient communication.

---

# 48.10 Engineering Summary

Board A achieves good overall performance through distribution of responsibilities.

The Raspberry Pi performs computationally intensive tasks while the STM32 manages deterministic hardware control.

Applications designed around this architecture generally achieve higher reliability and more predictable behaviour than those attempting to perform low-level timing directly under Linux.

# 49. Architectural Lessons and Design Principles

## 49.1 Introduction

The investigation presented throughout this manual has provided insight not only into the implementation of the Board A Controller, but also into a number of general robotics engineering principles.

Although developed for HiWonder educational robots, many of the architectural decisions are applicable to robotics systems in general.

---

# 49.2 Divide Responsibilities

One of the strongest aspects of the Board A architecture is the clear separation of responsibilities.

The Raspberry Pi performs:

- Vision
- Artificial intelligence
- Motion planning
- Networking
- User applications

The STM32 performs:

- Servo timing
- Motor control
- Peripheral management
- Real-time communication

This separation reduces software complexity and allows each processor to perform the tasks for which it is best suited.

---

# 49.3 Abstract the Hardware

Applications interact with the hardware through a well-defined software interface.

```
Robot Application

↓

Board SDK

↓

Protocol

↓

STM32

↓

Hardware
```

This abstraction provides several advantages.

- Hardware can evolve without affecting application software.
- Multiple programming languages can implement the same protocol.
- Testing becomes simpler.
- Applications remain easier to maintain.

---

# 49.4 Prefer Intelligent Peripherals

Board A demonstrates the advantages of using intelligent peripherals whenever practical.

Examples include:

- Intelligent bus servos.
- Embedded motor controllers.
- Smart sensors.

Distributing intelligence throughout the system reduces the complexity of the central controller and improves scalability.

---

# 49.5 Design for Determinism

General-purpose operating systems provide excellent flexibility but cannot guarantee deterministic timing.

Timing-critical tasks should therefore be delegated to dedicated embedded hardware.

Typical examples include:

- PWM generation.
- Motor control.
- Serial bus timing.
- Safety monitoring.

This principle is widely adopted in industrial automation and robotics.

---

# 49.6 Design for Diagnostics

An important characteristic of Board A is that many peripherals provide operating feedback.

Rather than simply issuing commands, the controller can observe:

- Position.
- Voltage.
- Temperature.
- Status.

This transforms maintenance from reactive fault finding into proactive condition monitoring.

---

# 49.7 Separate Permanent and Temporary Data

The investigation distinguishes three classes of information.

Permanent

- Servo IDs.
- Protection limits.
- Calibration.

Temporary

- Position commands.
- LED colours.
- Motor speed.

Measured

- Position feedback.
- Temperature.
- Voltage.

Treating these categories differently simplifies software design and reduces accidental configuration changes.

---

# 49.8 Design for Expansion

The communication protocol is intentionally modular.

Each major subsystem is assigned its own Function ID.

New functionality can therefore be added without redesigning the entire protocol.

This approach supports long-term maintainability.

---

# 49.9 Reusability

Perhaps the most significant architectural observation is that Board A is not tied to any single robot.

The same controller architecture has been successfully deployed across multiple platforms with widely differing mechanical designs.

Only the application software changes.

This demonstrates the value of separating hardware control from robot behaviour.

---

# 49.10 Engineering Conclusions

The Board A Controller illustrates several principles that are broadly applicable to robotics engineering.

These include:

- Separation of responsibilities.
- Distributed processing.
- Deterministic hardware control.
- Intelligent peripherals.
- Layered software architecture.
- Standardised communication protocols.
- Feedback-based operation.

Collectively, these principles produce systems that are easier to understand, easier to maintain and more readily adapted to new robotic platforms.

The architectural concepts described in this manual therefore extend beyond the specific implementation of Board A and provide a useful foundation for the design of future robotic control systems.

# 50. Future Work

## 50.1 Scope of this Manual

This manual documents the Board A Controller based on publicly available documentation, software analysis and practical investigation.

Where behaviour has been experimentally verified, this has been identified within the relevant chapters.

Future firmware revisions may extend the capabilities described here.

---

## 50.2 Areas for Further Investigation

Topics that would benefit from additional investigation include:

- Complete CRC implementation details.
- STM32 firmware internals.
- Bootloader behaviour.
- Firmware update mechanism.
- Error reporting protocol.
- Performance benchmarking.
- Protocol extensions.

---

## 50.3 Potential Enhancements

Future editions of this manual may include:

- Complete packet trace examples.
- Timing diagrams.
- Logic analyser captures.
- Firmware reverse engineering.
- C and Rust SDK examples.
- ROS 2 integration.
- Microcontroller host implementations.

---

## 50.4 Contribution

As the Board A platform continues to evolve, corrections and additional observations should be incorporated into future revisions of this manual.

Maintaining a clear distinction between documented behaviour, verified behaviour and engineering observations will help preserve the technical accuracy of the document.

---

## 50.5 Closing Remarks

The Board A Controller demonstrates that a relatively simple hardware platform can provide a capable and extensible foundation for educational and hobby robotics.

By combining a high-level computing platform with a dedicated real-time controller and intelligent peripherals, the system achieves a balance between flexibility, performance and ease of use.

It is hoped that this manual not only serves as a technical reference, but also provides insight into the engineering principles that underpin modern robotic control systems.


