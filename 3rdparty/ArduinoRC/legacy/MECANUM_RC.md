
# Standalone Mecanum Robot Controller
### Arduino Mega + FlySky iBus + Dual RoboClaw + Shooter

---

# Purpose

This sketch provides a **standalone remote-controlled mecanum robot**.

It is intended for manual driving using a FlySky transmitter and **does not communicate with the Raspberry Pi** or support autonomous operation.

The sketch controls:

- Four-wheel mecanum drivetrain
- Shooter motor
- Dual shooter-feed servos

The shooter and shooter-feed implementation is copied directly from the proven 2WD controller and intentionally behaves identically.

---

# System Overview

The control chain is:

```

FlySky Transmitter
│
▼
FlySky Receiver (iBus)
│
▼
Arduino Mega 2560
│
├──────────────► RoboClaw A (Front Motors)
│
├──────────────► RoboClaw B (Rear Motors)
│
├──────────────► Shooter Motor
│
└──────────────► Shooter Feed Servos

```

The Arduino performs all control logic.

No Raspberry Pi is required.

---

# Hardware

## Controller

- Arduino Mega 2560

## Radio

- FlySky receiver using iBus

## Motor Controllers

- RoboClaw A
- RoboClaw B

Packet Serial mode is used for both.

## Drive

- Four independent mecanum wheels

## Shooter

- One PWM/Direction controlled shooter motor

## Feeder

- Two goBILDA servos operating the ball feed mechanism

---

# Serial Interfaces

| Arduino Port | Connected Device |
|--------------|------------------|
| Serial1 | RoboClaw A |
| Serial2 | FlySky iBus |
| Serial3 | RoboClaw B |

---

# RoboClaw Configuration

The robot is wired as follows.

## RoboClaw A (Front)

| Output | Motor |
|---------|-------|
| M1 | Front Left |
| M2 | Front Right |

## RoboClaw B (Rear)

| Output | Motor |
|---------|-------|
| M1 | Rear Left |
| M2 | Rear Right |

Each RoboClaw is connected to its own hardware serial port.

---

# FlySky Control Mapping

## Driving

| Channel | Control | Function |
|----------|----------|----------|
| CH1 | Right Stick Left / Right | Strafe |
| CH2 | Right Stick Up / Down | Forward / Reverse |
| CH4 | Left Stick Left / Right | Rotate |

This provides:

- Right stick = Translation
- Left stick = Rotation

---

## Shooter

| Channel | Function |
|----------|----------|
| CH6 | Shooter Speed |

The shooter implementation is identical to the standard 2WD controller.

---

## Shooter Feed

| Channel | Function |
|----------|----------|
| CH7 | Feed Ball |

The feed mechanism is identical to the standard 2WD controller.

---

# Mecanum Drive Logic

Unlike the standard differential-drive controller, this sketch mixes three driver commands into four wheel speeds.

Inputs:

```

Forward
Strafe
Rotate

```

Wheel outputs:

```

Front Left  = Forward + Strafe + Rotate
Front Right = Forward - Strafe - Rotate

Rear Left   = Forward - Strafe + Rotate
Rear Right  = Forward + Strafe - Rotate

````

The outputs are normalised before being sent to the RoboClaws so that wheel commands always remain within the valid range.

This is the **only significant behavioural difference** from the standard 2WD controller.

---

# Shooter System

The shooter motor implementation is copied directly from the proven 2WD controller.

Features include:

- PWM/Direction control
- Adjustable speed scaling
- Immediate stop on signal loss

No behavioural changes have been made.

---

# Shooter Feed System

The shooter-feed implementation is also copied directly from the standard controller.

Features include:

- Dual goBILDA servos
- Timed feed pulse
- Automatic return
- Existing calibration retained

Servo end points may be tuned mechanically if additional ball travel is required.

---

# Signal Loss Protection

The controller continuously monitors incoming iBus frames.

If communication is lost for more than approximately **200 ms**, the controller immediately:

- Stops all four drive motors
- Stops the shooter motor
- Stops both shooter-feed servos

Control resumes automatically once valid FlySky communication returns.

---

# Improved iBus Parser

The current controller includes an improved iBus parser.

Unlike the original implementation, it:

- Processes all pending serial data each loop
- Prevents serial buffer overflow
- Eliminates periodic drive interruptions caused by delayed frame processing

This produces significantly smoother driving.

---

# Adjustable Parameters

The following values are intended for tuning.

## Drive Scaling

```cpp
DRIVE_SCALE
STRAFE_SCALE
TURN_SCALE
````

Adjusts the responsiveness of the drivetrain.

---

## Shooter

```cpp
TELEOP_SHOOTER_SCALE
```

Adjusts maximum shooter speed.

---

## Shooter Feed

Typical tuning parameters include:

```cpp
SHOOTER_FEED_PULSE_MS

SHOOTER_FEED_STOP_US
SHOOTER_FEED_LEFT_RUN_US
SHOOTER_FEED_RIGHT_RUN_US
```

These determine feed duration and servo travel.

---

# Wiring Summary

## FlySky Receiver

```
Receiver SERVO
        │
        ├── Signal → Mega RX2 (Pin 17)
        ├── +5V
        └── GND
```

---

## RoboClaw A

```
Mega TX1 (18) ─────► RoboClaw A S1
Mega GND ──────────► RoboClaw A GND
```

Optional:

```
Mega RX1 (19) ◄──── RoboClaw A S2
```

---

## RoboClaw B

```
Mega TX3 (14) ─────► RoboClaw B S1
Mega GND ──────────► RoboClaw B GND
```

Optional:

```
Mega RX3 (15) ◄──── RoboClaw B S2
```

---

# Commissioning Checklist

Before driving the robot:

* Verify the FlySky receiver is bound.
* Confirm both RoboClaws are configured for Packet Serial mode.
* Confirm both RoboClaws communicate correctly.
* Raise the robot so all four wheels are clear of the floor.
* Test:

  * Forward
  * Reverse
  * Left strafe
  * Right strafe
  * Clockwise rotation
  * Counter-clockwise rotation
* Verify shooter operation.
* Verify shooter-feed operation.

If any wheel rotates incorrectly, reverse that motor either:

* electrically (swap motor wires), or
* in software.

---

# Troubleshooting

## Robot behaves like a 2WD robot

Usually indicates the rear RoboClaw is not receiving commands.

Check:

* Serial3 wiring
* RoboClaw B configuration
* Packet Serial mode
* Common ground

---

## Robot stops briefly while driving

Usually indicates FlySky communication has been interrupted.

Check:

* Receiver wiring
* iBus connection
* Receiver power
* FlySky signal quality

---

## Shooter operates but feed travel is insufficient

Adjust:

```cpp
SHOOTER_FEED_LEFT_RUN_US
SHOOTER_FEED_RIGHT_RUN_US
```

before increasing:

```cpp
SHOOTER_FEED_PULSE_MS
```

Increasing the servo end points increases feed travel. Increasing the pulse duration only increases the time taken to complete the feed/return cycle.

---

# Summary

This controller is the standalone manual-control version of the robot.

It provides:

* FlySky iBus control
* Four-wheel mecanum drive
* Dual RoboClaw packet serial control
* Shooter motor control
* Dual shooter-feed servo control
* Signal-loss protection
* No Raspberry Pi dependency
* No autonomous operation

The design philosophy is intentionally simple: **retain the proven 2WD controller architecture and replace only the drivetrain mixer with a mecanum mixer.** All shooter behaviour, safety features and hardware interfaces remain unchanged from the tested 2WD implementation.

```
```
