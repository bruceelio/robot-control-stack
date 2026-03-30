# Mecanum Drive Guide (Arduino Mega + FlySky iBus + Dual RoboClaw)

This guide documents the updated mecanum drive configuration using:

- Arduino Mega 2560
- FlySky receiver via iBus
- Two RoboClaw motor controllers via packet serial
- Right-stick translation control

This file assumes the FlySky setup and RoboClaw packet-serial setup are already working.

Related files:
- FLYSKY.md
- ROBOCLAW.md

---

## Overview

The mecanum drive system uses:

    FlySky TX -> Receiver -> Arduino Mega -> RoboClaw A / RoboClaw B -> 4 mecanum wheels

The Arduino:
- reads iBus from the receiver
- converts stick inputs into mecanum wheel commands
- sends packet-serial motor commands to two RoboClaws

---

## Control Mapping

This updated mecanum version uses the transmitter as follows:

    CH2 -> throttle  -> right stick up/down
    CH1 -> strafe    -> right stick left/right
    CH4 -> rotate    -> left stick left/right

This gives:

- Right stick = translation
- Left stick horizontal = rotation

That is the most natural mapping for mecanum drive with your FlySky setup.

---

## Serial Interfaces

The Arduino Mega uses:

    Serial2 -> FlySky iBus
    Serial1 -> RoboClaw A
    Serial3 -> RoboClaw B

### FlySky iBus

iBus uses:

    Serial2.begin(115200, SERIAL_8N2);

Pins:

    RX2 = pin 17
    TX2 = pin 16 (unused)

### RoboClaw packet serial

RoboClaw links use:

    Serial1.begin(38400);
    Serial3.begin(38400);

---

## RoboClaw Mapping

This mecanum code assumes the following wheel-to-controller mapping:

### RoboClaw A
- M1 = Front Left
- M2 = Rear Left

### RoboClaw B
- M1 = Front Right
- M2 = Rear Right

Addresses assumed:

    RoboClaw A = 0x80
    RoboClaw B = 0x81

If RoboClaw B uses a different address, update the code accordingly.

---

## Packet Serial Format

The mecanum code now uses the corrected RoboClaw packet-serial implementation.

Important point:

    RoboClaw requires CRC16 packet format

This replaced the earlier simple checksum approach, which did not work reliably.

The working implementation now:
- computes CRC16
- sends valid packet serial commands
- correctly drives motors from Arduino

---

## Input Modes and Output Modes

The updated mecanum file supports selectable modes.

### Input mode options
- RC PWM input
- FlySky iBus input

### Output mode options
- RC PWM motor outputs
- RoboClaw packet serial outputs

Current preferred configuration:

    USE_INPUT_PWM  = 0
    USE_INPUT_IBUS = 1

    USE_OUTPUT_RC_PWM       = 0
    USE_OUTPUT_ROBOCLAW_PKT = 1

That means:
- input comes from FlySky iBus
- output goes to two RoboClaws via packet serial

---

## Mecanum Drive Logic

The mecanum wheel mix is:

    FL = forward + strafe + rotate
    FR = forward - strafe - rotate
    RL = forward - strafe + rotate
    RR = forward + strafe - rotate

After mixing:
- outputs are normalized
- commands are limited to the valid range
- resulting motor commands are sent to RoboClaws

---

## Speed Scaling

The updated code includes software scaling so the robot is easier to control.

Current values:

    DRIVE_SCALE  = 0.38
    STRAFE_SCALE = 0.38
    TURN_SCALE   = 0.45

Purpose:
- reduce top speed
- soften turning
- make 312 RPM motors feel closer to a slower drivetrain

These can be tuned later.

---

## Signal Loss Safety

The code stops the robot if iBus signal is lost.

Logic:

    if no valid iBus frame is received for more than 200 ms
    -> stop all motors

This protects against:
- transmitter off
- receiver signal loss
- iBus communication failure

---

## Required Wiring Summary

### FlySky Receiver to Arduino Mega

Use the receiver SERVO port:

    SERVO right  -> Arduino pin 17
    SERVO middle -> Arduino 5V
    SERVO left   -> Arduino GND

### Arduino Mega to RoboClaw A

    Mega pin 18 (TX1) -> RoboClaw A S1
    Mega GND          -> RoboClaw A GND

Optional:

    Mega pin 19 (RX1) <- RoboClaw A S2

### Arduino Mega to RoboClaw B

    Mega pin 14 (TX3) -> RoboClaw B S1
    Mega GND          -> RoboClaw B GND

Optional:

    Mega pin 15 (RX3) <- RoboClaw B S2

Important:
- all grounds must be common
- use packet serial mode on both RoboClaws

---

## Required RoboClaw Configuration

For each RoboClaw:

- Control Mode = Packet Serial
- Baud Rate = 38400
- Correct address assigned
- Settings written
- Power cycled after writing

Suggested addresses:

    RoboClaw A = 0x80
    RoboClaw B = 0x81

---

## Expected Driving Behavior

With the updated mapping:

- Right stick up/down = forward/backward
- Right stick left/right = strafe left/right
- Left stick left/right = rotate in place

Examples:

- push right stick up -> robot moves forward
- push right stick right -> robot strafes right
- push left stick left -> robot rotates left
- combine translation + rotation -> diagonal curved motion

---

## First Test Procedure

Before first mecanum test:

1. Confirm FlySky iBus test works
2. Confirm each RoboClaw can move motors correctly
3. Put wheels off the ground
4. Upload the mecanum code
5. Test one axis at a time:
   - throttle
   - strafe
   - rotate
6. Verify each wheel direction

If a wheel spins the wrong way:
- swap that motor's leads
or
- invert that wheel in code

---

## Common Issues

### Robot does not move
Check:
- iBus is working
- receiver is bound
- RoboClaws are in Packet Serial mode
- baud rate is 38400
- addresses match the code
- grounds are connected

### Robot moves strangely
Check:
- wheel mapping to M1/M2 is correct
- one or more motors may be reversed
- FlySky channel mapping may be wrong

### One side works, the other does not
Check:
- RoboClaw B address
- Serial3 wiring
- GND connection to RoboClaw B

### Motors respond in Motion Studio but not from Arduino
Check:
- packet serial CRC16 implementation
- Arduino serial port used
- address and baud match

---

## Current File Roles

At this point the three documentation files are:

### FLYSKY.md
Documents:
- Arduino IDE setup
- iBus receiver wiring
- live iBus testing
- channel verification

### ROBOCLAW.md
Documents:
- RoboClaw packet-serial configuration
- address and baud settings
- packet-serial testing
- grounding and control wiring

### MECANUM_DRIVE.md
Documents:
- updated mecanum control mapping
- dual RoboClaw architecture
- wheel/controller mapping
- packet-serial output assumptions
- speed scaling and safety behavior

---

## Summary

The updated mecanum system now uses:

- right-stick translation
- left-stick rotation
- FlySky iBus on Serial2
- RoboClaw packet serial with CRC16
- two RoboClaws for four-wheel mecanum drive
- speed scaling for more controllable motion
- signal-loss stop logic for safety

This is the current working architecture for the mecanum variation.