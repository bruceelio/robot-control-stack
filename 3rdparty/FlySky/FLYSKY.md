# Arduino Mega + FlySky iBus Setup Guide

This guide documents the setup of the Arduino Mega to receive control input from a FlySky receiver using iBus.

This includes:
- Software setup
- Wiring
- Serial configuration
- Initial testing

---

## Overview

The Arduino reads iBus data from the FlySky receiver and later sends commands to the RoboClaw.

System flow:

    FlySky TX -> Receiver -> Arduino (iBus)

---

## Required Hardware

- Arduino Mega 2560
- FlySky receiver (iA6B or similar)
- USB cable (Arduino -> PC)
- 3-wire cable (signal, 5V, GND)

---

## Step 1 — Install Arduino IDE

1. Download Arduino IDE
2. Install and open it
3. Connect Arduino Mega via USB

---

## Step 2 — Configure Arduino IDE

In the Arduino IDE:

Set board:

    Tools -> Board -> Arduino Mega or Mega 2560

Set processor:

    Tools -> Processor -> ATmega2560

Set port:

    Tools -> Port -> (select the COM port for your Arduino)

---

## Step 3 — Verify Upload Works

Upload a simple test sketch (empty or blink) to confirm:

    void setup() {}
    void loop() {}

If upload fails:
- Check USB cable
- Check correct COM port
- Ensure nothing is connected to pins 0 or 1

---

## Step 4 — FlySky Receiver Wiring

Use the SERVO port on the receiver (top right).

Wire as follows:

    Receiver SERVO (right)   -> Arduino pin 17 (RX2)
    Receiver SERVO (middle)  -> Arduino 5V
    Receiver SERVO (left)    -> Arduino GND

Important:
- Use SERVO port (not SENS)
- Maintain correct orientation (GND / 5V / Signal)

---

## Step 5 — Serial Configuration (iBus)

The FlySky iBus uses:

    Baud rate: 115200
    Format: 8N2

In Arduino code this is:

    Serial2.begin(115200, SERIAL_8N2);

Serial2 corresponds to:

    RX2 = pin 17
    TX2 = pin 16 (not used here)

---

## Step 6 — Upload iBus Test Code

Upload the file:

    ibus_test.ino

This is used to verify that:
- Receiver is working
- Wiring is correct
- Arduino is receiving channel data

---

## Step 7 — Open Serial Monitor

In Arduino IDE:

1. Open Serial Monitor
2. Set baud rate:

    115200

You should see output like:

    CH1: 1500  CH2: 1500  CH3: 1500 ...

---

## Step 8 — Verify Receiver Operation

Before testing:

- Receiver LED must be solid (not flashing)
- Transmitter must be ON and bound

If LED is flashing:
- Receiver is not bound

---

## Step 9 — Verify Channel Movement

Move one stick at a time.

Typical Mode 2 mapping:

    CH1 -> Right stick left/right
    CH2 -> Right stick up/down
    CH3 -> Left stick up/down
    CH4 -> Left stick left/right

Values should range:

    ~1000 to ~2000

Center position:

    ~1500

---

## Step 10 — Troubleshooting

If you see:

    NO SIGNAL

Check:

- Receiver is bound (LED solid)
- Using SERVO port (not SENS)
- Signal wire is on pin 17
- GND is connected
- Serial mode is SERIAL_8N2

---

## Step 11 — Transition to Drive Code

Once iBus test is working:

Upload:

    2wd_Mega2560.ino

This code:
- Reads iBus input
- Converts it to throttle and steering
- Sends commands to RoboClaw

---

## Step 12 — Important Notes

- Do not connect anything to pins 0 or 1 during upload
- Keep signal and ground wires short
- Twist signal and ground wires if possible
- Ensure solid connections (Dupont can be loose)

---

## Summary

- iBus runs on Serial2 (pin 17)
- Requires SERIAL_8N2
- Uses SERVO port on receiver
- ibus_test.ino verifies communication
- 2wd_Mega2560.ino implements driving

---

## Next Step

After this setup:

Proceed to RoboClaw configuration and integration.