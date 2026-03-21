# RoboClaw Configuration Guide (Arduino Mega + UART)

Great — now you’re at the RoboClaw configuration step, which is critical.
This guide walks you through it cleanly to avoid hard-to-debug issues later.

---

## Goal

Configure the RoboClaw 2x30A motor controller to accept UART (packet serial) commands from the Arduino Mega.

---

## Step 1 — Wiring (before configuration)

Ensure the following connections:

- Mega pin 18 (TX1) -> RoboClaw S1 (RX)
- Mega pin 19 (RX1) <- RoboClaw S2 (TX) (optional but recommended)
- Mega GND -> RoboClaw GND (LB-)

You can configure with just TX -> S1, but RX helps debugging.

---

## Step 2 — Connect RoboClaw to PC

You will need:

- USB cable (RoboClaw -> PC)
- Ion Motion Studio installed

---

## Step 3 — Open Ion Motion Studio

1. Plug RoboClaw into USB
2. Open Ion Motion Studio
3. Click:

    Connect -> select COM port -> Connect

---

## Step 4 — Set Control Mode

Navigate to:

    General Settings -> Control Mode

Set:

    Packet Serial

Do NOT use:
- RC
- Analog
- Simple Serial

---

## Step 5 — Set Baud Rate

Match your Arduino code:

    ROBOCLAW_SERIAL.begin(38400);

Set in RoboClaw:

    Baud Rate = 38400

---

## Step 6 — Set Address

Your Arduino code uses:

    #define ROBOCLAW_ADDR 0x80

Set in RoboClaw:

    Address = 0x80

---

## Step 7 — Serial Configuration (Updated UI)

In newer versions of Ion Motion Studio, there is no separate "Serial Port Settings" menu.

Setting:

    Control Mode = Packet Serial

automatically configures the serial interface (S1) for UART communication.

No additional configuration is required.

---

## Step 8 — Apply Settings

Click:

    Write Settings

If you skip this step, nothing is saved.

---

## Step 9 — Power Cycle

Turn RoboClaw OFF -> ON

---

## Step 10 — First Test

Before connecting motors:

1. Upload Arduino code
2. Turn on transmitter
3. Move stick slowly

---

## Expected Behavior

- Forward: both motors forward
- Backward: both motors reverse
- Left/right: tank turn

---

## Common Mistakes

Wrong mode:
    RC mode -> Arduino commands ignored

Wrong baud rate:
    Arduino 38400 != RoboClaw 115200 -> no movement

No common ground:
    Arduino GND MUST connect to RoboClaw GND

TX/RX swapped:
    Arduino TX -> RoboClaw RX (S1)

---

## Quick Sanity Test (No iBus)

Use this minimal test to verify RoboClaw:

    void setup() {
      Serial1.begin(38400);
    }

    void loop() {
      Serial1.write(0x80);
      Serial1.write(0x00);
      Serial1.write(64);
      Serial1.write((0x80 + 0x00 + 64) >> 8);
      Serial1.write((0x80 + 0x00 + 64) & 0xFF);

      delay(1000);
    }

Motor should spin forward slowly.

---

## Drive Configuration Choice

RoboClaw A = LEFT + RIGHT

Advantages:
- Simpler code
- Natural differential drive
- Fewer serial commands

Disadvantages:
- No per-wheel tuning
- Less flexibility for mecanum

For 2WD, this is the correct approach.

---

## Final Checklist

- Control Mode = Packet Serial
- Baud Rate = 38400
- Address = 0x80
- S1 = Packet Serial
- Mega TX1 (pin 18) -> S1
- GND connected
- Settings written
- Power cycled

---

## Next Steps

Once this works, you can:

- Add acceleration limiting
- Add failsafe stop
- Integrate shooter + drive logic