Great — the next step is to turn the BOM into a **clean wiring harness plan**. This is where many robotics builds either become **very reliable** or **very messy**, so planning it now saves a lot of time later.

I'll design this assuming the **goBILDA mecanum strafer chassis** layout used in the DECODE starter bot.

---

# 1. Overall electrical topology

Your robot should have **three electrical domains**:

```
HIGH CURRENT (motors)
MEDIUM CURRENT (mechanisms)
LOW CURRENT (logic + sensors)
```

Complete flow:

```
Battery
  ↓
Battery Fuse
  ↓
Main Switch
  ↓
Power Distribution Board
  ├── RoboClaw A (Left drive motors)
  ├── RoboClaw B (Right drive motors)
  ├── Dual VNH5019 (Shooter + Collector)
  ├── 5V Regulator → Arduino + Receiver
  └── 6V UBEC → Servo rail
```

This separation prevents motor noise from resetting the controller.

---

# 2. Recommended electronics placement

For the **mecanum strafer chassis**, a balanced layout works best.

### Center plate

Mount:

* Arduino Mega
* Power distribution board
* 5V regulator
* 6V UBEC
* FlySky receiver

This becomes the **control center**.

### Left side plate

Mount:

* RoboClaw A
  (for FL + RL motors)

### Right side plate

Mount:

* RoboClaw B
  (for FR + RR motors)

### Mechanism area (front or top)

Mount:

* Pololu Dual VNH5019
  (close to shooter motor)

Reason: short motor wires reduce noise and losses.

---

# 3. Wire gauge plan

| Circuit          | Wire gauge    |
| ---------------- | ------------- |
| Battery → switch | **14 AWG**    |
| Switch → PDB     | **14 AWG**    |
| PDB → RoboClaw   | **16 AWG**    |
| PDB → VNH5019    | **16 AWG**    |
| Motor wires      | **18 AWG**    |
| Logic power      | **22 AWG**    |
| Signals          | **24–26 AWG** |

Silicone wire is strongly recommended.

---

# 4. Power wiring

### Battery to switch

```
Battery XT30
  ↓
14 AWG
  ↓
Main switch
```

Length:
~10–15 cm

---

### Switch to power distribution board

```
Switch
  ↓
14 AWG
  ↓
Power distribution board input
```

---

### PDB to motor controllers

Three branches:

```
PDB
 ├── 16 AWG → RoboClaw A
 ├── 16 AWG → RoboClaw B
 └── 16 AWG → VNH5019
```

Try to keep these under **20–30 cm**.

---

# 5. Motor wiring

### Drive motors

Each RoboClaw channel connects to one motor.

Example:

```
RoboClaw A
  M1 → Front Left motor
  M2 → Rear Left motor
```

```
RoboClaw B
  M1 → Front Right motor
  M2 → Rear Right motor
```

Wire:

* **18 AWG**
* twisted pair preferred

---

### Mechanism motors

From VNH5019:

```
Motor A → Shooter
Motor B → Collector
```

Wire:

* **18 AWG**
* keep short

---

# 6. Logic power wiring

### 5V regulator output

```
5V regulator
 ├── Arduino Mega 5V
 └── FlySky receiver VCC
```

Ground shared with system ground.

Wire gauge:

* **22 AWG**

---

### Servo power rail

```
6V UBEC
  ↓
Servo distribution board
  ↓
Servos
```

Servo signal wires come from the Mega.

Do **NOT power servos from the Mega 5V pin**.

---

# 7. Signal wiring

### Receiver → Mega

```
Receiver iBUS
   ↓
Mega Serial2 RX (pin 17)
```

Connections:

```
Receiver GND → Mega GND
Receiver VCC → 5V regulator
Receiver iBUS → pin 17
```

---

### Mega → RoboClaw

Example mapping:

```
Mega TX1 (pin 18) → RoboClaw A RX
Mega RX1 (pin 19) ← RoboClaw A TX
```

```
Mega TX3 (pin 14) → RoboClaw B RX
Mega RX3 (pin 15) ← RoboClaw B TX
```

---

### Mega → VNH5019

Example control pins:

```
Shooter
  PWM → D5
  DIR → D22

Collector
  PWM → D6
  DIR → D24
```

Optional:

```
Current sense
A0 ← shooter
A1 ← collector
```

---

# 8. Cable routing strategy

Separate cables into **three bundles**.

### Bundle 1 — power

```
Battery
Switch
PDB
Motor controllers
```

Route along **frame edge**.

---

### Bundle 2 — motors

Motor wires run to each wheel.

Route along:

* inside chassis rails
* zip-tied every 5–8 cm.

---

### Bundle 3 — signals

Keep these away from motor wires.

Includes:

* receiver
* serial
* PWM
* sensor wires

---

# 9. Connectors

Recommended connector strategy:

| Connection             | Connector      |
| ---------------------- | -------------- |
| Battery                | XT30           |
| Motor controller power | XT30           |
| Motors                 | bullet or XT30 |
| Servo                  | standard 3-pin |
| Logic wires            | Dupont         |

---

# 10. Cable management

Use:

* braided sleeve
* zip ties
* adhesive mounts

Never leave wires loose near mecanum wheels.

---

# 11. Electrical grounding rule

Everything must share a **common ground**:

```
Battery -
  ↓
PDB ground
  ↓
RoboClaw
  ↓
VNH5019
  ↓
Mega
  ↓
Receiver
  ↓
Servo rail
```

Without this, signals will not work correctly.

---

# 12. Final system diagram

```
          +------------------+
          |    FlySky RX     |
          +--------+---------+
                   |
                   v
+-------+   +-----------------------+
|Battery|-->|Fuse → Switch → PDB    |
+-------+   +-----------------------+
               |      |      |     |
               |      |      |     +--> 6V UBEC → Servo rail
               |      |      |
               |      |      +--> 5V regulator → Mega + RX
               |      |
               |      +------> VNH5019
               |             |--> Shooter
               |             |--> Collector
               |
               +------> RoboClaw A → FL + RL
               |
               +------> RoboClaw B → FR + RR
```

---

# 13. One small but important tip

Put **motor controllers physically close to the motors they drive**.

Benefits:

* shorter high-current wires
* less electrical noise
* better efficiency.

---

# Next step (very helpful)

Now that the harness plan is done, the next useful thing is a **mechanical electronics layout for the goBILDA chassis**, showing exactly where to mount:

* RoboClaws
* Mega
* PDB
* regulators

This helps with **weight balance and wire routing**, especially with mecanum wheels.
