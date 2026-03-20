Great questions — these get into **good robotics electrical practice**, especially with motor controllers and servos. I’ll break it down clearly.

---

# 1. Why twist the motor wires from RoboClaw to motors?

Yes — the main reason is **PWM switching noise**.

Motor controllers like the RoboClaw 2x30A Motor Controller drive motors using **high-frequency PWM switching** (typically ~15–20 kHz internally). That means the motor leads carry **rapidly changing currents**.

### Without twisting

Two parallel wires form a **large loop antenna**:

```
controller ───────── motor
controller ───────── motor
```

This loop can:

* radiate electromagnetic noise (EMI)
* inject noise into nearby signal wires
* interfere with sensors or RC receivers

### With twisted pair

```
controller ═╦═╦═╦═ motor
            ╚═╩═╩═
```

Twisting makes the magnetic fields cancel each other out.

Benefits:

✔ reduces EMI radiation
✔ reduces noise coupling into signal wiring
✔ improves reliability of sensors and communication lines

This is the **same reason Ethernet cables and differential signal pairs are twisted**.

---

# 2. Why 18 AWG for motor wires?

Your drive motors (goBILDA Yellow Jackets) typically draw:

* **2–5 A typical**
* **~9 A stall**

18 AWG silicone wire is commonly rated:

* **10–16 A chassis wiring**

So it provides:

✔ enough current capacity
✔ flexibility for moving mechanisms
✔ low voltage drop at short distances

If the motor wires are **very short (<30 cm)**, 18 AWG is perfect.

If you were running long wires (>50 cm), then 16 AWG might be better.

---

# 3. Servo power wiring (UBEC → servo board → servos)

Servos draw very **bursty current**.

Typical goBILDA servo:

* idle: ~100 mA
* moving: ~500–900 mA
* stall: **1.5–2.5 A**

If you have **4 servos**, worst case:

```
4 × 2A ≈ 8A peak
```

That matches why you selected an **8 A UBEC**.

---

## Recommended wire sizes

### UBEC → servo distribution board

Use:

**20 AWG** (minimum)
**18 AWG** (ideal)

Reason:

* carries up to ~8 A bursts
* reduces voltage sag to servos

Length is usually short, so **18–20 AWG is perfect**.

---

### Servo board → individual servos

Standard servo cables are usually:

**22–24 AWG**

This is fine because each cable only feeds **one servo**.

Typical servo cable current:

* ~2 A peak
* short duration

So:

| Segment             | Wire size     |
| ------------------- | ------------- |
| UBEC → servo board  | **18–20 AWG** |
| Servo board → servo | **22–24 AWG** |

---

# 4. Grounding rule (important)

Servo ground must connect to **system ground**.

Example:

```
Battery -
   ↓
PDB ground
   ↓
Mega ground
   ↓
UBEC ground
   ↓
Servo ground
```

Otherwise the servo signal line has **no reference voltage**.

---

# 5. Keep servo wires away from motor wires

Even with twisting, high-current motor cables generate noise.

Good practice:

```
LEFT side of robot
   motor wiring

RIGHT side of robot
   signal wiring
```

Or at least keep **2–3 cm separation**.

---

# 6. One extra improvement many teams add

Add **capacitors across motor terminals**.

Example:

```
motor terminal
   |---||---|
```

Typical values:

* **0.1 µF ceramic**
* rated ≥50 V

This reduces:

* EMI
* controller resets
* radio interference

It’s very common in robotics builds.

---

# Quick summary

| Circuit                 | Recommended wire        |
| ----------------------- | ----------------------- |
| Battery → PDB           | 14 AWG                  |
| PDB → motor controllers | 16 AWG                  |
| Controller → motors     | **18 AWG twisted pair** |
| UBEC → servo board      | **18–20 AWG**           |
| Servo cable             | **22–24 AWG**           |
| Signal wiring           | 24–26 AWG               |

---

✅ Your wiring plan is **already very good** — these questions are exactly the kind that lead to a **very reliable robot**.

---

If you'd like, the next thing I can show you is a **very useful improvement many mecanum robots use: a star-ground power layout**, which reduces brownouts and communication glitches when the drive motors stall.
