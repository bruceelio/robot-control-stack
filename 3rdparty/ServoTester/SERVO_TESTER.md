# Servo Tester & Gripper Control Guide (FTC / goBILDA)

## Overview
This document explains how to use a servo tester to safely tune a gripper mechanism, especially when handling fragile objects like cardboard boxes (120–140 mm width).

---

# Servo Tester Modes

## Manual Mode
- The knob directly controls servo position.
- Turning the knob → moves the servo.
- Stopping the knob → servo holds that position.

### Key Concept
- You are controlling **position**, not force.
- The servo will apply **whatever force is needed** to maintain that position.

---

## Neutral Mode
- Sends a ~1500 µs PWM signal.
- Moves servo to the **center of its range (~50%)**.

### Important Notes
- This is **electrical center**, not mechanical center.
- Useful for:
  - Initial setup
  - Aligning servo horns
- Not useful for gripping or force control.

---

# How Servo Position Relates to Force

## Critical Insight

Servo position ≠ force control
Servo position → force is a result


- The servo keeps pushing until it reaches the commanded position.
- If blocked (e.g., by a box), it increases force.

---

# Gripper Testing Protocol

## Step 1: Add Protection
Before testing:
- Add foam or padding to claw surfaces
- Avoid bare metal contact

---

## Step 2: Prepare Test Objects
Use:
- 120 mm box
- 130 mm spacer
- 140 mm box

---

## Step 3: Slow Closing Test
1. Start fully open
2. Place object in gripper
3. Slowly turn knob toward closed
4. Watch carefully

---

## Step 4: Identify Key Points

For each object, find:

- **A: First Contact**
  - Claws just touch object

- **B: Secure Grip**
  - Object can be lifted safely

- **C: Crushing Point**
  - Visible deformation begins

---

## Step 5: Record Positions
Example:

120 mm box:

contact: 60%
grip: 65%
crush: 72%

140 mm box:

contact: 40%
grip: 48%
crush: 55%

---

## Step 6: Choose Final Position
- Pick a position that works for all sizes
- Typically near the **minimum secure grip**

---

# Important Safety Rules

## Do NOT:
- Turn knob quickly to full close
- Keep tightening after grip is achieved
- Test without padding

## DO:
- Move slowly
- Stop early
- Aim for minimum required grip

---

# Handling Variable Box Sizes (120–140 mm)

## Problem
- Smaller box → higher force at same position
- Larger box → lower force

## Solution
Add **compliance**:

### Recommended:
- Foam pads (3–10 mm thick)
- Rubber or EVA foam
- Spring-loaded claws
- Flexible mounting

---

# Force vs Pressure


Pressure = Force / Area


To reduce crushing:
- Increase contact area (flat plates)
- Add soft material (foam)

---

# Mechanical Recommendations

## Best Claw Design

Beam
→ Flat plate (polycarbonate/aluminum)
→ Foam layer


## Target Grip Force
- ~10–30 N is usually sufficient
- System capable of ~250 N → must limit effectively

---

# Translating to Code

After testing:

Example:

Best grip position ≈ 58%


In FTC code:
```java
servo.setPosition(0.58);