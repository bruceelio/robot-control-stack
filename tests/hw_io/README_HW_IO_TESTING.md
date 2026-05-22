tests/hw_io/README_HW_IO_TESTING.md

# Hardware IO Checkout

This directory contains the **pre-competition IO checkout system**.

The goal is to verify that all robot IO behaves correctly at the **software level**, using the same interface as competition code.

---

# Test Procedure

The checkout supports two modes.

## 1. Normal test system (unchanged)

```python
@register_test(...)
def test_precomp_io_checkout(...):
````

Used by:

```bash
python -m tests
```

---

## 2. Direct command mode

```bash
python tests/hw_io/test_io_checkout.py
```

or

```bash
python3 -m tests.hw_io.test_io_checkout
```

This bypasses the test framework and runs the checkout directly.

---

# Philosophy

This system tests:

> Does the IO behave correctly when used in robot code?

It does **not** test:

* individual wires
* TRIG/ECHO pairs
* encoder A/B pins
* PWM signals
* low-level electrical details

All tests operate at the **Pi abstraction layer**.

Examples:

```python
io.motor["shooter"].power
io.ultrasonic["front"]
io.encoder["drive_front_left"].count
```

---

# IO Naming Convention

All IO follows this format:

```python
io.<category>["<device_name>"]
io.<category>["<device_name>"].<property>
io.<category>.<property>
io.<category>["<device_name>"].<method>()
```

---

# Examples

## Inputs

```python
io.bumper["front_left"]
io.reflectance["centre"]
io.ultrasonic["front"]
```

---

## Measurements

```python
io.encoder["drive_front_left"].count
io.encoder["drive_front_left"].velocity

io.current["gripper_right"].amps
io.voltage["battery"].volts
```

---

## Actuators

```python
io.motor["shooter"].power
io.servo["lift"].position
```

---

## Multi-value devices

```python
io.imu.heading
io.imu.pitch
io.imu.roll
io.imu.orientation

io.otos.pose
io.otos.x
io.otos.y
io.otos.heading
```

---

## Actions

```python
io.camera["front"].see()
```

In synchronous mode this calls the camera backend directly.

In async mode this reads the latest camera observations through:

```python
AsyncCameraProxy
```

The calling interface remains unchanged.

---

# Rules

* Single-value sensors → implicit (no property)
* Multi-value sensors → explicit properties
* Actuators → use properties (`power`, `position`)
* Actions → use methods (`()`)

---

# IO Map (CSV)

Each robot defines its IO map in:

```text
tests/hw_io/maps/*.csv
```

---

# CSV Format

```text
pi_io_path,exists,enabled,notes
```

---

# Columns

| Column       | Meaning                     |
| ------------ | --------------------------- |
| `pi_io_path` | Exact IO call string        |
| `exists`     | Hardware physically present |
| `enabled`    | Include in test run         |
| `notes`      | Optional operator guidance  |

---

# Example

```text
io.motor["drive_front_left"].power,True,True,
io.ultrasonic["rear"],False,False,not installed
io.camera["front"].see(),True,True,AprilTag observation test
```

---

# Behaviour

| State                        | Result  |
| ---------------------------- | ------- |
| `exists=False`               | ignored |
| `exists=True, enabled=False` | skipped |
| `exists=True, enabled=True`  | tested  |

---

# Test Execution

The checkout runs:

* top to bottom in CSV order
* each IO according to its rule type in `io_rules.py`

---

# Test Patterns

## Digital Inputs

Examples:

* bumpers
* limits

Behaviour:

* live state monitoring
* detect change (`False ↔ True`)
* pass when both states observed

---

## Analog Inputs

Examples:

* reflectance
* ultrasonic
* current

Behaviour:

* track live value
* record min/max
* pass if value changes meaningfully

---

## Electrical

Examples:

* voltage

Behaviour:

* verify reading exists
* verify reading is within expected range

---

## Motors

Behaviour:

* short forward pulse
* short reverse pulse
* operator confirms movement

---

## Servos

Behaviour:

* operator defines safe range
* default range:

  * `-0.5`
  * `0.0`
  * `+0.5`
* move through positions
* operator confirms behaviour

---

## Encoders

Behaviour:

* manual movement only
* no motor commands
* verify count/velocity changes

---

## IMU

Behaviour:

* rotate or tilt robot
* verify heading/pitch/roll changes

---

## OTOS

Behaviour:

* move robot
* verify pose updates

---

## Camera

Behaviour:

* point camera at visible AprilTags
* call:

```python
io.camera["front"].see()
```

* display:

  * marker ID
  * distance
  * bearing
  * vertical angle

In async mode the test waits for the vision worker to produce observations before failing.

---

# Operator Controls

During monitoring:

```text
[f] fail and continue
[s] skip
[q] quit checkout
```

After tests:

```text
[c] continue
[r] retest
```

---

# Logging

Results are saved to:

```text
tests/hw_io/maps/<robot>_checkout_<timestamp>.txt
```

---

# Log Format

```text
PASS  io.bumper["front_left"]
FAIL  io.motor["collector"].power
SKIP  io.ultrasonic["rear"]
```

Final summary:

```text
PASS: X
FAIL: Y
SKIP: Z
```

---

# Key Design Principles

* test functions, not wires
* match competition code usage
* keep robot-agnostic
* keep operator interaction simple
* prioritize safety

  * low power
  * short motion
  * manual confirmation

---

# Adding New IO

1. Add entry to CSV
2. Ensure naming convention is followed
3. Ensure category exists in `io_rules.py`

No changes to the test runner should be required.

---

# Summary

This system provides:

* fast pre-competition checkout
* robot-agnostic configuration
* consistent IO interface validation
* single-operator workflow

```
```
