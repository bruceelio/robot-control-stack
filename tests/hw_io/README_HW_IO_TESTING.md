tests/hw_io/README_HW_IO_TESTING.md

# Hardware IO Checkout

This directory contains the **pre-competition IO checkout system**.

The goal is to verify that all robot IO behaves correctly at the **software level**, using the same interface as competition code.

---
## Test Procedure 

Now the file works in two modes:

1. Normal test system (unchanged)
```python 
@register_test(...)
def test_precomp_io_checkout(...)
```

Used by:
```python 
python -m tests
```

2. Direct command (new)

```python 
python tests/hw_io/test_io_checkout.py

Or

python3 -m tests.hw_io.test_io_checkout
```

This bypasses the test framework and just runs the checkout.

## Philosophy

This system tests:


Does the IO behave correctly when used in robot code?


It does **not** test:


individual wires (TRIG/ECHO, encoder A/B, PWM pins, etc.)


All tests operate at the **Pi abstraction layer**:

```python
io.motor["shooter"].power
io.ultrasonic["front"]
io.encoder["drive_front_left"].count
IO Naming Convention

All IO must follow this format:

io.<category>["<device_name>"]
io.<category>["<device_name>"].<property>
io.<category>.<property>
io.<category>["<device_name>"].<method>()
Examples
# inputs
io.bumper["front_left"]
io.reflectance["centre"]
io.ultrasonic["front"]

# measurements
io.encoder["drive_front_left"].count
io.encoder["drive_front_left"].velocity
io.current["gripper_right"].amps
io.voltage["battery"].volts

# actuators
io.motor["shooter"].power
io.servo["lift"].position

# multi-value devices
io.imu.heading
io.imu.pitch
io.imu.roll
io.imu.orientation

io.otos.pose
io.otos.x
io.otos.y
io.otos.heading

# actions
io.camera["front"].see()
```

Rules
Single-value sensors → implicit (no property)
Multi-value sensors → explicit properties
Actuators → use properties (power, position)
Actions → use methods ()
IO Map (CSV)

Each robot defines its IO in:

tests/hw_io/maps/*.csv
Format
pi_io_path,exists,enabled,notes
Columns
pi_io_path → exact IO call string
exists → hardware physically present
enabled → include in test run
notes → optional operator guidance
Example
io.motor["drive_front_left"].power,True,True,
io.ultrasonic["rear"],False,False,not installed
io.camera["front"].see(),True,False,tested separately
Behavior
exists=False  → ignored
exists=True, enabled=False → skipped
exists=True, enabled=True → tested
Test Execution

The checkout runs:

Top to bottom in CSV order

Each IO is tested according to its type (see io_rules.py).

Test Patterns
Digital Inputs (bumpers, limits)
Live state monitoring
Detect change (False ↔ True)
Pass when both states observed
Analog Inputs (reflectance, ultrasonic, current)
Track live value
Record min/max
Pass if value changes meaningfully
Electrical (voltage)
Check value exists and is within expected range
Motors
Short forward and reverse pulses
Operator confirms motion
Servos
Operator defines safe range
Default: -0.5 / 0.0 / +0.5
Move through range
Operator confirms behavior
Encoders
Manual movement only (no motor commands)
Check count and velocity change
IMU
Rotate/tilt robot
Check heading/pitch/roll change
OTOS
Move robot
Check pose (x, y, heading) updates
Camera
Not tested here
Uses separate camera testing protocol
Operator Controls

During tests:

[f] fail and continue
[s] skip
[q] quit checkout

After test:

[c] continue
[r] retest
Logging

Results are saved to:

tests/hw_io/maps/<robot>_checkout_<timestamp>.txt
Format
PASS  io.bumper["front_left"]
FAIL  io.motor["collector"].power
SKIP  io.ultrasonic["rear"]

Final summary:

PASS: X
FAIL: Y
SKIP: Z
Key Design Principles
Test functions, not wires
Match competition code usage
Keep robot-agnostic
Keep operator interaction simple
Prioritize safety (low power, short motion)
Adding New IO
Add entry to CSV
Ensure it follows naming convention
Confirm category exists in io_rules.py

No changes to test runner required.

Summary

This system provides:

Fast pre-competition checkout
Robot-agnostic configuration
Consistent IO interface validation
Single-operator workflow