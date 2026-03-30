diagnostics/README.md

# Diagnostics

The `diagnostics/` package contains **active measurement and inspection routines** used to understand robot behavior, timing, geometry, and sensor characteristics.

Diagnostics are **not tests** and **not competition behaviors**.

They exist to answer questions like:

* *How long does a 1000 mm drive really take?*
* *What camera pitch corresponds to a floor marker vs a platform marker?*
* *Is rotation speed symmetric clockwise vs counter-clockwise?*
* *What does the robot “see” immediately after InitEscape?*

---

## How Diagnostics Run

Diagnostics are executed **explicitly** via configuration:

```python
# config/__init__.py
RUN_MODE = RunMode.DIAGNOSTICS
```

When enabled:

* Diagnostics run **instead of** the normal robot control loop
* No behaviors or state machines are entered
* The program exits cleanly when diagnostics complete

Execution flow:

```
Robot.py
  └── Controller.run()
        └── diagnostics.runner.run_diagnostics()
```

---

## What Diagnostics Are Allowed To Do

Diagnostics **may execute robot actions**.

They are intentionally more powerful than tests.

Allowed:

* Drive and rotate the robot
* Run motion primitives
* Use motion backends
* Read sensors and cameras
* Perform InitEscape or similar setup motions
* Collect and print measurements
* Log raw data for calibration or tuning

Not allowed:

* Control competition behavior
* Decide match strategy
* Transition robot states
* Replace behaviors
* Persist runtime state

> Diagnostics may **execute**, but they do not **decide**.

---

## Diagnostics vs Tests

| Aspect                  | Tests              | Diagnostics              |
| ----------------------- | ------------------ | ------------------------ |
| Purpose                 | Verify correctness | Measure reality          |
| Deterministic           | Yes                | Often no                 |
| Robot motion            | Minimal or mocked  | Explicit and intentional |
| Output                  | Pass / Fail        | Data, logs, measurements |
| Runs during competition | Never              | Never                    |

Rule of thumb:

> **If you’re checking correctness, it’s a test.
> If you’re learning something, it’s diagnostics.**

---

## Typical Diagnostics Modules

Examples of diagnostics you might place here:

* `camera_angles.py`

  * Print pitch/yaw/roll of visible markers
  * Distinguish floor vs platform markers
  * Infer camera mounting geometry (simulation only)

* `drive_timing.py`

  * Drive fixed distances
  * Measure duration vs commanded distance
  * Produce calibration data

* `rotation_timing.py`

  * Rotate fixed angles
  * Measure overshoot and timing

* `sensor_visibility.py`

  * Observe sensor dropout during motion
  * Determine blind spots

---

## Relationship to Calibration

Diagnostics **produce data**.
Calibration **consumes data**.

Typical flow:

```
Diagnostics → measurement output → calibration tables → runtime configuration
```

For example:

* Diagnostics measure camera pitch vs marker height
* Calibration uses that data to define thresholds
* Runtime code uses calibrated values only

Diagnostics should **never hardcode calibration values**.

---

## Relationship to Behaviors and Navigation

Diagnostics:

* Do **not** use the behavior state machine
* Do **not** invoke navigation goals
* May directly invoke primitives or motion backends

Behaviors and navigation remain untouched and clean.

---

## Safety Notes

* Diagnostics can move the robot
* Always assume motors may run
* Diagnostics should:

  * Clearly print what they are about to do
  * Stop motors on exit
  * Avoid infinite loops

---

## Summary

Diagnostics are:

* Explicitly enabled
* Actively executed
* Measurement-focused
* Non-competitive
* Architecturally isolated

They exist so that **calibration, tuning, and understanding** do not contaminate competition logic.

> Diagnostics tell you *what is happening*
> Calibration tells you *what numbers to use*
> Runtime code simply *uses the numbers*

-