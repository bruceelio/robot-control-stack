diagnostics/README_DIAGNOSTICS.md

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

## Relationship to Analysis

Diagnostics collect raw measurement data.

Analysis tools process that data into useful engineering information such as:

* Graphs
* Curve fitting
* Statistical summaries
* Calibration constants

Typical workflow:

```
Robot Diagnostic
        │
        ▼
CSV results
        │
        ▼
Analysis program
        │
        ├── Graphs
        ├── Regression
        ├── Statistics
        └── Calibration values
```

Diagnostics should remain focused on collecting accurate data.

Analysis programs should perform all graphing, regression, and numerical modelling so that historical datasets can be re-analysed at any time.

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

---

# Running Diagnostics

Open a terminal in the project root directory.

Diagnostics are executed as Python modules.

Examples:

```bash
python3 -m diagnostics.drive_timing
python3 -m diagnostics.rotation_timing
python3 -m diagnostics.drive_encoder
```

Analysis programs are executed in the same way.

Examples:

```bash
python3 -m diagnostics.analysis.analyse_drive_timing
python3 -m diagnostics.analysis.analyse_rotation_timing
python3 -m diagnostics.analysis.analyse_drive_encoder
```

Results are normally written to:

```
diagnostics/results/
```

Generated graphs, reports, and curve-fitting results should be written to:

```
diagnostics/analysis/graphs/
diagnostics/analysis/reports/
```

Running modules using `python3 -m` is the preferred method because it ensures imports are resolved correctly from the project root.