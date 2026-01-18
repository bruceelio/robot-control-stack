
# Robot Test Framework

This directory contains the automated test framework for the robot software stack.

These tests exist to **fail before the robot does** — catching logic errors, regressions,
and unsafe assumptions *before* they reach real hardware.

This is a **lightweight, custom test system**, intentionally not using `pytest` or
other external frameworks, so it can run in the SR simulator and on real robots
with minimal dependencies.

---

## What This Test Framework Is

- A structured way to validate robot logic and safety
- A registry-based system for discovering and running tests
- A mix of:
  - **Pure logic tests** (safe anywhere)
  - **Simulation tests**
  - **Explicit hardware tests** (opt-in)

---

## What This Test Framework Is NOT

- A replacement for behavior logic
- A system that controls the robot autonomously
- A place to experiment with live hardware by accident
- A full CI system (though it can support one)

---

## Directory Structure

```

tests/
├── **init**.py
├── registry.py          # Test registration decorator + registry
├── runner.py            # Test execution logic
│
├── test_hal_io.py       # Level 1 / HAL I/O tests
├── test_motion.py      # Level 2 motion command tests
├── test_safety.py      # Safety logic (bumpers, limits, interlocks)
├── test_localisation.py# Pose estimation & localisation math
└── test_behaviors.py   # (Optional) high-level behavior checks

````

---

## Core Files

### `registry.py`
Defines:
- `register_test()` decorator
- Global `TESTS` registry

All tests must be registered using this decorator.
There is **no automatic test discovery**.

---

### `runner.py`
Responsible for:
- Executing registered tests
- Filtering by name or category
- Enforcing safety rules (`requires_robot`)

The runner is the **only supported entry point** for running tests.

---

## Writing a Test

Example:

```python
from tests.registry import register_test

@register_test(category="safety", requires_robot=False)
def test_virtual_front_bumper():
    ...
````

Each test declares:

* `category` — logical grouping (hal, motion, safety, etc.)
* `requires_robot` — whether physical hardware is required
* `enabled` — whether the test is runnable

---

## Running Tests

Tests are run via the test runner — **not by executing test files directly**.

### Run all tests

```python
from tests.runner import run_tests
run_tests()
```

### Run a single test

```python
run_tests(only="test_virtual_front_bumper")
```

### Run a category

```python
run_tests(category="safety")
```

---

## Simulation vs Real Robot

Each test explicitly declares whether it requires hardware:

* `requires_robot=False`

  * Pure logic / virtual tests
  * Safe in any environment

* `requires_robot=True`

  * May access HAL, Level2, or actuators
  * Must be run deliberately
  * Intended for simulation or supervised real-robot use

The test runner enforces this automatically.

---

## Configuration

Testing behavior (simulation mode, speed scaling, debug flags) is controlled via:

```
config/testing.py
```

This file configures the environment but **does not run tests** and
**does not import the test framework**.

---

## Design Principles

* Tests should fail before the robot does
* Safety over convenience
* Explicit is better than automatic
* No hidden hardware access
* Tests validate logic — behaviors make decisions
* Tests must be idempotent and leave the system in a safe state.

---

## Future Expansion

This framework is designed to scale across projects with the same architecture.

Additional test categories or files can be added without changing the runner,
as long as tests are registered explicitly.

```

---

If you want, next good steps would be:
- sanity-check `registry.py` and `runner.py` against this contract
- add a `test_localisation.py` skeleton
- add a guard to prevent hardware tests from running accidentally on real robots
```
