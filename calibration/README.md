# Calibration

This directory contains **robot motion and geometry calibration data**.

Calibration answers the question:

> *“How long / how much power does the robot need to perform a physical action?”*

Calibration is **not logic**, **not behavior**, and **not decision-making**.

---

## Design Rules

- Calibration data is **read-only during runtime**
- Calibration code **never decides**
- Calibration **never touches hardware**
- Calibration **never prints on import**
- Calibration **can be inspected by humans**, but is not used by behaviors directly

If calibration changes robot behavior dynamically, the architecture is broken.

---

## Directory Structure

```text
calibration/
├── __init__.py
├── base.py          ← structure & formulas (never duplicated)
├── simulation.py    ← simulation-only numbers
├── sr1.py           ← real robot calibration values (Robot 1)
├── sr2.py           ← real robot calibration values (Robot 2)
└── inspect.py       ← human-readable inspection / reporting
````

---

## File Responsibilities

### `base.py`

**Canonical calibration logic**

* Defines:

  * Calibration *interfaces*
  * Mathematical models
  * Helper functions (e.g. duration estimation)
* Contains **no robot-specific numbers**
* Must never be duplicated or modified per robot

> If you are copy-pasting calibration math, you are doing it wrong.

---

### `simulation.py`

**Simulation calibration values only**

* Contains **numbers only**
* Imports structure from `base.py`
* Matches simulator behavior, not real hardware
* Safe to change without affecting real robots

---

### `sr1.py`, `sr2.py`

**Per-robot real-world calibration values**

* Numbers only
* One file per physical robot
* Allows:

  * Mechanical differences
  * Wear and tear compensation
  * Hardware swaps

> Real robots drift. Calibration files absorb that drift — not logic.

---

### `inspect.py`

**Human inspection & reporting**

* Produces readable summaries:

  * Drive calibration
  * Rotation calibration
* May print, log, or export data
* Must **never** be imported by runtime code
* Safe to delete or replace

Run with:

```bash
python -m calibration.inspect
```

---

## How Calibration Is Used

```
Calibration → Motion Primitives → Behaviors → Controller
```

* Motion primitives **query calibration**
* Behaviors **do not care** how calibration works
* Controller **does not know calibration exists**

---

## What Calibration Does NOT Do

Calibration **does not**:

* Make decisions
* Plan paths
* Choose behaviors
* Interpret sensors
* Run during competition
* Compensate dynamically for errors

Those belong elsewhere.

---

## Common Mistakes

❌ Printing during import
❌ Mixing logic with numbers
❌ Calibrating inside behaviors
❌ Hardcoding magic numbers outside calibration
❌ Sharing one calibration file across multiple real robots

---

## Mental Model

> **Calibration is a ruler.**
> You use it to measure — not to decide where to go.

---

## Final Rule

If calibration feels *interesting* to read, something is wrong.

Calibration should be boring, stable, and trusted.

`
