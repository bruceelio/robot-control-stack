
# IO Layer

This folder defines the **Input / Output boundary layer** of the robot.

It maps **canonical, robot-agnostic signal names** to **physical hardware endpoints**
such as Arduino pins, GPIOs, or board-specific outputs.

This layer contains **no behavior, no timing, and no decision logic**.

---

## Design Goals

- Support **multiple execution environments**:
  - Student Robotics simulator
  - Student Robotics hardware
  - SR-equivalent hardware without `sr.robot3`
  - Fully custom robots
- Allow the **same behavior and primitive code** to run unchanged
- Isolate all hardware wiring assumptions to one place

---

## Architectural Position



Behaviors / Primitives
↓
Level2
↓
IO ← (this folder)
↓
Physical Hardware


Level2 exposes **semantic capabilities** (e.g. `VACUUM_ON`, `LIFTER_UP`).

The IO layer provides **fallback physical mappings** when SR Robot3 APIs
are not available.

---

## Files Overview

### `canonical.py`
Defines **canonical signal names**.

These represent *what the robot can do*, not how it is wired.

Examples:
- `DO_VACUUM`
- `AO_LIFTER_SERVO`
- `PWM_MOT_LEFT`

This file:
- contains no hardware imports
- contains no SR references
- is pure vocabulary

---

### `sr_robot.py`
Maps canonical signals to **Arduino pins on the SR Robot Arduino board**.

IMPORTANT:
- This file ONLY covers Arduino I/O
- It does NOT include:
  - motors
  - vacuum pump
  - servos
- Those are handled directly via SR Robot3 APIs

Used when:
- running on SR hardware
- or SR simulator
- and accessing Arduino-connected sensors

---

### `sr_equivalent.py`
Maps canonical signals to **SR-equivalent physical wiring**
when `sr.robot3` is NOT available.

This supports:
- custom robots built with SR hardware
- identical wiring but custom software stacks

Pin names here represent **physical connections**, not SR APIs.

---

### `generic.py`
Optional mapping for fully custom robots.

This file can define arbitrary canonical-to-pin mappings
without assuming SR hardware.

---

### `mock.py`
Optional mock or stub mappings for:
- unit tests
- simulation without hardware
- CI environments

---

### `unified.py`
Builds the active `canonical_to_pin` map based on the selected hardware profile.

Level2 uses this map when SR Robot3 APIs are unavailable.

---

## What This Folder Must NOT Contain

- Motion logic
- Timing logic
- Behavior logic
- State machines
- Strategy decisions

If logic appears here, it belongs somewhere else.

---

## Key Principle

> Canonical signals describe **intent**.
>
> IO mappings describe **wiring**.
>
> Level2 decides **how to execute** the intent.

This separation is intentional and must be preserved.
