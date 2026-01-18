# HAL Layer (Legacy / Compatibility)

This folder contains the original **Hardware Abstraction Layer (HAL)** which maps
canonical signal names (pins / channels) to physical endpoints.

As of the current architecture, **`hw_io/` is the primary IO boundary**.
HAL remains for:
- legacy code paths which still reference pinmaps
- SR-equivalent / non-SR wiring maps
- gradual migration and cleanup

If you’re writing new robot code, prefer:
- `hw_io/` (IOMap implementations, camera wrappers, outputs)
- `Level2` talking only to `IOMap`

---

## Architectural Position (Current)

Behaviours / Primitives
↓
Controller (owns io + level2)
↓
Level2 (robot-agnostic actions)
↓
hw_io (IOMap implementation, per hardware profile)
↓
SR Robot3 / Arduino / GPIO / real devices

HAL (this folder) is now **optional / transitional**, and should not be extended
unless you are supporting an older code path.

---

## Why HAL is becoming obsolete

HAL was designed around:

- **canonical signal names** (e.g. `PWM_MOT_LEFT`)
- resolved to **pins/channels** (Arduino pins, GPIO, etc.)
- used by a “pinmap-style Level2” as fallback

The new architecture (`hw_io`) replaces this with an explicit object model:

- `IOMap.motors`, `IOMap.servos`, `IOMap.cameras()`
- `IOMap.outputs` (named digital outputs like VACUUM / SOLENOID)
- `IOMap.sleep()` to unify timing

This avoids conflating:
- robot identity
- simulation vs real
- hardware wiring
- device capability

---

## Folder Contents (What’s still here)

### `canonical.py`
Legacy vocabulary of canonical signal names.

### `pinmap.py` / `init_pins.py` / `unified.py`
Legacy “canonical-to-pin” mapping machinery.

### `sr_board.py` / `aux_board.py`
Legacy SR-board-specific helpers.

### `hardware.py`
Legacy SR detection helpers.

---

## Migration Guidance

When you touch code in the robot stack:

1. **Prefer using `Controller.io` (IOMap)**
2. Level2 should call IOMap (motors/servos/outputs/cameras), not SR APIs or pinmaps.
3. If code still uses HAL pinmaps, treat it as migration debt:
   - move it into an IOMap implementation under `hw_io/`
   - or rewrite the caller to use `io.*` instead of pins

---

## Key Principle

> Canonical names describe **intent**.
>
> `hw_io` provides **capability objects** which execute intent on real hardware.
>
> HAL pinmaps describe **wiring** (legacy), and should be phased out.


THE TEXT BELOW IS LEGACY

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
