# Configuration System

This directory defines the **entire configuration model** for the robot system.

All configuration is **declarative**, **explicit**, and **resolved exactly once** at startup into a single immutable `Config` object.

No runtime code should read configuration directly from files in this directory — everything flows through `schema.resolve()`.

---

## Design Goals

- One **single source of truth** at runtime (`Config`)
- No hidden constants
- No implicit imports
- No circular dependencies
- Easy to switch between:
  - robots
  - competitions
  - environments (simulation / real)
- Easy to audit and print configuration before execution
- Future-proof for multi-robot and multi-camera systems

---

## High-Level Flow

```

arena.py        ┐
robots/*.py    │
strategy.py    ├──► schema.resolve(...) ───► Config (immutable)
│
competition.py ┘

````

At startup:

1. The appropriate **arena**, **robot profile**, and **strategy** modules are selected
2. `schema.resolve()` validates and combines them
3. A frozen `Config` dataclass is produced
4. The resolved configuration is printed
5. All runtime code consumes `config.CONFIG`

---

## Files & Responsibilities

### `__init__.py`

Entry point for the configuration system.

Responsibilities:
- Select arena / robot / strategy modules
- Call `schema.resolve(...)`
- Expose the resolved `CONFIG`
- Print the resolved configuration for audit

Example:
```python
from config.schema import resolve
from config import arena
from config.robots import simulation
from config import strategy

CONFIG = resolve(
    arena=arena,
    profile=simulation,
    strategy=strategy,
)
````

---

### `schema.py`

**The heart of the configuration system.**

Responsibilities:

* Define the `Config` dataclass (single source of truth)
* Define validation tables
* Define `RESOLVE_MAP` (explicit mapping of sources → config fields)
* Compute derived calibration values
* Validate and freeze configuration

Key properties:

* Declarative mapping (no hard-coded assignments)
* Explicit failure if anything is missing
* No defaults hidden in code

Nothing outside `schema.py` should construct `Config`.

---

### `arena.py`

Defines **competition geometry**.

Typical contents:

* Arena size
* Physical layout constants

Example:

```python
ARENA_SIZE = 6000
```

Different competitions = different arena modules.

---

### `robots/`

Defines **robot-specific configuration**.

Each file represents a robot (or robot class):

```
robots/
├── simulation.py
├── sr1.py
└── future_robot.py
```

Robot profiles contain:

* Mechanical properties
* Motion constraints
* Vision calibration
* Surface / drive behaviour
* Camera-related constants

Even in simulation, robot profiles should be realistic and explicit.

---

### `strategy.py`

Defines **behavioural policy**, not mechanics.

Typical contents:

* Default target selection
* High-level run-time choices that may change between matches

Example:

```python
DEFAULT_TARGET_KIND = "basic"
```

Strategy is intentionally separated so behaviour can change without touching robot calibration.

---

### `competition.py` (optional / future)

Intended for:

* Competition-specific rules
* Game variants
* Scoring or match-level parameters

Currently optional, but structurally supported.

---

## Resolved Configuration (`Config`)

After resolution, **all configuration lives in one object**:

```python
from config import CONFIG
```

Properties:

* Immutable (`@dataclass(frozen=True)`)
* Fully validated
* Fully resolved
* Safe to use anywhere

No module should:

* Import robot profiles directly
* Import arena directly
* Import strategy directly

---

## Printing & Auditing

The resolved configuration is printed automatically at startup:

```
=== RESOLVED CONFIGURATION ===
{ ... }
=== END CONFIGURATION ===
```

This serves as:

* A pre-run checklist
* A debugging aid
* A record of exactly what the robot believed was true

---

## Adding a New Robot

1. Create a new file in `robots/`
2. Define all required constants
3. Select it in `config/__init__.py`
4. Run — missing fields will fail fast

No other changes required.

---

## Adding a New Competition

1. Create a new `arena.py` or arena variant
2. Optionally add a competition module
3. Select them in `config/__init__.py`

---

## Philosophy

> Profiles describe **intent**
> Schema produces **facts**
> Runtime code consumes **truth**

This separation is deliberate and enforced.

---

## Anti-Patterns (Do Not Do This)

* ❌ Reading constants directly from robot profiles
* ❌ Defining configuration inside runtime code
* ❌ Modifying configuration at runtime
* ❌ Using globals instead of `Config`
* ❌ Hiding defaults in logic

---

## Enforcement

Runtime code must only import:
```python
 from config import CONFIG

---

## Summary

This configuration system is designed to scale from:

* single robot → multiple robots
* simulation → real hardware
* single competition → many competitions

All while remaining:

* explicit
* auditable
* debuggable
* predictable

If configuration feels boring — it’s working.


