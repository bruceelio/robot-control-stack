
# Navigation Module

This folder contains all logic related to **deciding where the robot should go** and **reasoning about space**, independent of how the robot physically moves or senses the environment.

Navigation sits *between* high-level behaviors and low-level motion control.

---

## Design Philosophy

Navigation follows a strict separation of concerns:

- **Localisation** answers: *“Where am I?”*
- **Navigation** answers: *“Where should I go?”*
- **Motion** answers: *“How do I move?”*
- **Behaviors** answer: *“What task am I trying to accomplish?”*

This separation ensures:
- Testability without hardware
- Replaceable planning strategies
- Long-term architectural stability

---

## Folder Structure

```

navigation/
├── **init**.py
├── localisation.py
├── markers.py
├── arena.py
├── geometry.py
├── navigator.py
├── waypoint.py
├── obstacle_aware.py
├── search_routes.py
└── cost_functions.py

```

---

## Module Responsibilities

### `localisation.py`
**Pose estimation**

Tracks the robot’s estimated position and heading.

- Maintains `(x, y, heading)`
- Applies motion updates
- (Future) Sensor fusion, odometry, IMU integration

**Does NOT:**
- Plan paths
- Choose goals
- Detect objects

---

### `markers.py`
**Marker semantics**

Defines marker IDs and their symbolic meaning.

- Categorizes markers (arena, acidic, basic, etc.)
- Maps marker ID → logical type

**Does NOT:**
- Perform geometry math
- Estimate pose
- Plan movement

---

### `arena.py`
**Static world model**

Defines the known layout of the environment.

- Arena dimensions
- Fixed marker locations
- Known reference points

**Does NOT:**
- Track robot state
- Plan routes
- Execute movement

---

### `geometry.py`
**Pure spatial math utilities**

Reusable geometry and math functions.

- Distance and angle calculations
- Circle intersections
- Triangulation helpers

**Rule of thumb:**
> Code here should work even if the robot didn’t exist.

---

### `navigator.py`
**Navigation interface / coordinator**

The main entry point for navigation logic.

- Accepts navigation goals (e.g. pose, waypoint)
- Tracks navigation progress
- Delegates to specific strategies

**Behaviors interact with this file, not planners directly.**

**Does NOT:**
- Control motors
- Read sensors directly

---

### `waypoint.py`
**Simple navigation strategy**

Baseline, straight-line navigation.

- Go-to-point logic
- Minimal assumptions
- First working planner

Used as:
- Default strategy
- Fallback
- Reference implementation

---

### `obstacle_aware.py`
**Reactive navigation logic**

Adds obstacle reasoning on top of basic navigation.

- Detours
- Path adjustments
- Reactive avoidance

**Does NOT:**
- Execute motor commands
- Sense the environment directly

---

### `search_routes.py`
**Exploration and search patterns**

Defines systematic movement patterns.

- Sweep routes
- Spiral searches
- Coverage strategies

Typically used by:
- Seek-and-collect behaviors
- Exploration behaviors

---

### `cost_functions.py`
**Path scoring utilities**

Evaluates and compares navigation options.

- Distance cost
- Risk penalties
- Visibility or confidence scoring

Used by planners to choose *better* paths, not to execute them.

---

## What Navigation Does NOT Do

Navigation explicitly avoids:

- Motor control
- Timing or actuation
- Direct sensor reads
- Hardware-specific logic

Those belong in:
- Motion backends
- HAL
- Perception modules

---

## Architectural Rule of Thumb

If you’re unsure where code belongs, ask:

> “Is this deciding **where** to go, or **how** to move?”

- **Where to go** → `navigation/`
- **How to move** → motion / Level2
- **What I see** → perception
- **What task I’m doing** → behaviors

---

## Future Extensions (Intentional, Optional)

This architecture supports future additions without breaking behaviors:

- Dynamic maps
- SLAM-based localisation
- Multiple competing planners
- Costmap-based navigation
- Performance optimization

These are **deliberately not included yet** to keep the system simple and robust.

---

## Summary

The navigation module is responsible for **spatial reasoning and decision-making**, not execution.

It provides a clean, testable interface that allows behaviors to request movement without knowing *how* navigation is implemented.

This structure is designed to scale from simple simulation to complex real-world robotics without architectural rewrites.
```


