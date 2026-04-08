navigation/controllers/README_PURE_PURSUIT.md

# Navigation & Pure Pursuit Path Following Design

This document describes the design and implementation plan for **Pure Pursuit** path following within the `robot-control-stack`.

It is intended to complement the global A* planner, not replace it.

The design reflects:

- the current project architecture,
- the Student Robotics 2026 arena,
- the desire to support both **stepped** and **continuous** execution modes.

---

# Overview

The navigation stack is split into four distinct responsibilities:

- **Strategy layer**
  - Chooses *what to do* (which sample, which type, return timing)
- **Planning layer**
  - Decides *where to go* (global path to an approach pose)
- **Control layer**
  - Decides *how to follow the chosen path*
- **Motion / backend layer**
  - Executes *how to actuate the robot* (rotate, drive, arc, velocity commands)

Pure Pursuit belongs in the **control layer**.

It consumes a path produced by a planner such as A* and produces short-horizon motion commands which move the robot along that path.

---

# Naming and Terminology

## Recommended package naming

```text
navigation/
    planners/
        a_star/
    controllers/
        pure_pursuit/
            __init__.py
            controller.py
            geometry.py
            lookahead.py
            types.py
            adapters.py
```

## Why `controllers`?

`controllers` is the most typical robotics term for this layer.

A planner computes a path.
A controller tracks or follows that path.

Terms such as **path follower** or **tracker** are also valid, but are usually used as descriptive sub-types of controller rather than as the main architectural layer name.

So the recommended terminology is:

- `planners/` for A*, D*, RRT, etc.
- `controllers/` for Pure Pursuit, Stanley, PID path tracking, MPC, etc.

Within that, Pure Pursuit can still be described as a **path-following controller**.

---

# Role of Pure Pursuit in this Stack

A* remains the **global planner**.

Pure Pursuit becomes the **path-following controller**.

That means:

- A* chooses a collision-aware route around arena obstacles
- Pure Pursuit follows that route smoothly
- motion backends and primitives execute the resulting command

This matches the architecture rule that navigation reasons about space rather than motors, while primitives and backends perform execution.

---

# Why Pure Pursuit is a Good Fit Here

For Student Robotics 2026, the arena is approximately **4575 mm square**. The central area is a raised deck in the middle of the arena measuring **1220 ± 50 mm by 1220 ± 50 mm**, elevated **180 ± 30 mm**, with **solid walls and no markers**. The arena floor uses **textured interlocking foam tiles** rather than carpet, and robots must remain within a **600 mm × 600 mm** horizontal square during the match. These details matter because the central deck is a hard obstacle for the chassis, marker coverage is worse near it, and wheel slip / tracking error may be less predictable on foam than on carpet. [1]

Pure Pursuit complements A* well in this environment because A* provides safe global routing, while Pure Pursuit improves local path tracking, smooths cornering, and reduces stop-turn-drive behaviour around inflated obstacles.

---

# Core Design Decision

Pure Pursuit should be implemented as a **controller over a path**, not as a direct target-approach routine.

That means it should track a sequence of waypoints or a resampled path, rather than repeatedly aiming at the final goal pose.

In this stack:

- planner output = path
- controller input = current pose + path
- controller output = tracking command
- motion layer input = tracking command

---

# Supported Execution Modes

The implementation should support two output modes from the start.

## 1. Stepped Pure Pursuit

This is the initial implementation target.

The controller computes a short-horizon tracking request, which is then converted into primitives such as:

- rotate a small angle
- drive a short distance

This fits the current stack well because the existing motion system already works naturally with discrete actions.

### Advantages

- simple to integrate
- easy to debug
- fits primitive-based execution
- safe for early testing

### Trade-offs

- less smooth than continuous control
- more stop-start behaviour
- tracking quality depends on step size

---

## 2. Continuous Pure Pursuit

Later, the same controller logic should also support a continuous mode where it produces values such as:

- linear velocity `v`
- angular velocity `omega`

or an equivalent curvature-based command.

This would allow tighter tracking and smoother motion if the backend grows a continuous command interface.

### Advantages

- smoother path tracking
- better performance on long curves
- less command quantisation

### Trade-offs

- requires backend support for continuous actuation
- slightly harder to tune
- may require tighter safety limits and watchdog logic

---

# Important Architectural Principle

The **Pure Pursuit algorithm itself should stay independent of whether execution is stepped or continuous**.

The cleanest split is:

- **Pure Pursuit controller** computes geometric tracking information
- **adapter / executor layer** converts that into either:
  - stepped primitives, or
  - continuous velocity commands

This keeps the control logic reusable.

---

# Recommended Internal Structure

## `controller.py`

Contains the main `PurePursuitController`.

Responsibilities:

- accept current pose and path
- select lookahead point
- compute curvature / heading demand
- return a generic tracking result

## `lookahead.py`

Contains logic for:

- projecting robot pose onto path
- selecting the lookahead point at a specified distance
- advancing along resampled path segments

## `geometry.py`

Contains pure geometry helpers:

- frame transforms
- arc / curvature math
- path projection
- angle normalisation

## `types.py`

Contains data models such as:

- `PathPoint`
- `TrackedPath`
- `LookaheadTarget`
- `TrackingCommand`
- `ControlMode`

## `adapters.py`

Converts controller output into:

- stepped rotate+drive requests
- continuous `(v, omega)` commands

---

# Planner / Controller Interface

Pure Pursuit should not operate on raw grid cells.

A* output should first be processed into a follower-friendly path.

Recommended pipeline:

1. A* computes a grid path
2. path is simplified
3. path is resampled at fixed spacing
4. Pure Pursuit follows the resampled path

This makes tracking more stable and avoids pathological behaviour caused by staircase-like grid paths.

---

# Path Requirements

The Pure Pursuit controller should assume the path:

- is collision-safe at the planner level
- has already been simplified
- is sampled densely enough for smooth tracking
- ends at a valid approach pose, not at the centre of a sample

Recommended initial path spacing:

- **50-100 mm**, with **75 mm** as a good starting point

---

# Lookahead Strategy

Pure Pursuit works by selecting a point ahead of the robot on the path and steering toward that point.

## Fixed lookahead

Simplest starting point:

- one constant lookahead distance

Good for first implementation, but not always ideal.

## Adaptive lookahead

Preferred longer-term approach:

- longer lookahead on straight segments
- shorter lookahead near tight turns
- shorter lookahead near the goal
- optionally shorter lookahead in cluttered areas or near the console

Recommended starting values:

- **200 mm** in tight spaces / near target
- **300-450 mm** in open space

---

# Basic Pure Pursuit Geometry

At each control update:

1. determine the closest or projected point on the path
2. select a lookahead point ahead by distance `L_d`
3. transform that point into the robot frame
4. compute curvature toward the lookahead point
5. convert curvature into an executable command

A common differential-drive form is:

```text
kappa = 2 * y / (L_d^2)
```

where:

- `y` is the lateral offset of the lookahead point in robot coordinates
- `L_d` is the lookahead distance
- `kappa` is the desired curvature

The controller does not need to know motor PWM values.
It only needs to express the geometric turning demand.

---

# Output Abstraction

To support both execution styles, the controller should return a generic output.

Example concept:

```python
class TrackingCommand:
    curvature: float
    lookahead_distance_mm: float
    cross_track_error_mm: float
    heading_error_rad: float
    target_speed_scale: float
```

Then downstream code can convert that into either:

- stepped motion:
  - rotate angle
  - drive distance
- continuous motion:
  - `v`
  - `omega`

This keeps the mathematical layer clean and testable.

---

# Stepped Mode Design

In stepped mode, the adapter converts the tracking command into small primitive actions.

Typical behaviour:

- if heading error is large, rotate first
- otherwise drive a short step while applying bounded steering intent
- repeat until goal tolerance is reached

Recommended initial values:

- forward step: **80-150 mm**
- maximum rotation per step: **10-20°**
- reduce step size when curvature is high

This is likely the best first implementation for the current stack.

---

# Continuous Mode Design

In continuous mode, the adapter converts curvature into:

```text
omega = kappa * v
```

with velocity regulation.

Velocity should reduce when:

- curvature is high
- the robot is near the goal
- localisation confidence is poor
- the path runs near inflated obstacles

This mode should remain optional until the backend layer provides a stable continuous command interface.

---

# Goal Handling

Pure Pursuit should stop when the robot reaches the approach pose within tolerance.

Recommended initial tolerances:

- position tolerance: **50-80 mm**
- heading tolerance: only enforce when heading matters for pickup

For some approach poses, final heading matters strongly.
For others, position may matter more than exact yaw.

---

# Interaction with Console Approach Poses

The A* README already defines an important distinction between a `SampleTarget` and an `ApproachPose`, and states that planning should always go to an approach pose rather than directly to a sample. That is the right model for Pure Pursuit as well. [2]

For console samples in particular:

- the central deck is non-traversable for the chassis
- the robot should drive to an offset pose near an accessible edge
- the final heading often matters for arm reach and pickup reliability

Pure Pursuit therefore follows a path to the **approach pose**, not to the sample centre.

---

# Replanning Rules

Pure Pursuit should not replace replanning.

A replan should be triggered when:

- the path becomes invalid
- robot deviation from the path grows too large
- a target changes
- execution repeatedly fails
- a dynamic obstacle blocks the route

A good separation is:

- planner handles route changes
- controller handles ordinary path-tracking error

---

# Arena-Specific Considerations

The SR2026 arena includes a central deck with no markers on its walls, while boundary markers are placed on the outer walls. That means localisation may be weaker near the central deck than near the perimeter. [1]

As a result, Pure Pursuit should behave conservatively near the console:

- reduce speed or step size
- reduce lookahead distance
- use larger safety margins in planning
- prefer stable approach geometry over aggressive corner-cutting

The foam tile floor is also different from previous carpeted arenas, so tuning should assume some slip and heading drift under acceleration or turning. [1]

---

# Performance Notes (Pi 4B, 2GB)

Pure Pursuit is computationally lightweight.

For the expected arena size and path lengths in this competition, the algorithm should run comfortably on a Raspberry Pi 4B with 2GB RAM.

The main implementation priority should therefore be **clarity and testability**, not premature optimisation.

---

# Initial Implementation Plan

## Step 1 — Create controller package

Add:

```text
navigation/controllers/pure_pursuit/
```

## Step 2 — Define controller data types

Implement:

- `PathPoint`
- `TrackingCommand`
- `ControlMode`
- `GoalTolerance`

## Step 3 — Implement geometry helpers

Implement:

- pose transforms
- nearest-point / projection helpers
- curvature calculation
- angle wrapping

## Step 4 — Implement lookahead selection

Support:

- path projection
- fixed lookahead
- adaptive lookahead hooks

## Step 5 — Implement Pure Pursuit controller core

Input:

- current pose
- path
- controller config

Output:

- generic `TrackingCommand`

## Step 6 — Implement stepped adapter first

Convert tracking command to:

- rotate primitive request
- drive primitive request

## Step 7 — Integrate with Navigator

Navigator should:

- request a path from planner
- pass path + pose to controller
- dispatch resulting motion command
- request replanning when needed

## Step 8 — Add continuous adapter later

When backend support exists, convert tracking command into:

- linear velocity
- angular velocity

## Step 9 — Tune in simulation first

Tune:

- lookahead distance
- step size
- heading thresholds
- goal tolerances

## Step 10 — Validate on real arena surface

Pay special attention to:

- slip on foam tiles
- localisation quality near console
- final approach consistency for pickup

---

# Suggested Future Extensions

- adaptive lookahead based on speed and curvature
- reverse-capable path following if ever needed
- regulated speed selection based on curvature
- confidence-aware slowdown near localisation-poor regions
- controller debug visualisation
- path progress metrics and failure counters
- optional final pose alignment controller

---

# Summary

This design keeps Pure Pursuit in the correct architectural role:

- **strategy** decides the objective
- **planner** computes the route
- **controller** tracks the route
- **motion layer** executes the command

It also keeps the implementation flexible:

- stepped execution can be implemented first
- continuous execution can be added later
- the same Pure Pursuit core supports both

This should integrate cleanly with the existing A* design and provide a natural next step for smoother, more robust navigation.

---

# References

[1] Student Robotics 2026 Rulebook — arena size, central area dimensions, marker placement, foam floor, and robot size rules.
[2] `README_ASTAR.md` — current A* design and approach-pose model.
