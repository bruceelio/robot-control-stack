navigation/motion_planning/README_NAV_MOTION_PLANNING.md

# Motion Planning Layer

This document defines the **motion planning layer**, which sits between:

- global path planning (`navigation/path_planning/`)
- path tracking / control (`navigation/path_tracking/`)

---

# Architecture Context

The system is structured as:

- Strategy layer → decides *what to do*
- Path Planning → decides *where to go*
- Motion Planning → decides *how to move feasibly*
- Controller layer → decides *how to follow the path*
- Motion layer → executes commands

---

# Role of Motion Planning

Path planners (e.g. A*) produce:

> A geometric path (waypoints)

Controllers require:

> A feasible, smooth, trackable reference

Motion planning performs the transformation:

> Path → Drivable path or trajectory

---

# Responsibilities

Motion planning is responsible for:

- Path smoothing
- Curvature feasibility
- Velocity profiling
- Local obstacle avoidance (optional)
- Short-horizon replanning (optional)

It ensures the robot can **physically execute** the path.

---

# Holonomic vs Non-Holonomic Motion Constraints

## Description

Holonomic robots (e.g. mecanum drive) are physically capable of independent lateral motion (`lateral_y`). However, allowing unrestricted strafing is not always the most stable, efficient, or tactically correct behavior.

Motion planning is responsible for deciding whether lateral motion should be:

- fully enabled
- partially constrained
- completely disabled

for each path segment or trajectory section.

The downstream path tracking layer must obey these constraints when generating `cmd_vel_*` outputs.

---

# Why This Belongs in Motion Planning

Path planning determines:

> where the robot should go

Path tracking determines:

> how to minimize path-following error

Motion planning determines:

> what motion behavior is physically or operationally feasible

Whether a robot should strafe is fundamentally a motion-feasibility and motion-policy decision.

---

# Motion Modes

## Holonomic Mode

Allows:

```text
linear_x
lateral_y
angular_z
```

Typical for:

- open-field navigation
- docking/alignment
- obstacle bypassing
- precise positioning
- narrow-angle correction
- field-relative driving

### Advantages

- Shorter path execution
- Better obstacle avoidance
- Precise lateral alignment
- Reduced turning requirements

### Disadvantages

- Lower lateral traction
- Increased wheel slip
- Higher localization sensitivity
- More difficult tuning

---

## Constrained Holonomic Mode

Allows lateral motion, but limits magnitude:

```text
max_lateral_y_mps
```

Typical for:

- moderate obstacle density
- uneven surfaces
- carrying unstable payloads
- battery/current limiting
- partial traction loss

### Advantages

- Preserves some holonomic flexibility
- Improves stability
- Reduces lateral slip

### Disadvantages

- Reduced maneuverability
- Longer trajectory execution

---

## Non-Holonomic Mode

Forces:

```text
lateral_y = 0
```

Even on a mecanum robot.

The robot behaves like a differential/tank robot while still using the mecanum motion backend underneath.

Typical for:

- ramps or obstacle transitions
- high-traction pushing/defense
- wheel damage or roller failure
- highly constrained corridors
- poor lateral localization confidence
- slippery or inconsistent flooring
- high-speed travel where strafing instability becomes problematic

### Advantages

- Better forward traction
- More predictable tracking
- Reduced slip
- Simpler controller behavior

### Disadvantages

- Longer path execution
- Larger turning radius
- Reduced agility

---

# Segment-Level Motion Constraints

Motion planning may attach motion constraints to individual path segments or trajectory sections.

Example:

```text
segment.motion_mode = holonomic
segment.max_lateral_y_mps = 0.45
```

or:

```text
segment.motion_mode = nonholonomic
segment.max_lateral_y_mps = 0.0
```

These constraints are consumed by the downstream path tracking/controller layer.

---

# Relationship to Path Tracking

Path tracking/controllers generate robot velocity commands:

```text
cmd_vel_linear_x_mps
cmd_vel_lateral_y_mps
cmd_vel_angular_z_rps
```

The controller must obey motion-planning constraints.

Example:

```python
if motion_mode == "nonholonomic":
    cmd_vel_lateral_y_mps = 0.0
```

The controller itself should not independently decide whether strafing is permitted unless performing emergency local collision avoidance or safety fallback behavior.

---

# Relationship to Motion Backends

Motion backends do not decide whether strafing is allowed.

They only execute the commanded motion.

Example:

```text
VelocityMecanum backend
```

may still be active even when:

```text
lateral_y = 0
```

This preserves a consistent drivetrain execution path while allowing motion planning to dynamically constrain robot behavior.

---

# Example Scenarios

## Narrow Corridor Navigation

In narrow corridors or tight passages, lateral correction may increase collision risk with nearby obstacles.

Motion planning may temporarily disable strafing:

```text
segment.motion_mode = nonholonomic
```

This forces the robot to align itself using rotation rather than lateral translation.

---

## Obstacle Transition or Ramp Crossing

Mecanum rollers have reduced traction compared to standard wheels. During transitions such as ramps, thresholds, cable protectors, or uneven flooring, lateral motion may become unstable.

Motion planning may:

- disable strafing entirely
- or heavily limit `max_lateral_y_mps`

to preserve stability and traction.

---

## Defensive or Pushing Behavior

When pushing another robot or resisting external force, forward traction is significantly stronger than lateral traction on mecanum wheels.

Motion planning may intentionally disable strafing so the drivetrain behaves more like a traditional tank/differential platform.

---

## High-Speed Traversal

At higher speeds, mecanum strafing can amplify localization noise, wheel slip, and oscillation.

Motion planning may constrain lateral motion during long straight segments to improve stability and tracking accuracy.

---

## Poor Localization Confidence

If the localization arbiter reports degraded confidence in `robot_lateral_y_mps` estimation, motion planning may temporarily suppress lateral movement until localization quality improves.

---

## Precision Docking and Alignment

During docking or fine alignment operations, full holonomic motion is often desirable.

Motion planning may explicitly enable unrestricted lateral motion:

```text
segment.motion_mode = holonomic
```

to allow precise sideways correction.

---

# Recommended Initial Strategy

For early implementation:

- Default mecanum robots to holonomic mode
- Allow motion planning to disable strafing in:
  - narrow passages
  - rough terrain
  - obstacle-heavy regions
  - defense/pushing zones

This provides most of the practical benefit without requiring fully dynamic locomotion-mode switching.

# Terminology

Recommended naming:

- motion_planning/
- trajectory/

Avoid:

- "controller" (downstream responsibility)
- "global planner" (upstream responsibility)

---

# Problem Definition

Given:

- A path from the global planner
- Robot constraints (e.g. differential drive)
- Environment information (optional local updates)

Compute:

> A feasible path or trajectory suitable for control

---

# Algorithm Categories

## Path Post-Processing (Required)

### Description

Improves raw planner output.

### Methods

- Line-of-sight pruning
- Waypoint reduction
- Corner smoothing

### Pros

- Very lightweight
- Large performance gain

### Verdict

**Required baseline**

---

## Trajectory Generation

### Description

Converts path into time-parameterised motion.

### Methods

- Constant velocity profiles
- Trapezoidal velocity profiles
- Polynomial trajectories (cubic, quintic)

### Pros

- Smooth motion
- Better tracking performance

### Cons

- Slightly more implementation effort

### Verdict

**Recommended**

---

## Reactive / Local Planning

### Methods

- Dynamic Window Approach (DWA)
- Timed Elastic Band (TEB)

### Pros

- Handles dynamic obstacles
- Real-time adjustments

### Cons

- More complex
- Requires tuning

### Verdict

**Optional upgrade**

---

## Optimization-Based

### Methods

- CHOMP
- TrajOpt
- STOMP

### Pros

- High-quality trajectories

### Cons

- Computationally expensive
- Complex

### Verdict

**Not required for current system**

---

## Velocity Obstacles

### Methods

- VO / RVO / ORCA

### Use Case

- Multi-agent or dynamic environments

### Verdict

**Not required**

---

## Kinodynamic Planning

### Methods

- Kinodynamic RRT
- State lattice planners

### Pros

- Integrates dynamics into planning

### Cons

- High complexity

### Verdict

**Future upgrade**

---

# Recommended Approach

Given:

- Small static environment
- A* global planner
- Differential drive robot
- Limited compute (Pi 4B)

We use:

> **Lightweight motion planning via path smoothing and simple velocity profiling**

---

# Implementation Strategy

## Phase 1

- Path simplification (line-of-sight)
- Remove unnecessary waypoints

## Phase 2

- Add smoothing (optional spline)

## Phase 3

- Add velocity profile

## Phase 4 (optional)

- Local obstacle handling

---

# Package Structure

```text
navigation/
    motion_planning/
        smoothing/
        trajectory/
        velocity/
        interfaces/