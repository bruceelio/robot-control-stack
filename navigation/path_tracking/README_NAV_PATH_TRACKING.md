navigation/path_tracking/README_NAV_PATH_TRACKING.md

# Alternative Path-Following Controllers

This document compares several path-following controllers suitable for use alongside A* within:

navigation/controllers/

It is intended to complement the global planner design and support a modular navigation stack.

---

# Architecture Context

- Strategy layer → decides *what to do*
- Path Planning → decides *where to go*
- Motion Planning → decides *how to move feasibly*
- Controller layer → decides *how to follow the path*
- Motion layer → executes commands

A* is a **global planner only**.

Controllers track a trajectory or refined path produced by motion planning.

---

# Terminology

Recommended naming:

- planners/
- controllers/

"Controller" is the standard robotics term.

Avoid "local planner" (overloaded, especially in ROS).

---

# Environment Assumptions

- Indoor arena (~4575 mm square)
- Static known geometry
- Central obstacle (console)
- Moderate speeds
- Raspberry Pi 4B (2GB)
- Differential drive robot

Execution modes:

- Stepped: rotate + drive
- Continuous: (v, omega)

---

# Controller Options

## Pure Pursuit

### Description
Tracks a lookahead point on the path.

### Pros
- Simple
- Smooth
- Lightweight
- Works with both execution modes

### Cons
- Lookahead tuning required
- Can cut corners

### Verdict
**Best primary controller**

---

## PID Waypoint Follower

### Description
Tracks successive waypoints using PID control.

### Pros
- Very simple
- Easy to debug

### Cons
- Not smooth
- Can be stop-start
- Needs heuristics

### Verdict
**Good fallback / debug controller**

---

## Stanley Controller

### Description
Uses heading + cross-track error.

### Pros
- Good path accuracy
- Handles offset well

### Cons
- Less intuitive tuning
- Can oscillate at low speed

### Verdict
**Good secondary option**

---

## Ramsete

### Description
Nonlinear trajectory tracking controller.

### Pros
- Accurate tracking
- Designed for diff drive

### Cons
- Needs trajectory (not path)
- More complex

### Verdict
**Only if moving to trajectory control**

---

## LQR

### Description
Optimal state-space controller.

### Pros
- Elegant
- Accurate

### Cons
- Requires modelling
- Complex

### Verdict
**Not worth it here**

---

## MPC

### Description
Optimises control over a time horizon.

### Pros
- Very powerful
- Handles constraints

### Cons
- Heavy
- Complex

### Verdict
**Overkill**

---

# Comparison Table

| Controller | Complexity | Compute Cost | Tuning Difficulty | Smoothness | Path Accuracy | Stepped | Continuous | Recommended |
|-----------|-----------|--------------|------------------|-----------|--------------|--------|------------|-------------|
| Pure Pursuit | Low | Very low | Low | Good | Good | Yes | Yes | YES |
| PID Follower | Low | Very low | Low–Med | Fair | Fair | Yes | Yes | Yes (fallback) |
| Stanley | Low–Med | Very low | Medium | Good | Good–Very good | Limited | Yes | Maybe |
| Ramsete | Medium | Low | Medium | Very good | Very good | No | Yes | Later |
| LQR | Med–High | Low | High | Very good | Very good | No | Yes | No |
| MPC | High | Med–High | High | Excellent | Excellent | No | Yes | No |

---

# Pi 4B Suitability

All lightweight controllers are easily supported:

- Pure Pursuit
- PID
- Stanley

Even advanced methods are possible, but not worthwhile.

Key constraints are:
- engineering time
- robustness
- tuning complexity

---

# Recommended Roadmap

## Phase 1
- Pure Pursuit

## Phase 2
- PID fallback

## Phase 3
- Stanley

## Phase 4 (optional)
- Ramsete

---

# Package Structure

navigation/
    controllers/
        base.py
        pure_pursuit/
        pid_follower/
        stanley/

---

# Key Insight

Performance depends more on:

- localisation
- path quality
- obstacle inflation
- execution reliability

than controller complexity.

---

# Bottom Line

Use:

1. Pure Pursuit (primary)
2. PID (fallback)
3. Stanley (optional)

Avoid over-engineering early.

---

# Controller Selection Strategy

Although a single primary path-tracking controller is usually sufficient for most navigation tasks, different controllers perform better under different operational conditions.

For this reason, the navigation architecture may optionally support dynamic controller selection based on:

- navigation state
- motion mode
- environment complexity
- localization quality
- speed profile
- precision requirements

Controller switching should be treated as a high-level navigation or strategy decision, not a low-level continuous optimization loop.

---

# Recommended Default Strategy

For most robots:

```text
Pure Pursuit
```

should remain the primary controller for normal autonomous navigation.

Additional controllers should be introduced only for specialized scenarios where their behavior provides a clear advantage.

---

# Typical Controller Roles

## Pure Pursuit

Best general-purpose controller.

Recommended for:

- normal autonomous navigation
- smooth corridor traversal
- moderate-speed movement
- low-compute systems
- most waypoint following

### Advantages

- Smooth motion
- Lightweight
- Easy to tune
- Stable under most conditions

### Weaknesses

- Can cut corners
- Less precise for final alignment
- Less aggressive cross-track correction

---

## PID Waypoint Follower

Best precision/utility controller.

Recommended for:

- final docking
- precision alignment
- exact waypoint convergence
- low-speed maneuvering
- fallback/debug operation

### Advantages

- Simple and predictable
- Good for exact positioning
- Easy to debug

### Weaknesses

- Less smooth
- Can produce stop-start motion
- Poor high-speed behavior

---

## Stanley Controller

Best path-accuracy controller.

Recommended for:

- high-speed travel
- long straight corridors
- aggressive cross-track correction
- environments where path accuracy is more important than smoothness

### Advantages

- Strong path convergence
- Good cross-track handling
- Maintains path accuracy well

### Weaknesses

- More difficult tuning
- Can oscillate at low speeds
- Less intuitive behavior than Pure Pursuit

---

# Example Operational Scenarios

## General Autonomous Navigation

Use:

```text
Pure Pursuit
```

This should remain the default controller for most normal navigation tasks.

---

## Final Docking or Alignment

Use:

```text
PID Waypoint Follower
```

Typical examples:

- charging dock alignment
- AprilTag alignment
- pickup/dropoff positioning
- precise robot staging

Reason:

Pure Pursuit intentionally smooths trajectories and may not aggressively converge on an exact final pose.

---

## High-Speed Traversal

Use:

```text
Stanley
```

Typical examples:

- long straight travel
- fast hallway traversal
- open-field navigation

Reason:

Stanley aggressively minimizes cross-track error and often maintains better path accuracy at speed.

---

## Narrow Indoor Navigation

Use:

```text
PID Waypoint Follower
```

or:

```text
Pure Pursuit with reduced lookahead
```

Typical examples:

- narrow aisles
- obstacle-dense regions
- constrained indoor movement

Reason:

Aggressive path smoothing may increase collision risk in tight environments.

---

## Localization Degradation

Fallback to:

```text
PID Waypoint Follower
```

Reason:

Pure Pursuit depends heavily on stable lookahead geometry and smooth pose estimation. Simpler local error correction may behave more predictably under degraded localization conditions.

---

## Holonomic Mecanum Navigation

Mecanum robots may eventually benefit from controllers specialized for holonomic motion.

Examples:

```text
PurePursuitMecanum
PIDHolonomic
```

rather than reusing controllers originally designed around non-holonomic vehicle assumptions.

---

# Relationship to Motion Planning

Motion planning decides:

- whether holonomic motion is allowed
- velocity constraints
- feasible trajectory shape
- segment motion constraints

Path tracking/controllers decide:

- how to minimize trajectory/path error

Controllers must obey motion-planning constraints.

Example:

```python
if segment.motion_mode == "nonholonomic":
    cmd_vel_lateral_y_mps = 0.0
```

---

# Relationship to Motion Backends

Path tracking/controllers produce canonical robot velocity commands:

```text
cmd_vel_linear_x_mps
cmd_vel_lateral_y_mps
cmd_vel_angular_z_rps
```

Motion backends consume those commands and convert them into drivetrain-specific motor outputs.

Controllers should never directly generate motor power commands.

---

# Recommended Architecture

```text
Path Planning
        ↓
Motion Planning
        ↓
Path Tracking Controller
    Pure Pursuit
    PID Follower
    Stanley
        ↓
Path Tracking Mux
        ↓
Canonical cmd_vel outputs
        ↓
Motion Backend
```

---

# Controller Switching Guidance

Dynamic controller switching should be performed carefully.

Improper switching can introduce:

- discontinuous velocity outputs
- oscillation
- transient heading jumps
- integrator windup
- unstable path convergence

Controller transitions should preferably occur at:

- segment boundaries
- waypoint completion
- low-speed states
- stopped robot conditions

Avoid switching controllers continuously during aggressive motion.

---

# Recommended Initial Roadmap

## Phase 1

Primary controller:

```text
Pure Pursuit
```

---

## Phase 2

Add:

```text
PID Waypoint Follower
```

for docking, debugging, and precision alignment.

---

## Phase 3

Add:

```text
Stanley
```

for improved high-speed path accuracy.

---

## Phase 4 (optional)

Add:

```text
Ramsete
```

if transitioning toward fully time-parameterized trajectory tracking.

navigation/
└── path_tracking/
    ├── __init__.py
    ├── path_tracking_arbiter.py      # selects active tracker and outputs canonical pt_cmd_vel_*
    ├── base_tracker.py               # shared interface / abstract base class
    ├── tracker_types.py              # enums/constants: PURE_PURSUIT, PID, STANLEY, MECANUM, NONMEC
    ├── tracking_config.py            # dataclasses/config validation
    ├── tracking_state.py             # shared runtime state/result objects
    ├── tracking_utils.py             # angle wrap, distance, lookahead helpers
    │
    ├── pure_pursuit_nonmec.py
    ├── pure_pursuit_mecanum.py
    │
    ├── pid_nonmec.py
    ├── pid_mecanum.py
    │
    ├── stanley_nonmec.py
    └── stanley_mecanum.py