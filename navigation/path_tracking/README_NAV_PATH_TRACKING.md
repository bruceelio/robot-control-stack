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

