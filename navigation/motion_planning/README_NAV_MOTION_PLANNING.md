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