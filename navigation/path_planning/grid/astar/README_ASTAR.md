navigation/astar/README_ASTAR.md

# Navigation & A* Path Planning Design

This document describes the design and implementation plan for global path planning using A* within the robot-control-stack.

It reflects the Student Robotics 2026 arena constraints and the current system architecture.

---

# Overview

The navigation system is split into three layers:

- **Strategy layer**
  - Chooses *what* to do (which sample, which type, return timing)
- **Navigation (A*) layer**
  - Decides *where to go* (global path to an approach pose)
- **Motion / control layer**
  - Executes *how to move* (rotate, drive, arc, etc.)

A* is used as a **global planner only**, not a motion controller.

---

# Arena Model Assumptions

- Arena: ~4575 mm × 4575 mm
- Central console:
  - ~1220 mm × 1220 mm
  - Raised (not driveable)
- Samples:
  - Floor samples (fully reachable)
  - Console samples (reachable via arm from edge)
- Labs:
  - Corner zones, starting positions

### Key Design Decision

- The **central console is treated as a hard obstacle for the chassis**
- But **console samples are still valid targets via edge approach poses**

---

# Core Concepts

## 1. Sample Target vs Approach Pose

We explicitly separate:

- **SampleTarget** → physical object in the world
- **ApproachPose** → where the robot should drive to

A* always plans to an **ApproachPose**, never directly to a sample.

---

## 2. Approach Pose Strategy

Each sample has **multiple candidate approach poses**:

- Floor sample → typically 1–2 poses
- Console sample → 2–4 poses around accessible edges

At runtime:
- We evaluate all candidates
- Choose the cheapest reachable one

This provides robustness against:
- Obstacles
- Other robots
- Bad geometry

---

## 3. Opening Strategy

Unlike a generic planner, the robot does **not** start by exiting the lab.

Instead:
- The opening move is **directly toward a chosen sample**
- Strategy is **pre-configured before each match**

Example:

```yaml
opening:
  sample_type: ACID
  preference: CONSOLE
````

---

# A* Planning Approach

## Grid Representation

* Cell size: 50–100 mm
* Grid size: ~50×50 to 100×100
* 8-connected movement

## Costs

* Orthogonal: 10
* Diagonal: 14
* Heuristic: octile distance

Additional penalties:

* Near walls
* Near console
* Tight spaces

---

## Obstacle Handling

We model:

* Arena boundaries
* Central console (inflated)
* Lab keep-out zones
* Dynamic obstacles (optional)

### Inflation

Obstacle inflation includes:

* Half robot width
* Localization error
* Safety margin

Typical: **300–350 mm**

---

# System Architecture

## Folder Layout

```
navigation/
    arena.py
    navigator.py
    types.py
    targets.py
    astar/
        __init__.py
        grid.py
        planner.py
        heuristic.py
        simplify.py
```

---

## Data Models

### Geometry

* `Point2D`
* `Pose2D`
* `GridCoord`

### Arena

* `ArenaMap`
* `RectObstacle`

### Targets

* `SampleTarget`
* `ApproachPose`

### Planning

* `PlannerRequest`
* `PlannerResult`
* `PathWaypoint`

---

# Planning Pipeline

1. Strategy selects `SampleTarget`
2. Generate candidate `ApproachPose`s
3. Navigator selects best candidate
4. Build `PlannerRequest`
5. Run A*
6. Simplify path → waypoints
7. Execute via motion primitives
8. Arm handles pickup

---

# Modified Implementation Plan (10 Steps)

## Step 1 — Use A* as Global Planner

A* only computes paths between poses.

Motion primitives handle execution.

---

## Step 2 — Build Occupancy Grid

* Fixed-size grid
* Static arena geometry
* Optional dynamic updates

---

## Step 3 — Inflate Obstacles

Account for robot size and uncertainty.

---

## Step 4 — Plan Directly to First Sample

**Changed from original design**

* No "exit lab" phase
* First goal = approach pose of selected sample

---

## Step 5 — Standard A* (8-connected)

* Octile heuristic
* Simple cost model
* Fast enough for Pi 4B

---

## Step 6 — Static World Model First

Start with:

* Known arena
* No SLAM

Add dynamic obstacles later.

---

## Step 7 — Plan to Approach Poses

Never plan to sample centers.

Always plan to:

* Offset position
* Correct heading

---

## Step 8 — Event-Based Replanning

Replan only when:

* Path blocked
* Target changes
* Robot deviates
* Execution fails

---

## Step 9 — Modular A* Package

Use:

```
navigation/astar/
```

Keeps:

* Planner logic isolated
* Code maintainable
* Easy to swap algorithms later

---

## Step 10 — Strategy Above Planner

A* does NOT decide:

* Which sample to pick
* Whether to use console or floor
* When to return to lab

That belongs in the **strategy layer**

---

# Special Handling: Console Samples

* Console is **non-traversable**
* Samples are reachable via arm

We:

* Generate edge approach poses
* Prefer poses with:

  * Good alignment
  * Escape space after pickup

---

# Performance Notes (Pi 4B, 2GB)

* Grid size is small → fast A*
* Typical planning time: <10 ms
* No need for heavy optimization initially

---

# Future Improvements

* Dynamic obstacle avoidance
* Cost maps (traffic zones)
* Path smoothing (splines)
* Multi-target planning
* Time-aware planning

---

# Summary

This design:

* Keeps A* simple and robust
* Matches the repo architecture
* Supports both floor and console pickups
* Allows strategy flexibility
* Runs comfortably on Raspberry Pi 4B

---

# Next Steps

1. Implement core data models
2. Build static arena map
3. Generate sample + approach pose data
4. Implement occupancy grid
5. Implement A* planner
6. Integrate with Navigator
7. Test in simulation

```
