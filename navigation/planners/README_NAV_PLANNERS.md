# Alternative Path Planning Algorithms

This document compares several path planning algorithms suitable for use within:

navigation/planners/

It is intended to complement the navigation architecture and justify the selection of A* as the primary global planner.

---

# Architecture Context

The system is structured as:

- Strategy layer → decides *what to do*
- Planner layer → decides *where to go*
- Controller layer → decides *how to follow the path*
- Motion layer → executes commands

The planner produces a **collision-free path** (sequence of waypoints or poses).

Controllers convert this path into motion.

---

# Terminology

Recommended naming:

- planners/
- controllers/

"Planner" refers to global path planning.

Avoid mixing:
- planning (global reasoning)
- control (path tracking)

---

# Environment Assumptions

- Indoor arena (~4575 mm square)
- Static, known geometry
- Central obstacle (console)
- Discrete targets (samples)
- Differential drive robot (non-holonomic)
- Raspberry Pi 4B (2GB)

Key characteristics:

- Small environment
- Low dynamic complexity
- Hard obstacles
- Real-time responsiveness required

---

# Planning Problem

We must compute:

> A collision-free path from current pose to a target approach pose

Constraints:

- Robot has a defined footprint
- Cannot pass through obstacles
- Should produce **driveable paths**
- Must run quickly on limited hardware

---

# Planner Options

## A* (A-Star)

### Description
Graph search algorithm using cost + heuristic:

f(n) = g(n) + h(n)

- g(n): cost from start
- h(n): estimated cost to goal

Typically implemented on a grid.

---

### Pros

- Optimal (with admissible heuristic)
- Fast for small grids
- Easy to implement
- Deterministic
- Supports obstacle inflation
- Works well with static maps
- Easy to debug

---

### Cons

- Produces grid-like paths (needs smoothing)
- Ignores robot kinematics (initially)
- Can generate sharp turns

---

### Verdict

**Best choice for this project**

---

## Dijkstra

### Description
Uniform-cost search (A* without heuristic)

---

### Pros

- Guaranteed optimal
- Simple

---

### Cons

- Explores entire space
- Much slower than A*
- No directionality toward goal

---

### Verdict

**Strictly worse than A*** for this use case

---

## Greedy Best-First Search

### Description
Uses heuristic only:

f(n) = h(n)

---

### Pros

- Very fast
- Simple

---

### Cons

- Not optimal
- Can get stuck in poor paths
- Unreliable in cluttered environments

---

### Verdict

**Too unreliable**

---

## Breadth-First Search (BFS)

### Description
Explores uniformly without cost weighting

---

### Pros

- Simple
- Finds shortest path (unweighted)

---

### Cons

- Extremely inefficient for large spaces
- No cost model
- No heuristic

---

### Verdict

**Not suitable**

---

## Theta*

### Description
Variant of A* with line-of-sight optimisation

---

### Pros

- Produces smoother paths than A*
- Fewer waypoints

---

### Cons

- More complex
- Slightly harder to debug
- Gains are modest in small grids

---

### Verdict

**Nice upgrade later**

---

## D* / D* Lite

### Description
Incremental replanning algorithms

---

### Pros

- Efficient for changing environments
- Reuses previous computation

---

### Cons

- Complex
- Not needed for mostly static arena

---

### Verdict

**Unnecessary complexity**

---

## RRT (Rapidly-exploring Random Tree)

### Description
Sampling-based planner

---

### Pros

- Handles complex spaces
- Works in continuous domains

---

### Cons

- Non-deterministic
- Not optimal
- Requires tuning
- Overkill for small grid world

---

### Verdict

**Not appropriate**

---

## RRT*

### Description
Optimal version of RRT

---

### Pros

- Asymptotically optimal

---

### Cons

- Slow
- Complex
- Still unnecessary here

---

### Verdict

**Overkill**

---

## Potential Fields

### Description
Attractive force to goal, repulsive from obstacles

---

### Pros

- Very simple
- Fast

---

### Cons

- Local minima problems
- Unreliable navigation
- Can oscillate

---

### Verdict

**Not robust enough**

---

## Hybrid A* (State Lattice)

### Description
A* over (x, y, θ) state space

---

### Pros

- Respects robot kinematics
- Produces driveable paths

---

### Cons

- Much more complex
- Larger search space
- Harder to tune

---

### Verdict

**Future upgrade only**

---

# Comparison Table

| Algorithm | Optimal | Speed | Complexity | Deterministic | Handles Kinematics | Suitable | Recommended |
|----------|--------|------|-----------|--------------|-------------------|----------|-------------|
| A* | Yes | Fast | Low | Yes | No (basic) | Yes | YES |
| Dijkstra | Yes | Slow | Low | Yes | No | Yes | No |
| Greedy BFS | No | Very fast | Low | Yes | No | Limited | No |
| BFS | Yes | Very slow | Low | Yes | No | No | No |
| Theta* | Yes | Fast | Med | Yes | No | Yes | Later |
| D* Lite | Yes | Fast | High | Yes | No | Overkill | No |
| RRT | No | Variable | Med | No | Yes | No | No |
| RRT* | Yes | Slow | High | No | Yes | No | No |
| Potential Fields | No | Very fast | Low | Yes | No | No | No |
| Hybrid A* | Yes | Med | High | Yes | Yes | Yes | Later |

---

# Why A* is the Right Choice

A* is selected because it best matches the problem:

## 1. Environment Size

- Small grid (~50–100 cells per side)
- A* runs extremely fast

---

## 2. Static Geometry

- Arena is known in advance
- Obstacles do not move significantly

A* excels in static maps.

---

## 3. Determinism

- Predictable behaviour
- Important for debugging and competition

---

## 4. Simplicity

- Easy to implement correctly
- Easy to test and visualise

---

## 5. Integration

Works cleanly with:

- occupancy grids
- obstacle inflation
- waypoint-based execution

---

## 6. Performance on Pi 4B

- Low CPU usage
- Minimal memory footprint
- Real-time capable

---

## 7. Modularity

Fits cleanly into architecture:

- Strategy chooses goal
- A* computes path
- Controller follows path

---

# Limitations of A*

A* does **not**:

- consider robot orientation
- enforce turning constraints
- produce smooth trajectories

These are handled by:

- obstacle inflation
- path simplification
- controller layer

---

# Design Decision

We use:

> **Grid-based A* with obstacle inflation and path simplification**

And explicitly separate:

- planning (A*)
- execution (controllers)

---

# Implementation Strategy

## Phase 1

- Basic A* on occupancy grid
- Static obstacles only

---

## Phase 2

- Obstacle inflation
- Path simplification

---

## Phase 3

- Integration with Navigator
- Waypoint execution

---

## Phase 4

- Dynamic obstacles (optional)

---

## Phase 5 (optional upgrades)

- Theta* (smoother paths)
- Hybrid A* (orientation-aware)

---

# Package Structure

```text
navigation/
    planners/
        astar/
            planner.py
            grid.py
            heuristic.py
            simplify.py