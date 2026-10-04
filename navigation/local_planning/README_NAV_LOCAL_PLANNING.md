# Navigation Local Planning

## Purpose

`navigation/local_planning/` contains localisation-independent local navigation algorithms.

These algorithms operate on **local perception and robot-relative state**. They do not require an arena pose, global map, SLAM, or Stage 3 localisation.

The package is intended to support reusable Stage 2 navigation for SR, UniBots, and other robots, while remaining usable underneath Stage 3 navigation when localisation is available.



## Core Architectural Rule

Local planning algorithms must consume **sensor-independent perception outputs**, not raw hardware interfaces.

For example, local planning should not know whether an obstacle distance came from:

- a VL53L7CX;
- a VL53L5CX;
- an ultrasonic sensor;
- a camera obstacle detector;
- a depth camera;
- a future LiDAR.

Sensor interpretation, filtering, classification, fusion, and confidence handling belong in `perception/`.

Local planning consumes robot-frame abstractions such as:

- local obstacle/free-space observations;
- preferred bearing or local goal direction;
- tracked dynamic obstacles;
- current robot motion;
- robot footprint and dynamic limits.

The same rule applies to wall following: navigation should request relevant wall geometry or distances from perception rather than directly interrogating a particular sensor.

---

## Local Planning Pipeline

A representative Stage 2 pipeline is:

```text
target / mission intent
        |
        v
preferred local bearing
        |
        v
PERCEPTION
  - free-space / obstacle field
  - tracked dynamic obstacles
  - wall geometry
  - target state
        |
        v
FOLLOW-THE-GAP
choose a traversable corridor
biased toward the preferred direction
        |
        v
DYNAMIC WINDOW APPROACH
evaluate dynamically achievable (v, omega)
with stopping / clearance constraints
        |
        v
VELOCITY-OBSTACLE CONSTRAINTS
reject or penalise motions predicted
to collide with moving obstacles
        |
        v
local velocity request
        |
        v
Velocity Arbiter
        |
        v
Velocity Backend
```

This is a representative composition, not a mandatory serial architecture.

Valid combinations include:

- FTG alone;
- FTG + DWA;
- DWA without FTG;
- DWA + VO;
- FTG + DWA + VO;
- wall following + VO;
- other local planners using the same perception abstractions.

---

## `local_planning_coordinator.py`

The local-planning coordinator composes enabled local-planning capabilities.

It is deliberately **not** called an arbiter.

In the navigation stack, an arbiter normally selects between competing command sources. The local-planning coordinator instead passes information and constraints between compatible algorithms and combines their roles.

It is also not a perception fusion layer. Sensor fusion belongs in `perception/`.

The coordinator should remain thin. Algorithm-specific mathematics belongs in the individual algorithm modules.

### Representative Inputs

```text
preferred_direction
local_obstacle_field
tracked_dynamic_obstacles
current_motion
robot_constraints
```

### Representative Output

```text
local velocity request
status / reason
```

### Example Composition

```text
FTG
  -> selects a preferred free-space direction

DWA
  -> evaluates achievable local velocities toward that direction

VO
  -> constrains or rejects candidate velocities that create
     predicted collisions with moving obstacles

Coordinator
  -> manages the enabled composition and returns the final
     local velocity request
```

---

## Follow-the-Gap

File:

```text
follow_the_gap.py
```

### Purpose

Follow-the-Gap selects a traversable local corridor from an ordered obstacle/free-space representation.

It answers:

> Which locally visible direction provides a safe route through the current free space?

### Inputs

Typical inputs include:

- ordered robot-frame obstacle ranges or free-space sectors;
- obstacle inflation / robot footprint;
- preferred bearing;
- minimum usable gap width;
- clearance constraints.

### Outputs

Typical outputs include:

- selected gap;
- selected steering direction / local heading;
- clearance information;
- status.

### Target-Biased FTG

For object acquisition, FTG should normally be biased toward a **target bearing** rather than simply choosing the geometrically largest gap.

A useful conceptual score is:

```text
gap quality
+ clearance
+ traversability
+ alignment with preferred / target bearing
```

Target visibility should not be a hard requirement.

In clutter, the safest route through a gap may temporarily occlude the target or move it outside the camera FoV.

The target state can instead be propagated for short periods using robot-relative motion estimation and corrected when vision reacquires it.

This remains Stage 2 behaviour and does not require arena localisation.

### Return-to-Base Use

FTG is also useful when returning to base.

The preferred direction may come from:

- a return-guide bearing;
- an estimated return direction;
- wall-following intent;
- another local navigation objective.

The robot can continue safely through clutter and later re-establish localisation or guide-tag vision.

---

## Dynamic Window Approach

File:

```text
dynamic_window.py
```

### Purpose

DWA selects a safe and dynamically achievable `(v, omega)` command.

It answers:

> Given the robot's current motion and physical limits, how should it drive through the selected local free space?

### Inputs

Typical inputs include:

- current linear velocity;
- current angular velocity;
- linear acceleration / deceleration limits;
- angular acceleration limits;
- robot footprint;
- obstacle field;
- desired local direction or motion;
- stopping-distance constraints.

### Outputs

Typical outputs include:

- selected linear velocity `v`;
- selected angular velocity `omega`;
- candidate score / safety information;
- status.

### Relationship to FTG

FTG and DWA solve different problems:

```text
FTG:
    choose the corridor / direction

DWA:
    choose how to drive through it
```

FTG may therefore be useful without DWA, while DWA can add:

- velocity selection;
- curvature selection;
- braking awareness;
- acceleration constraints;
- clearance-aware speed reduction.

The word *dynamic* in DWA primarily refers to the robot's own dynamic constraints. Classical DWA does not by itself provide full prediction of independently moving obstacles.

---

## Velocity Obstacles

File:

```text
velocity_obstacle.py
```

### Purpose

Velocity Obstacles constrain robot velocities that are predicted to cause a future collision with a moving obstacle.

They answer:

> Which candidate velocities would put the robot on a collision course with an observed moving object?

### Inputs

Typical inputs include:

- tracked obstacle relative position;
- tracked obstacle relative velocity;
- candidate robot velocity;
- robot footprint or collision radius;
- obstacle footprint or collision radius;
- prediction horizon.

### Outputs

Typical outputs include:

- forbidden velocity region;
- safe / unsafe candidate classification;
- collision prediction information.

### Relationship to DWA

VO complements rather than replaces DWA.

A useful conceptual combination is:

```text
DWA
    generates dynamically achievable candidate velocities

VO
    removes or penalises candidates that create predicted
    collisions with moving obstacles

remaining candidate
    becomes the local velocity request
```

TTC and moving-object tracking are supporting perception / prediction primitives. They are not necessarily separate navigation planners.

Future variants may include:

- VO;
- RVO;
- ORCA.

---

## Other Local Planning Algorithms

The package also retains other academically established local-planning methods.

### Artificial Potential Fields

```text
artificial_potential_field.py
```

Uses attractive and repulsive fields to generate local navigation behaviour.

### Bug2

```text
bug2.py
```

A classical obstacle-avoidance method based on direct progress toward a goal with boundary following when blocked.

### Tangent Bug

```text
tangent_bug.py
```

A Bug-family method using local range information and tangent geometry.

### Vector Field Histogram

```text
vector_field_histogram.py
```

Constructs an obstacle-density representation and selects locally traversable directions.

These methods remain useful for comparison, experimentation, specialised behaviours, and future robots.

---

## Relationship to Perception

The local-planning package does not classify the environment.

Perception is responsible for producing the abstractions used here.

Examples include:

```text
local_obstacle_field
free_space_sectors
tracked_dynamic_obstacles
target_bearing
target_range
left_wall_distance
right_wall_distance
wall_heading
front_clearance
```

Perception may use multiple sensor sources to produce them.

For example:

```text
ToF + camera
    -> perception
    -> obstacle / free-space representation
    -> FTG / DWA

camera tracking + range observations
    -> perception
    -> tracked moving obstacle
    -> VO

ToF + camera geometry
    -> perception
    -> wall distance / wall heading
    -> wall_following/
```

This keeps navigation independent of sensor choice.

---

## Relationship to Visual Servoing

`visual_servoing/` and `local_planning/` solve different problems.

Visual servoing is appropriate when a usable direct target observation is available and precision relative motion is required.

Local planning is appropriate when the robot must continue to navigate through locally perceived free space, including when:

- a target is temporarily occluded;
- another object lies between the robot and target;
- the desired route must bend around clutter;
- global localisation is unavailable.

PBVS, Smooth Control Law, and other visual-servoing methods therefore remain useful tools, but they are not required for Stage 2 local navigation.

For object acquisition, a robust architecture may be:

```text
select target
    ->
camera target bearing / tracking
    ->
target-biased FTG
    ->
DWA velocity selection
    ->
optional VO dynamic-obstacle constraints
    ->
close-range visual alignment / centring
    ->
mechanically tolerant pickup
```

---

## Relationship to Wall Following

`wall_following/` remains a separate navigation capability.

It should consume perception-derived wall information rather than raw sensor readings.

Examples:

```text
wall distance
wall heading
front clearance
wall confidence
```

Local planning and wall following may be coordinated by higher-level navigation according to mission state.

Velocity Obstacles may also be applied when wall following if moving-obstacle avoidance is required.

---

## Relationship to Stage 3

Stage 3 global planning can provide a preferred local direction, waypoint, path segment, or trajectory reference to Stage 2 local planning.

The local planner may then protect execution from unexpected local obstacles.

Conceptually:

```text
Stage 3 global path / trajectory
        |
        v
local desired direction / motion
        |
        v
Stage 2 local planning
        |
        v
safe local velocity
```

If Stage 3 localisation becomes stale or unavailable, Stage 2 should be capable of continuing local collision avoidance and free-space navigation independently.

---

## Directory

Files are kept alphabetically within the package.

```text
local_planning/
├── __init__.py
├── artificial_potential_field.py
├── bug2.py
├── dynamic_window.py
├── follow_the_gap.py
├── local_planning_coordinator.py
├── models.py
├── README_NAV_LOCAL_PLANNING.md
├── tangent_bug.py
├── vector_field_histogram.py
└── velocity_obstacle.py
```

---

## Design Rules

1. **No localisation dependency.**  
   Local-planning algorithms must be usable entirely in the robot frame.

2. **No raw-sensor dependency.**  
   Algorithms consume perception outputs, not device-specific sensor APIs.

3. **No perception fusion in navigation.**  
   Sensor fusion, object classification, wall identification, and observation confidence belong in `perception/`.

4. **Algorithms remain individually reusable.**  
   FTG, DWA, VO, VFH, and other planners must not require the coordinator in order to be used independently.

5. **The coordinator coordinates; it does not contain algorithms.**  
   Mathematical implementation remains in each algorithm module.

6. **Stage 2 is a capability threshold, not package ownership.**  
   Stage 2 makes local-planning methods possible because reliable local perception exists. The same methods remain available at Stage 3.

7. **Temporary target loss is expected.**  
   Local navigation must not assume that a selected target remains continuously visible while traversing clutter.

8. **Global localisation loss must not disable obstacle avoidance.**  
   Stage 2 remains operational independently of Stage 3 pose quality.

9. **Robot-specific tuning stays outside core algorithm logic where practical.**  
   Footprint, dynamics, speed limits, sensor coverage, and competition-specific parameters should be supplied through configuration/models.

10. **Outputs should integrate with the existing velocity-command path.**  
    Local planning should ultimately produce a velocity request compatible with `Velocity Arbiter` and the normal velocity backend.

---

## Current Priority

The current implementation priority is:

```text
1. Follow-the-Gap
2. Dynamic Window Approach
3. Velocity Obstacles / dynamic-obstacle constraints
4. Local-planning coordinator
```

Initial development should use synthetic/perception-agnostic inputs so the algorithms remain reusable across robots and sensor configurations.

The first intended real-world Stage 2 perception source is expected to combine normal camera perception with a compact multi-zone ToF front sensor array, while preserving the same local-planning interfaces for future sensing systems.
