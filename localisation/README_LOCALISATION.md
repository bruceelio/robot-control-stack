localisation/README_LOCALISATION.md

# Localisation System

This document describes the localisation architecture used within:

```
localisation/
```

It defines how the robot estimates its pose using multiple localisation methods, how these methods are organised, and how the robot selects the best available estimate at runtime.

---

# Purpose of This Module

This localisation system is not designed around a single “best” method.

Instead, it is designed to:

* explore multiple localisation approaches
* compare their behaviour under real conditions
* support robots with different hardware capabilities
* allow incremental upgrades over time
* enable learning and experimentation

The goal is not only to achieve accurate localisation, but to:

> **understand the strengths, weaknesses, and tradeoffs of different methods**

As such:

* simpler methods are not considered inferior
* different robots may intentionally use different methods
* multiple methods may run simultaneously for comparison

---

# Architecture Context

The system is structured as:

* Strategy layer → decides *what to do*
* Planner layer → decides *where to go*
* Controller layer → decides *how to follow the path*
* Motion layer → executes commands
* **Localisation layer → determines *where the robot is***

Localisation provides the robot pose estimate:

```
(x, y, θ)
```

---

# Design Philosophy

## 1. Multiple Methods

There is no single perfect localisation method.

Instead, the system supports multiple independent methods ranging from very simple estimates to advanced sensor-driven approaches:

* time-based open-loop estimation
* velocity integration
* drive-wheel encoder odometry
* encoder + IMU hybrid
* deadwheel odometry
* deadwheel + IMU
* IMU-only estimation
* OTOS / optical tracking
* vision-based localisation
* fused approaches

Each method produces its own estimate of robot pose or motion.

---

## 2. Capability Depends on Hardware

Not all robots have the same localisation hardware.

Examples:

* simulation → time or velocity only
* basic robot → drive encoders
* competition robot → minimal sensors + vision
* advanced robot → deadwheels + IMU
* alternative robot → OTOS
* vision-enabled robot → cameras

---

## 3. Observation-Based Interface

All localisation methods output a common structure:

```
PoseObservation
```

This includes:

* position (optional)
* heading (optional)
* velocity (optional)
* timestamp
* confidence
* validity / health flags
* diagnostic data

---

## 4. Arbitration, Not Hard Switching

The system does not rely on fixed primary / secondary switching.

Instead:

> The best available observation is selected at runtime based on quality

---

# Observability Levels

Not all methods provide the same information:

| Method     | Position | Heading |
| ---------- | -------- | ------- |
| Open loop  | ✔        | ✔       |
| Velocity   | ✔        | ✔       |
| Drivewheel | ✔        | ✔       |
| IMU only   | ✖        | ✔       |
| Vision     | ✔        | partial |
| OTOS       | ✔        | ✔       |

This is important when comparing methods and designing arbitration.

---

# Folder Structure

```text
localisation/
    localisation.py
    arbitration.py
    pose_types.py

    methods/
        openloop/
        velocity/
        drivewheel/
        deadwheel/
        twodeadwheelimu/
        imuonly/
        otos/
        vision/
        fused/
```

---

# Capability Ladder

```
Level 0   → Time-based (open loop)
Level 0.5 → Velocity integration

Level 1   → Drive encoder odometry
Level 1.5 → Gyro-only (heading only)

Level 2   → Encoder + IMU hybrid
Level 2.5 → Single deadwheel + IMU

Level 3   → Deadwheel odometry
Level 3.5 → Two deadwheel + IMU

Level 4   → OTOS / optical tracking
Level 4.5 → Vision / absolute reference

Level 5   → Sensor fusion
```

---

# Method Families

---

## Open Loop

```
methods/openloop/
```

* no feedback
* useful in simulation
* very high drift

---

## Velocity Integration

```
methods/velocity/
```

* uses commanded velocity × time
* improved open-loop

---

## Drive-Wheel Odometry

```
methods/drivewheel/
```

* encoder-based
* sensitive to slip

---

## Encoder + IMU Hybrid

* encoders → distance
* IMU → heading

---

## Deadwheel Odometry

```
methods/deadwheel/
```

* independent of drive slip
* improved repeatability

---

## Two Dead Wheel + IMU

```
methods/twodeadwheelimu/
```

* strong continuous estimation
* low latency

---

## IMU Only

```
methods/imuonly/
```

* heading only
* fallback method

---

## OTOS

```
methods/otos/
```

* integrated motion sensor
* simple hardware integration
* surface dependent

---

## Vision

```
methods/vision/
```

Uses cameras and known references.

### Subtypes

### Fixed Camera

* simple geometry
* limited field of view

### Pan/Tilt Camera (Active Vision)

* camera can rotate
* wider search capability
* requires joint angle tracking
* more complex coordinate transforms

### Characteristics

* can provide absolute correction
* dependent on visibility
* useful as recovery mechanism

---

## Active Vision Tracking (Pan/Tilt)

A secondary camera may be mounted on a pan/tilt mechanism and oriented toward the rear of the robot.

### Motivation

In many competition environments:

- the forward direction is cluttered with objects and robots
- the arena perimeter is clearer and contains localisation markers

This creates an asymmetric visibility environment.

### Approach

- the primary forward camera is used for object detection and navigation
- a secondary pan/tilt camera continuously tracks arena markers
- the camera prioritises maintaining visibility of a stable reference marker

### Benefits

- continuous localisation updates instead of opportunistic detection
- reduced occlusion compared to forward-facing vision
- improved stability of pose estimate
- persistent knowledge of "home" or reference location

### Characteristics

- acts as a correction source rather than primary odometry
- dependent on marker availability
- requires calibration of camera pose relative to robot
- introduces additional mechanical complexity

### Notes

This approach is particularly effective in environments where:

- markers are placed around the perimeter
- central areas are highly occluded

It allows the robot to:

> maintain a continuous reference to the arena while operating in cluttered space


## Fused Methods

```
methods/fused/
```

* combines multiple sources
* reduces drift
* highest complexity

---

# Comparison Mode

The system may run multiple localisation methods simultaneously without selecting a single primary source.

Used for:

* method comparison
* drift analysis
* confidence tuning

---

# Pose Observation Model

```
PoseObservation
```

```text
source: str
timestamp: float

position: (x, y) | None
heading: θ | None
velocity: (vx, vy, ω) | None

confidence: float
valid: bool

diagnostics: dict
```

---

# Arbitration System

## Responsibilities

* collect observations
* reject invalid/stale data
* score observations
* select best estimate

---

# Example Robot Configurations

## SR Robot (Competition-Oriented)

* drivewheel / velocity baseline
* optional IMU
* vision (fixed or pan/tilt) for correction

---

## Decode Robot (Instrumentation Platform)

* OTOS
* optional vision
* method comparison

---

# Runtime Behaviour

1. detect available methods
2. collect observations
3. filter invalid data
4. score observations
5. select best estimate
6. update pose

---

# Configuration Example

```yaml
localisation:
  preferred_order:
    - twodeadwheelimu
    - otos
    - vision
    - drivewheel
    - velocity
    - openloop
    - imuonly

  methods:
    openloop:
      enabled: true
      base_weight: 0.1

    velocity:
      enabled: true
      base_weight: 0.2

    drivewheel:
      enabled: true
      base_weight: 0.4

    twodeadwheelimu:
      enabled: true
      base_weight: 1.0

    imuonly:
      enabled: true
      base_weight: 0.3

    otos:
      enabled: true
      base_weight: 0.9

    vision:
      enabled: true
      base_weight: 0.8
```

---

# Design Summary

The localisation system is based on:

> **Multiple localisation methods organised in a capability ladder with runtime arbitration**

The goal is not only localisation accuracy, but:

> **understanding localisation through experimentation**

This ensures:

* adaptability across robots
* support for simple and advanced systems
* continuous improvement over time
* meaningful comparison between methods

---

# Future Work

* improved confidence modelling
* smoother transitions between methods
* expanded vision capabilities
* sensor fusion development
* long-term drift analysis tools

---

# Final Note

This system is intentionally flexible.

It is designed to support:

* competition robots
* experimental platforms
* simulation environments

and to evolve as understanding improves.
