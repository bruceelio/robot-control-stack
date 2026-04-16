localisation/README_LOCALISATION.md

# Localisation System

This document describes the localisation architecture used within:

localisation/

It defines how the robot estimates its pose using multiple localisation providers, how those providers are organised, and how the robot selects the best available estimate at runtime.

---

# Purpose of This Module

This localisation system is not designed around a single “best” method.

Instead, it is designed to:

- support multiple localisation approaches
- compare their behaviour under real conditions
- support robots with different hardware capabilities
- allow incremental upgrades over time
- enable learning and experimentation

The goal is not only to achieve accurate localisation, but to:

> understand the strengths, weaknesses, and tradeoffs of different methods

As such:

- simpler methods are not considered inferior
- different robots may intentionally use different providers
- multiple providers may run simultaneously for comparison
- the same architecture applies in both simulation and on the real robot

---

# Architecture Context

The system is structured as:

- Strategy layer → decides what to do
- Planner layer → decides where to go
- Controller layer → decides how to follow the path
- Motion layer → executes commands
- Localisation layer → determines where the robot is

Localisation provides the robot pose estimate:

(x, y, θ)

---

# Design Philosophy

## 1. Localisation is Estimation + Arbitration

Localisation should not be a single monolithic pose function.

Instead it:

- collects pose observations from one or more providers
- compares them
- accepts the best currently available estimate
- maintains pose state with validity and freshness

---

## 2. Providers Are Organised by Evidence Source

Providers are grouped by how pose evidence is obtained:

- startup pose
- vision fixes
- motion-based propagation
- odometry-based propagation
- inertial heading
- fused approaches

There is no separate “simulation localisation”.

The same providers apply everywhere, but may produce weaker observations depending on conditions.

---

## 3. Capability Depends on Hardware and Runtime Conditions

Providers may exist but produce low-quality data depending on environment.

Examples:

- no encoders → odometry is weak or invalid
- occluded vision → vision is weak
- motion-only → drift increases over time

Arbitration must handle this.

---

## 4. Observation-Based Interface

All providers output:

PoseObservation

Containing:

- position (optional)
- heading (optional)
- timestamp
- confidence
- validity
- source
- diagnostics

---

## 5. Arbitration, Not Hard Switching

The system selects the best observation at runtime based on quality.

Lower-tier providers may win if they are fresher or more reliable.

---

# Observability Levels

| Provider type   | Position | Heading |
|-----------------|----------|---------|
| Startup         | ✔        | ✔       |
| Motion          | ✔        | ✔       |
| Odometry        | ✔        | ✔       |
| IMU only        | ✖        | ✔       |
| Vision          | ✔        | partial |
| Optical         | ✔        | ✔       |

---

# Folder Structure

localisation/
    localisation.py
    arbitration.py
    pose_types.py

    providers/
        base.py

        startup/
            startup_config.py

        vision/
            cam1_markers2.py
            cam1_markers3plus.py

        motion/
            commanded_motion.py

        odometry/
            wheel_odometry.py

        inertial/
            imu_heading.py

        fused/

---

# Provider Families

## Startup

- provides initial pose
- deterministic
- becomes stale after movement

---

## Vision

- uses arena markers
- provides absolute correction
- depends on visibility

---

## Motion-Based

- based on commanded movement
- works without hardware sensors
- drifts over time
- valid on real robots as fallback

---

## Odometry

- based on measured movement
- depends on hardware quality
- may degrade with slip or poor data

---

## Inertial

- heading-only providers
- supports other methods

---

## Optical

- hardware-specific tracking
- may provide continuous motion

---

## Fused

- combines multiple providers
- reduces drift
- higher complexity

---

# PoseObservation Model

source: str  
timestamp: float  

position: (x, y) or None  
heading: θ or None  

confidence: float  
valid: bool  

diagnostics: dict  

---

# Arbitration System

Responsibilities:

- collect observations
- reject invalid/stale data
- score observations
- select best estimate
- maintain pose

---

# Runtime Behaviour

1. collect observations
2. filter invalid
3. score observations
4. select best
5. update pose

---

# Example Configuration

```yaml
localisation:
  providers:
    startup_config:
      enabled: true
      base_weight: 0.4

    cam1_markers2:
      enabled: true
      base_weight: 0.8

    commanded_motion:
      enabled: true
      base_weight: 0.3

    wheel_odometry:
      enabled: true
      base_weight: 0.5
      
      
Design Summary

The system is:

multiple providers + runtime arbitration

This ensures:

flexibility
robustness
hardware independence
graceful fallback
Future Work
improved confidence modelling
better fusion
drift analysis tools
Final Note

This system is designed to work consistently across:

simulation
real robots
experimental platforms
