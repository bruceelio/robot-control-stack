localisation/README_LOCALISATION.md

# Localisation System

This document describes the localisation architecture used within:

```
localisation/
```

It defines how the robot estimates its pose using multiple localisation methods, and how these methods are organised, evaluated, and selected at runtime.

---

# Architecture Context

The system is structured as:

* Strategy layer → decides *what to do*
* Planner layer → decides *where to go*
* Controller layer → decides *how to follow the path*
* Motion layer → executes commands
* **Localisation layer → determines *where the robot is***

Localisation provides the robot’s **pose estimate**:

```
(x, y, θ)
```

This pose is consumed by:

* navigation planners
* controllers
* strategy logic

---

# Design Philosophy

Localisation is designed around three principles:

## 1. Multiple Methods

There is no single perfect localisation method.

Instead, the system supports **multiple independent methods**, such as:

* vision-based localisation
* odometry (dead wheels + IMU)
* IMU-only estimation
* fused approaches

Each method produces its own estimate of the robot pose.

---

## 2. Observation-Based Interface

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

This abstraction allows all methods to be treated uniformly.

---

## 3. Arbitration, Not Hard Switching

The system does **not** rely on fixed primary/secondary switching.

Instead:

> The best available observation is selected at runtime based on quality.

Each method is:

* evaluated for validity
* scored based on confidence and freshness
* compared against other available observations

This allows the system to adapt dynamically to changing conditions.

---

# Folder Structure

```text
localisation/
    localisation.py          # owns current pose state
    arbitration.py           # selection and scoring logic
    pose_types.py            # Pose and PoseObservation definitions

    methods/
        vision/
        twodeadwheelimu/
        imuonly/
        fused/
```

---

# Method Families

Localisation methods are grouped into **families**.

Each family may contain multiple implementations.

---

## Vision

```text
methods/vision/
```

Uses cameras and known field features.

Typical implementations:

* single camera + 2 markers (position only)
* multi-marker triangulation
* multi-camera fusion

### Characteristics

* Provides absolute position
* May not always provide heading
* Sensitive to occlusion and lighting
* High accuracy when valid

---

## Two Dead Wheel + IMU

```text
methods/twodeadwheelimu/
```

Uses:

* forward dead wheel encoder
* lateral dead wheel encoder
* IMU (heading)

### Characteristics

* Continuous estimation
* Independent of vision
* Low latency
* Drift accumulates over time

This is expected to become the **primary localisation method**.

---

## IMU Only

```text
methods/imuonly/
```

Uses IMU data only.

### Characteristics

* Provides heading only (θ)
* No position estimate
* Always available (fallback)
* Subject to drift

---

## Fused Methods (Future)

```text
methods/fused/
```

Combines multiple sources:

* vision + odometry
* IMU + odometry (EKF)
* full sensor fusion

### Characteristics

* Reduced drift
* Increased robustness
* Higher complexity

---

# Pose Observation Model

All methods must produce:

```
PoseObservation
```

Recommended structure:

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

The arbitration layer is responsible for selecting the best pose estimate.

---

## Responsibilities

* collect observations from all enabled methods
* reject invalid or stale data
* score observations
* select best candidate
* optionally fuse results (future)

---

## Selection Criteria

Observations are evaluated based on:

* validity (must be true)
* completeness (position + heading preferred)
* confidence score
* timestamp freshness
* stability (no large jumps)
* method health

---

## Example Policy

Preferred methods (in order):

1. TwoDeadWheelIMU
2. Vision
3. IMUOnly

However:

> Preference is advisory — not absolute.

A lower-priority method may be selected if:

* higher-priority method is unavailable
* data is stale or invalid
* confidence is significantly higher

---

# Runtime Behaviour

At each update cycle:

1. Each method produces a `PoseObservation`
2. Observations are filtered (valid + fresh)
3. Observations are scored
4. Best observation is selected
5. Pose state is updated

---

# Configuration

Example configuration:

```yaml
localisation:
  preferred_order:
    - twodeadwheelimu
    - vision
    - imuonly

  methods:
    twodeadwheelimu:
      enabled: true
      base_weight: 1.0

    vision:
      enabled: true
      base_weight: 0.8

    imuonly:
      enabled: true
      base_weight: 0.4
```

---

# Why This Architecture

## 1. Robustness

Multiple independent methods reduce single points of failure.

---

## 2. Flexibility

New localisation methods can be added without modifying core logic.

---

## 3. Runtime Adaptation

System automatically selects the best available estimate.

---

## 4. Clear Separation

* methods generate observations
* arbitration selects estimates
* localisation maintains state

---

## 5. Future Compatibility

Supports:

* sensor fusion (EKF)
* SLAM integration
* additional sensors (LiDAR, vision)

---

# Limitations

Current system:

* does not yet perform full sensor fusion
* relies on best-method selection rather than probabilistic merging
* may exhibit jumps when switching sources

---

# Future Work

## Phase 1

* Implement TwoDeadWheelIMU method
* Improve observation confidence scoring

---

## Phase 2

* Add smoothing / transition handling
* Introduce velocity estimation

---

## Phase 3

* Implement EKF-based fusion
* Combine odometry + IMU + vision

---

## Phase 4

* Add global localisation (SLAM or map-based correction)

---

# Design Summary

The localisation system is based on:

> **Multiple independent localisation methods + runtime arbitration**

Rather than committing to a single approach, the system:

* gathers multiple pose estimates
* evaluates their quality
* selects the best available estimate

This ensures robust and adaptable localisation across varying conditions.
