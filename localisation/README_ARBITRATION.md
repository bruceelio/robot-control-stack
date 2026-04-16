localisation/README_ARBITRATION.md

# Localisation Arbitration Design

---

## 1. Estimator + Arbitrator Model

Localisation should:

- gather observations from multiple providers
- compare them
- select the best
- maintain pose state

One system owns pose, many provide evidence.

---

## 2. Folder Structure

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

    motion/
      commanded_motion.py

    odometry/
      wheel_odometry.py

    inertial/
      imu_heading.py

---

## 3. Providers Represent Evidence Sources

Providers are not split by simulation vs real.

They represent:

- startup knowledge
- vision fixes
- motion propagation
- odometry
- inertial support

All are valid in both simulation and real-world contexts.

---

## 4. PoseObservation Interface

Each provider returns:

- position
- heading (optional)
- timestamp
- confidence
- validity
- source
- diagnostics

---

## 5. Scoring Instead of Priority

Do NOT use fixed priority.

Use scoring based on:

- base provider weight
- freshness
- presence of position
- presence of heading
- consistency with current pose
- diagnostics

---

## 6. Example Weighting

provider_weight:

- startup_config: 0.4
- cam1_markers2: 0.8
- commanded_motion: 0.3
- wheel_odometry: 0.5

Weights are only starting points.

---

## 7. Provider Inputs

Providers should use:

raw perception data

NOT direct hardware access.

Flow:

perception → detections → providers → observations → arbitration

---

## 8. Naming

Use clear capability-based names:

- startup_config
- cam1_markers2
- commanded_motion
- wheel_odometry

---

## 9. Validity Separation

Track separately:

- position_valid
- heading_valid

Never assume heading = 0 is valid.

---

## 10. Migration Plan

Minimal steps:

1. move current vision method into provider
2. return PoseObservation
3. centralise pose in localisation.py
4. add arbitration
5. add motion provider

---

## 11. Key Principle

Providers may exist but be low quality.

Arbitration must:

- prefer best evidence
- handle weak providers
- allow fallback naturally

---

## Final Summary

Localisation works by:

multiple providers → scored observations → best selected

This ensures:

- flexibility
- robustness
- consistency across environments