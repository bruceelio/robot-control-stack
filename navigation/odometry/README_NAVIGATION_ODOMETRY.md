navigation/odometry/README_NAVIGATION_ODOMETRY.md

# navigation/odometry — Robot-Relative Odometry

This package measures **robot-relative movement**.

It does **not** own the robot's global arena pose.

Conceptually:

```text
navigation/odometry
    measures movement

        ↓

localisation/providers/odometry
    propagates global pose
```

## Generic Interface

Odometry sources implement the common interface from:

```text
navigation/odometry/base.py
```

The required contract is:

```python
reset()
read() -> OdometryDelta
```

`reset()` establishes the current physical position as the zero reference.

`read()` returns cumulative movement since the most recent `reset()`.

## OdometryDelta

The canonical robot-relative output is:

```text
forward_mm
lateral_mm
heading_rad
covariance
```

Sign convention:

```text
+forward_mm = robot forward
+lateral_mm = robot left
+heading_rad = left / counter-clockwise
```

`heading_rad` may be `None` for a source that cannot determine heading.

## Current Implementations

```text
drive_encoders.py
    DriveEncoderOdometry
    Active drive-encoder odometry.

three_deadwheel.py
    ThreeDeadwheelKinematics
    ThreeDeadwheelOdometry
    Generic three-tracking-wheel support.
    Proven synthetically but not currently active on the robot.
```

Future sources may include:

```text
two deadwheels + IMU
other tracking-wheel layouts
```

They should expose the same `reset()` / `read()` interface.

## Separation from Localisation

An odometry source reports movement only:

```text
OdometrySource
      ↓
OdometryDelta
```

The localisation layer is responsible for applying that movement to an accepted
global pose:

```text
OdometrySource
      ↓
OdometryPoseProvider
      ↓
OdometryArbiter
      ↓
global pose estimate
```

Robot-specific pose arbitration and absolute-vs-propagated decisions do not belong
in this package.

## Independent Instances

Consumers which reset odometry for different purposes must use independent source
instances.

For example, the motion backend may reset its odometry baseline for a movement
primitive while localisation needs to continue tracking from its own baseline.

Therefore:

```text
motion control odometry instance
        !=
localisation odometry instance
```

Do not share one mutable odometry object between those consumers.

## Covariance

`OdometryDelta.covariance`, when supplied, describes uncertainty in robot-relative:

```text
[forward_mm, lateral_mm, heading_rad]
```

It is **not** the same as global pose covariance:

```text
[x, y, heading]
```

Robot-relative covariance must not simply be copied into a global pose observation.
A proper world-frame transformation / propagation model is required.

`None` means covariance is not currently supplied.

## Design Rule

Keep this package focused on:

> **measuring robot-relative movement from sensors**

Global pose ownership, reseeding, provider arbitration, Vision corrections and future
sensor fusion belong in `localisation/`.
