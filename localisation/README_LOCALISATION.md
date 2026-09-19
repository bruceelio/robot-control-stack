localisation/README_LOCALISATION.md

# localisation — Robot Pose Estimation

This folder owns the robot's estimate of:

```text
where am I?
```

Localisation maintains the robot pose:

```text
x
y
heading
```

and decides which available pose information should currently be trusted.

It does **not** own:

- camera capture;
- object detection;
- target selection;
- behaviour decisions;
- motor control.

---

# Architectural Boundary

Vision and Perception are separate systems.

The key rule is:

> **Perception consumes Vision. Localisation consumes Vision. Localisation does not consume Perception.**

Conceptually:

```text
                         VISION
                            |
              +-------------+-------------+
              |                           |
              v                           v
         PERCEPTION                  LOCALISATION
              |                           |
       objects / targets              robot pose
              |                           |
              +-------------+-------------+
                            |
                        BEHAVIOURS
```

The controller obtains the current Vision message and makes the same observation
available independently to:

```text
Perception
Localisation
```

Localisation therefore does not depend on object tracking or behaviour-specific
interpretation.

See:

```text
/vision/README.md
/perception/README.md
```

for the corresponding boundaries.

---

# Core Principle

There is one owner of robot pose:

```text
Localisation
```

Multiple sources may provide information which can establish, correct or propagate
that pose.

These sources are not all equivalent.

The most important distinction is between:

```text
absolute / corrective pose sources
```

and:

```text
propagation sources
```

---

# Absolute Pose vs Propagation

## Absolute / corrective sources

These can establish or correct the robot's position in the arena reference frame.

Current or expected examples include:

```text
Vision
startup pose
OTOS        # future, when arena-referenced
```

Vision derives pose from known arena landmarks.

The configured startup pose establishes the initial known pose before movement.

An appropriately configured OTOS may later provide an arena-referenced pose.

---

## Propagation sources

These do not independently establish where the robot is in the arena.

Instead they update an already known pose as the robot moves.

Current examples are:

```text
drive-encoder odometry
commanded motion
```

Future odometry sources may include:

```text
three deadwheels
two deadwheels + IMU
```

Conceptually:

```text
                 ABSOLUTE / CORRECTIVE
                ┌─────────────────────┐
                │ Vision              │
                │ Startup pose        │
                │ OTOS (future)       │
                └──────────┬──────────┘
                           |
                           v
                      CURRENT POSE
                           ^
                           |
                ┌──────────┴──────────┐
                │                     │
           Odometry family       Commanded motion
```

A propagated pose may be the best available estimate while absolute localisation is
unavailable, but it must not be treated as a new independent absolute measurement.

---

# Pose

The maintained robot pose is represented by:

```text
localisation/pose_types.py
```

Conceptually:

```python
Pose(
    x=...,
    y=...,
    heading=...,

    position_valid=...,
    heading_valid=...,

    source=...,
    timestamp=...,

    covariance=...,
)
```

Position and heading validity are deliberately independent.

A robot may know:

```text
x/y but not heading
```

or:

```text
heading but not x/y
```

depending on the available sensors.

`covariance` is optional. `None` means that the current pose does not have a usable
covariance estimate; it does **not** mean zero uncertainty.

---

# PoseObservation

Localisation providers do not directly overwrite the current pose.

They produce:

```text
PoseObservation
```

A `PoseObservation` is a provider's proposed pose measurement or propagated estimate.

The canonical information is:

```text
x
y
heading

position_valid
heading_valid

confidence
source
timestamp

is_absolute

covariance
diagnostics
```

Conceptually:

```python
PoseObservation(
    x=...,
    y=...,
    heading=...,

    position_valid=True,
    heading_valid=True,

    confidence=0.8,
    source="apriltag_pnp",
    timestamp=...,

    is_absolute=True,

    covariance=...,
    diagnostics={...},
)
```

There is deliberately no requirement that every observation contain a complete pose.

---

# Validity

Position and heading validity must be treated separately.

```text
position_valid
heading_valid
```

are statements about what the provider actually knows.

Do not assume:

```text
heading = 0
```

means:

```text
heading is valid
```

Likewise, an observation with a useful position but no trustworthy heading can still
contribute useful information.

A usable observation therefore needs at least one valid component:

```text
position_valid OR heading_valid
```

---

# Confidence

`confidence` is currently a numeric value from:

```text
0.0 to 1.0
```

It is used by the present arbitration logic when selecting between otherwise suitable
observations.

Confidence does **not** make a propagated observation equivalent to an absolute
measurement.

The final arbitration logic first distinguishes:

```text
absolute
vs
propagated
```

and only then compares suitable candidates within the selected class.

Confidence is therefore a current arbitration mechanism, not a substitute for a proper
uncertainty model.

---

# Covariance

All pose observations are capable of carrying covariance.

For global pose observations the ordering is:

```text
[x, y, heading]
```

with the current internal units:

```text
x, y        = millimetres
heading     = radians

var(x/y)    = mm²
var(heading)= rad²
```

`None` means covariance has not been supplied or cannot currently be propagated
correctly.

The rule is:

> **Do not invent covariance values simply to fill the field.**

In particular, propagated odometry cannot simply copy robot-relative odometry
covariance into world-frame pose covariance. Proper propagation requires the relevant
coordinate transform / Jacobian and process-noise model.

Current drive-encoder propagation therefore:

```text
reseed from accepted pose
    -> preserve the accepted covariance while no motion occurs

first actual propagated movement
    -> covariance becomes None
```

until a proper propagation model is implemented.

---

# Source and Diagnostics

Every observation carries a `source`.

The source is primarily for:

- diagnostics;
- logging;
- understanding which localisation family is active;
- comparing alternative providers.

Family arbiters may expose a family-level source while preserving the selected child
source in diagnostics.

Current examples:

```text
VisionArbiter output:
    source = selected vision method
    e.g. cam1_markers2 / apriltag_pnp

OdometryArbiter output:
    source = odometry
    diagnostics["provider_source"] = drive_encoders

CommandedMotionProvider:
    source = commanded_motion
```

The underlying source must not be lost even when a family-level name is exposed.

---

# Timestamp and Freshness

The timestamp belongs to the **measurement**.

For Vision this means the time the camera observation was made, not the time
localisation happened to process it.

Freshness is checked before accepting observations.

Camera/object-control freshness and localisation freshness are separate concepts.

For example:

```text
camera observation used for close object control
    may require a very fresh frame

localisation pose observation
    may remain useful slightly longer
```

These thresholds should not automatically be forced to the same value.

---

# Current Package Structure

The active localisation structure is:

```text
localisation/
    README_LOCALISATION.md
    __init__.py
    localisation.py
    arbitration.py
    pose_types.py

    fusion/
        __init__.py
        base.py

    providers/
        __init__.py
        base.py

        vision/
            __init__.py
            vision_arbiter.py
            pose_cam1_markers2.py
            pose_apriltag_pnp.py

        odometry/
            __init__.py
            base.py
            odometry_arbiter.py
            drive_encoders.py

        dead_reckoning/
            __init__.py
            commanded_motion.py
```

Robot-relative odometry measurement code is deliberately outside localisation:

```text
navigation/
    odometry/
        __init__.py
        base.py
        drive_encoders.py
        three_deadwheel.py
```

`navigation/odometry/three_deadwheel.py` currently proves the generic odometry
abstraction and three-wheel kinematics, but it is not part of the active default
localisation provider tree.

Providers are organised by **evidence source**, not by robot.

There should not be separate localisation algorithms merely because the program is
running on:

```text
Webots
rob_bot
bob_bot
Student Robotics hardware
```

Hardware differences belong below localisation and in resolved configuration.

---

# Current Runtime Provider Tree

The current default tree is:

```text
VisionArbiter -> vision
    Cam1Markers2Provider -> cam1_markers2
    AprilTagPnPPoseProvider -> apriltag_pnp

OdometryArbiter -> odometry
    DriveEncoderProvider -> drive_encoders

CommandedMotionProvider -> commanded_motion

        |
        v
    Arbitrator
        |
        v
   Localisation
```

The final estimator is currently:

```text
Arbitrator
```

but `Localisation` accesses it through the generic `PoseEstimator` boundary.

---

# Vision Localisation

Vision is an absolute/corrective localisation source.

The Vision layer supplies the current camera observation independently of Perception.

Conceptually:

```text
camera
   |
   v
VISION
   |
   v
AprilTag observations
   |
   v
vision localisation methods
   |
   v
PoseObservation
```

Known arena marker geometry is then used to infer robot pose.

---

# VisionArbiter

There may be more than one way to calculate pose from the same Vision observation.

Current methods include:

```text
markers2 geometry
AprilTag PnP
```

These are children of:

```text
VisionArbiter
```

Conceptually:

```text
                one Vision message
                      |
          +-----------+-----------+
          |                       |
          v                       v
      markers2                   PnP
          |                       |
          v                       v
    PoseObservation         PoseObservation
          \                       /
           \                     /
            +---- VisionArbiter -+
                      |
                      v
             selected vision result
```

The current `VisionArbiter` is primarily **method selection for one Vision message**.

It is not yet a complete multi-camera family arbiter.

A future multi-camera structure would conceptually be:

```text
front camera
    -> pose-method selection
    -> front camera pose

rear camera
    -> pose-method selection
    -> rear camera pose

front/rear camera poses
    -> Vision family arbitration
    -> one stable Vision result
```

That refactor is deliberately deferred until multiple camera pose sources are needed.

The current VisionArbiter preserves the selected observation's measurement fields,
including covariance.

If the current Vision observation contains no usable localisation information,
Vision should simply be unavailable for that cycle.

Old visual measurements should not be silently cached and presented as though they
were current observations.

---

# Generic Odometry Boundary

Robot-relative odometry implements the generic interface in:

```text
navigation/odometry/base.py
```

The minimal contract is:

```python
reset()
read() -> OdometryDelta
```

`OdometryDelta` represents cumulative robot-relative movement since the last reset:

```text
forward_mm
lateral_mm
heading_rad
covariance
```

Robot-relative sign convention:

```text
+forward_mm = robot forward
+lateral_mm = robot left
+heading    = counter-clockwise / left turn
```

Sensor-specific diagnostics may provide additional methods, but those are not part of
the generic `OdometrySource` contract.

For example, drive encoders retain a drive-specific `check_consistency()` diagnostic
without requiring every future odometry source to implement it.

---

# Generic Odometry Localisation Provider

The localisation-side common implementation is:

```text
localisation/providers/odometry/base.py
```

with:

```text
OdometryPoseProvider
```

It handles the behaviour common to relative odometry providers:

- reseeding from an accepted global pose;
- owning an independent odometry source;
- resetting the odometry baseline after reseed;
- reading cumulative robot-relative movement;
- applying only the increment since the previous update;
- transforming robot-relative movement into world coordinates;
- midpoint-heading integration;
- publishing a propagated `PoseObservation`;
- preserving covariance while stationary after reseed;
- dropping covariance after movement until proper propagation exists.

Concrete odometry providers therefore remain small sensor-specific adapters.

---

# Drive Encoder Odometry

Drive-encoder odometry is the active measured propagation source on the current robot.

The measurement layer is:

```text
navigation/odometry/drive_encoders.py
    DriveEncoderOdometry
```

It owns its own `EncoderManager`, applies configured encoder signs and geometry, and
returns robot-relative motion.

For differential drive:

```text
forward_mm = (left_mm + right_mm) / 2

heading_rad = (right_mm - left_mm) / track_width_mm

lateral_mm = 0
```

The localisation adapter is:

```text
localisation/providers/odometry/drive_encoders.py
    DriveEncoderProvider
```

`DriveEncoderProvider` is intentionally thin. It selects/configures
`DriveEncoderOdometry`; generic pose integration is inherited from
`OdometryPoseProvider`.

The localisation provider owns an **independent** odometry source.

It must not share the motion backend's odometry object because motion primitives reset
their own baseline independently.

---

# OdometryArbiter

Odometry sources are grouped behind:

```text
OdometryArbiter
```

Current:

```text
DriveEncoderProvider
        |
        v
OdometryArbiter
        |
        v
family odometry result
```

Future:

```text
DriveEncoderProvider ------\
ThreeDeadwheelProvider -----+--> OdometryArbiter
TwoDeadwheelImuProvider ----/
```

Usually a robot will have only one active odometry implementation.

The family arbiter still provides a stable architectural boundary so the rest of
localisation does not depend on which odometry hardware is installed.

The family rule is:

```text
0 valid sources -> no odometry observation
1 valid source  -> use it
>1 valid        -> keep the currently selected valid source
                   and switch only when necessary
```

The arbiter should not chatter between nearly equivalent sources because one confidence
value changes slightly.

The selected child information is preserved, including:

```text
measurement
timestamp
validity
confidence
covariance
underlying provider source
diagnostics
```

---

# Three-Deadwheel Support

Three-deadwheel kinematics have been implemented at the generic navigation-odometry
layer.

The tested layout is conceptually:

```text
left parallel wheel
right parallel wheel
perpendicular / lateral wheel
```

It produces:

```text
forward_mm
lateral_mm
heading_rad
```

including correction for rotation-induced movement at an offset perpendicular wheel.

This currently exists to prove the generic odometry abstraction.

It is **not** registered in the default localisation provider tree and should not gain
robot-specific configuration until a robot actually uses three deadwheels.

The same principle applies to future two-deadwheel + IMU odometry: implement it when
the hardware is actually needed, not merely to fill out the architecture.

---

# Final PoseEstimator Boundary

`Localisation` does not directly require a particular final estimation algorithm.

The boundary is:

```text
family providers / arbiters
          |
          v
     PoseEstimator
          |
          v
      Localisation
```

The default factory currently creates:

```text
Arbitrator
```

so present behaviour remains simple and understandable.

In future the default estimator can become:

```text
EKF
other fusion estimator
```

without changing the public responsibility of `Localisation`.

For compatibility with older callers, `Localisation.arbitrator` currently aliases the
active estimator.

---

# Current Final Arbitration

The current final estimator receives family-level and fallback observations.

Conceptually:

```text
VisionArbiter -----------\
                          \
OdometryArbiter -----------> Arbitrator
                            /
CommandedMotionProvider ---/
                          /
OTOS (future) -----------/
```

The current policy is intentionally simple:

1. reject unusable observations;
2. reject stale observations;
3. reject invalid confidence values;
4. separate absolute and propagated observations;
5. if any usable absolute observation is available, use the absolute class;
6. otherwise use propagation;
7. within the selected class, prefer the highest-confidence suitable observation;
8. use freshness as a secondary preference where necessary.

Therefore an available absolute Vision observation currently takes precedence over
odometry or commanded motion even if the propagated source reports a higher confidence.

Conceptually:

```text
absolute available?
       |
   +---+---+
   |       |
  YES      NO
   |       |
   v       v
absolute   propagated
candidates candidates
   |       |
   +---+---+
       |
       v
selected observation
       |
       v
current Pose
```

---

# Reseeding

Propagation sources need a known pose from which to continue.

When a new **absolute** pose is accepted:

```text
absolute correction
        |
        v
current Pose
        |
        v
reseed propagation providers
```

Current propagated sources include:

```text
OdometryArbiter
    -> DriveEncoderProvider

CommandedMotionProvider
```

For the drive-encoder provider, reseeding establishes a new global pose and causes its
own odometry baseline to be reset on the next observation.

This prevents accumulated propagation drift from surviving a good absolute correction.

---

## Propagation Must Not Reseed Itself

A propagated observation must be allowed to continue accumulating movement.

Therefore:

> **Propagated observations must not cause propagation providers to be reseeded from their own output.**

Correct behaviour:

```text
Vision / future arena-referenced OTOS
    |
    | absolute correction
    v
RESEED propagation

Odometry / Commanded Motion
    |
    | propagated estimate
    v
DO NOT reseed propagation
```

Explicit `set_pose()` operations may reseed all providers because they deliberately
establish a new known pose.

---

# Commanded Motion Provider

The fallback provider is:

```text
localisation/providers/dead_reckoning/commanded_motion.py
```

It provides temporary pose propagation based on the motion the robot was instructed to
perform.

It does not measure actual wheel movement.

It therefore provides:

```text
propagated estimate
```

rather than:

```text
absolute pose
```

The motion stack informs localisation when a timed drive or rotation begins.

For example:

```text
begin_commanded_drive(...)
begin_commanded_rotate(...)
```

The provider advances the estimated pose as the command progresses.

This remains useful as a fallback when measured odometry is unavailable.

Measured drive-encoder odometry is normally the better propagation source when valid.

---

# Commanded Motion Limitations

Commanded motion assumes commanded movement approximately matches physical movement.

Real robots introduce:

- wheel slip;
- battery effects;
- surface variation;
- unequal motors;
- collisions;
- mechanical tolerances.

Commanded motion is therefore expected to drift.

It is a fallback propagation source, not a replacement for measured localisation.

Very small commanded rotations may also be deliberately excluded from commanded-motion
propagation where experience shows the estimate is less trustworthy than ignoring the
small command.

---

# OTOS

OTOS is not currently an active provider.

It differs from ordinary wheel odometry because it may provide a direct pose estimate
when correctly referenced to the arena frame.

Where configured as an arena-referenced pose source it belongs on the
absolute/corrective side:

```text
Vision
OTOS
```

rather than being treated automatically as just another dead-reckoning source.

Implementation should wait until the actual hardware/reference behaviour is known.

---

# Startup Pose

At startup the robot is placed in a known competition start location.

The controller establishes this pose from:

```text
match zone
start slot
arena configuration
```

Conceptually:

```text
get_start_pose(...)
        |
        v
localisation.set_pose(...)
        |
        v
initial current Pose
        |
        v
reseed propagation providers
```

This provides the initial reference from which odometry and commanded-motion
propagation can begin.

---

# Coordinate Convention

Arena coordinates use a fixed world frame.

The established localisation convention is:

```text
heading 0°     = +X
heading +90°   = +Y
positive angle = counter-clockwise / left
```

Robot-relative odometry uses:

```text
+forward = robot forward
+lateral = robot left
+heading = counter-clockwise / left
```

The motion stack has also been physically checked so that:

```text
rotate(+angle)
```

means:

```text
left / counter-clockwise
```

and:

```text
rotate(-angle)
```

means:

```text
right / clockwise
```

Detector/control bearing conventions may differ; conversions must happen at the
appropriate boundary rather than changing the localisation coordinate convention.

---

# Perception Is Not a Localisation Provider

This boundary is deliberate.

Incorrect:

```text
camera
  |
  v
Perception
  |
  v
Localisation
```

Correct:

```text
                 Vision
                 /    \
                /      \
               v        v
       Perception    Localisation
```

Perception answers questions such as:

```text
What objects are visible?
Which target is being tracked?
Where is the target relative to the robot/gripper?
```

Localisation answers:

```text
Where is the robot in the arena?
```

Neither should own the other's job.

---

# Runtime Flow

The current runtime concept is:

```text
Controller tick
      |
      v
obtain current Vision message
      |
      +--------------------------+
      |                          |
      v                          v
 Perception                 Localisation
                                 |
                    +------------+-------------+
                    |                          |
                    v                          v
              VisionArbiter             OdometryArbiter
                    |                          |
                    |                    DriveEncoderProvider
                    |                          |
                    +------------+-------------+
                                 |
                     CommandedMotionProvider
                                 |
                                 v
                           PoseEstimator
                                 |
                       currently Arbitrator
                                 |
                                 v
                           current Pose
```

The exact call structure need not mirror this diagram line-for-line; it shows ownership
and evidence flow.

Motion commands separately inform the commanded-motion fallback:

```text
Motion stack
      |
      +---- begin_commanded_drive(...)
      |
      +---- begin_commanded_rotate(...)
                 |
                 v
          CommandedMotionProvider
```

Measured odometry reaches localisation through semantic IO and its independent
`DriveEncoderOdometry` instance.

---

# What Does Not Belong Here

The localisation package should not contain:

- camera device paths;
- OpenCV capture;
- Picamera2 capture;
- V4L2 setup;
- Student Robotics board wiring;
- object selection;
- target approach policy;
- motor PWM;
- servo control;
- competition strategy;
- robot-specific encoder pin assignments;
- robot-specific odometry geometry embedded directly in algorithms.

Those responsibilities belong elsewhere.

---

# Current Design Status

The current architecture now supports:

- configured startup pose;
- independent position and heading validity;
- `Pose` covariance;
- `PoseObservation` covariance;
- multiple localisation provider families;
- multiple visual localisation methods;
- Vision-level method arbitration;
- generic robot-relative `OdometrySource`;
- generic `OdometryPoseProvider`;
- measured drive-encoder pose propagation;
- independent localisation odometry state;
- `OdometryArbiter` family boundary;
- commanded-motion fallback;
- final absolute-vs-propagated arbitration;
- provider reseeding from absolute corrections;
- covariance preservation through family/final selection where available;
- a generic `PoseEstimator` boundary;
- simulation and physical robots using the same localisation architecture.

The current default provider tree is:

```text
VisionArbiter
    Cam1Markers2Provider
    AprilTagPnPPoseProvider

OdometryArbiter
    DriveEncoderProvider

CommandedMotionProvider
```

The current default final estimator is:

```text
Arbitrator
```

Deferred until there is a concrete hardware or performance need:

- three-deadwheel localisation provider registration/configuration;
- two-deadwheel + IMU odometry;
- OTOS integration;
- multi-camera family arbitration;
- proper covariance propagation/process-noise models;
- EKF or other sensor fusion.

---

# Future Fusion Direction

The family boundaries remain useful even after simple final arbitration is replaced.

The intended long-term structure is:

```text
individual sensor / method providers
            |
            v
       FAMILY ARBITERS
            |
            v
      FUSION ESTIMATOR
       (EKF or similar)
            |
            v
      Localisation Pose
```

Conceptually:

```text
vision methods/cameras
        |
        v
   Vision family result ----\

odometry implementations ----> Fusion
        |                     /
        v                    /
  Odometry family result ---/

OTOS ----------------------/

Dead reckoning -----------/
```

Fusion should normally consume one result from each family rather than every correlated
child measurement independently.

For example, PnP and marker triangulation derived from the same camera frame are
correlated. Feeding both independently into a future EKF as though they were unrelated
measurements would overstate the available information.

Likewise, if a future odometry provider already incorporates IMU heading, the same IMU
should not automatically be fed independently again without accounting for that
correlation.

The current family arbiters therefore remain part of the long-term design rather than
being temporary code to discard when fusion arrives.

---

# Summary

Localisation has one central responsibility:

> **Maintain the best available estimate of the robot's pose in the arena.**

The current architecture is:

```text
          Vision methods
               |
               v
          VisionArbiter
               |
               |
Drive encoders -> OdometryArbiter
               |
Commanded motion
               |
               v
          PoseEstimator
      currently Arbitrator
               |
               v
          current Pose
```

The most important design rules are:

1. Localisation owns robot pose.
2. Localisation consumes Vision directly.
3. Localisation does not consume Perception.
4. Position and heading validity are independent.
5. Absolute and propagated observations are different classes of evidence.
6. Absolute corrections reseed propagation.
7. Propagation must not reseed itself.
8. Robot-relative odometry uses a generic `reset()/read()` boundary.
9. Odometry family selection is separate from global pose estimation.
10. Hardware-specific details do not belong in localisation algorithms.
11. All providers use one common arena coordinate convention.
12. Covariance is optional and must never be invented.
13. Family arbiters preserve the selected provider's information.
14. `Localisation` depends on the `PoseEstimator` boundary, not directly on a future-specific fusion implementation.
15. The current simple `Arbitrator` can later be replaced by fusion without restructuring the whole package.
