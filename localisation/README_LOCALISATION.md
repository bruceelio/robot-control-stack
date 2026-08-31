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
OTOS
startup pose
```

Vision derives pose from known arena landmarks.

An appropriately configured OTOS may also provide an arena-referenced pose.

The configured startup pose establishes the initial known pose before movement.

---

## Propagation sources

These do not independently establish where the robot is in the arena.

Instead they update an already known pose as the robot moves.

Examples include:

```text
commanded motion
wheel encoders / odometry
IMU heading propagation
```

Conceptually:

```text
                 ABSOLUTE / CORRECTIVE
                ┌─────────────────────┐
                │ Vision              │
                │ OTOS                │
                │ Startup pose        │
                └──────────┬──────────┘
                           |
                           v
                      CURRENT POSE
                           ^
                           |
                ┌──────────┴──────────┐
                │                     │
           Commanded motion       Encoders
                                      |
                                     IMU
```

This distinction is important.

A propagated pose may be the best available estimate while absolute localisation is
unavailable, but it should not be treated as a new independent absolute measurement.

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
    source="front:pnp:3tags",
    timestamp=...,

    is_absolute=True,

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

`confidence` is a numeric value from:

```text
0.0 to 1.0
```

It expresses the provider's confidence in the observation.

Confidence is useful for selecting between observations of the **same general class**.

For example:

```text
vision candidate A
vs
vision candidate B
```

or:

```text
propagation candidate A
vs
propagation candidate B
```

Confidence should not be interpreted as making a propagated pose equivalent to an
absolute measurement.

The architecture first distinguishes:

```text
absolute
vs
propagated
```

and then compares suitable candidates within that class.

---

# Source

Every observation carries a `source`.

The source is primarily for:

- diagnostics;
- logging;
- understanding which localisation method is active;
- comparing alternative providers.

Examples may include:

```text
startup_config

front:markers2:2tags
front:markers2:3tags

front:pnp:1tag
front:pnp:2tags
front:pnp:3tags

commanded_motion
```

The source should describe the observation actually selected rather than hide it
behind a generic name.

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

These thresholds should therefore not automatically be forced to the same value.

---

# Provider Architecture

Providers implement the common localisation provider interface.

Current structure is broadly:

```text
localisation/
    README.md
    __init__.py
    localisation.py
    arbitration.py
    pose_types.py

    providers/
        __init__.py
        base.py

        vision/
            vision_arbiter.py
            pose_cam1_markers2.py
            pose_apriltag_pnp.py

        motion/
            commanded_motion.py
```

Additional provider families can be added as hardware is introduced, for example:

```text
odometry/
otos/
inertial/
```

Providers are organised by **evidence source**, not by robot.

There should not be separate localisation algorithms merely because the program is
running on:

```text
Webots
rob_bot
bob_bot
Student Robotics hardware
```

Hardware differences belong below localisation.

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
vision localisation providers
   |
   v
PoseObservation
```

Known arena marker geometry is then used to infer robot pose.

---

# VisionArbiter

There may be more than one way to calculate pose from the same Vision observation.

Current examples include:

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
                Vision message
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
             best vision evidence
```

The VisionArbiter chooses between available visual pose candidates.

It should preserve the selected observation's:

```text
source
timestamp
confidence
validity
```

rather than replacing those values with artificial downstream information.

If the current Vision observation contains no usable localisation information,
Vision should simply be unavailable for that cycle.

Old visual measurements should not be silently cached and presented as though they
were current observations.

---

# Final Localisation Arbitration

The final arbitrator receives observations from the configured localisation providers.

Conceptually:

```text
VisionArbiter --------\
                       \
OTOS -------------------> Localisation Arbitrator
                         /
Commanded Motion -------/
                       /
Encoders --------------/
```

The current policy is intentionally simple:

1. reject unusable observations;
2. reject stale observations;
3. reject invalid confidence values;
4. separate absolute and propagated observations;
5. if an absolute observation is available, use the absolute class;
6. otherwise use propagation;
7. within the selected class, prefer the highest-confidence observation;
8. use freshness as a secondary preference where necessary.

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
highest suitable confidence
       |
       v
current pose
```

---

# Reseeding

Propagation sources need a known pose from which to continue.

When a new **absolute** pose is accepted:

```text
absolute correction
        |
        v
current pose
        |
        v
reseed propagation providers
```

For example:

```text
Vision fix
    |
    v
CommandedMotion reseeded
```

This prevents accumulated propagation drift from surviving a good absolute correction.

---

## Propagation Must Not Reseed Itself

A propagated observation must be allowed to continue accumulating motion.

Therefore:

> **Propagated observations must not cause the propagation providers to be reseeded from their own output.**

Otherwise the final fraction of an active movement can be lost.

Correct behaviour:

```text
Vision / OTOS
    |
    | absolute correction
    v
RESEED propagation

Commanded Motion / Encoders
    |
    | propagated estimate
    v
DO NOT reseed propagation
```

Explicit `set_pose()` operations may reseed all providers because they deliberately
establish a new known pose.

---

# Commanded Motion Provider

`commanded_motion.py` provides temporary pose propagation based on the motion the robot
was instructed to perform.

It does not measure actual wheel movement.

It therefore provides:

```text
propagated estimate
```

rather than:

```text
absolute pose
```

The motion backend informs localisation when a timed drive or rotation begins.

For example:

```text
BEGIN_DRIVE
distance
duration
start time
```

or:

```text
BEGIN_ROTATE
angle
duration
start time
```

The provider advances the estimated pose as the command progresses.

This gives localisation a useful temporary estimate when absolute Vision is
unavailable.

---

# Commanded Motion Limitations

Commanded motion assumes that commanded movement approximately matches physical
movement.

Real robots introduce:

- wheel slip;
- battery effects;
- surface variation;
- unequal motors;
- collisions;
- mechanical tolerances.

Commanded motion is therefore expected to drift.

It is a fallback / propagation source, not a replacement for measured localisation.

---

## Small Rotations

Very small commanded rotations are intentionally not necessarily propagated into the
localisation estimate.

The physical robot showed that repeated small rotation estimates could accumulate
significant heading error.

The robot may therefore physically execute a small turn while commanded-motion
localisation deliberately ignores that turn.

This is a localisation policy, not a motor-control limitation.

---

# Future Encoder Integration

Encoders belong naturally on the propagation side of the architecture.

They measure actual wheel movement and should therefore eventually provide better
motion information than commanded motion.

Conceptually:

```text
current pose
    |
    +---- commanded motion
    |
    +---- wheel encoders
    |
    +---- IMU
```

Encoders still do not independently establish the global arena pose.

They propagate an existing pose.

---

# OTOS

OTOS differs from ordinary wheel odometry because it may provide a direct pose
estimate when correctly referenced to the arena frame.

Where configured as an arena-referenced pose source it belongs on the
absolute/corrective side:

```text
Vision
OTOS
```

rather than being treated as just another competing dead-reckoning estimate.

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
initial current pose
        |
        v
reseed propagation providers
```

This provides the initial reference from which motion propagation can begin.

---

# Coordinate Convention

Arena coordinates use a fixed world frame.

The intended field convention is:

```text
heading 0°     = +X
heading +90°   = +Y
positive angle = counter-clockwise
```

All localisation providers should ultimately express pose using the same convention.

Motor/rotation command sign conventions are a separate issue.

The motion stack still needs a final audit so that:

```text
positive commanded rotation
```

is consistently interpreted as:

```text
counter-clockwise
```

throughout the complete stack.

Do not change localisation coordinate conventions merely to compensate for a
motor-command sign mismatch.

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

The normal runtime concept is:

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
      |                          |
objects/targets              pose providers
                                 |
                                 v
                         VisionArbiter
                                 |
                                 v
                     Localisation Arbitrator
                                 |
                                 v
                           current Pose
```

Motion commands separately feed propagation providers:

```text
Motion Backend
      |
      +---- begin_commanded_drive(...)
      |
      +---- begin_commanded_rotate(...)
                 |
                 v
          CommandedMotionProvider
```

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
- competition strategy.

Those responsibilities belong elsewhere.

---

# Current Design Status

The current architecture supports:

- configured startup pose;
- independent position and heading validity;
- multiple localisation providers;
- multiple visual localisation methods;
- vision-level arbitration;
- final absolute-vs-propagated arbitration;
- commanded-motion fallback;
- provider reseeding from absolute corrections;
- simulation and physical robots using the same localisation architecture.

Planned additions include:

- encoder-based propagation;
- OTOS integration;
- IMU support;
- stronger uncertainty modelling;
- multi-camera localisation;
- eventual EKF-based sensor fusion.

---

# Future EKF Direction

The current provider/arbitration architecture is intentionally understandable and
useful while the robot has relatively few localisation sensors.

As measured motion sources are added:

```text
encoders
IMU
OTOS
multiple cameras
```

an Extended Kalman Filter may become the more natural fusion architecture.

In that future model:

```text
absolute sources
    -> pose measurements + covariance

propagation sensors
    -> motion / velocity / delta measurements + covariance
```

The current distinction between:

```text
absolute correction
```

and:

```text
pose propagation
```

is therefore important even if the final implementation later moves to an EKF.

---

# Summary

Localisation has one central responsibility:

> **Maintain the best available estimate of the robot's pose in the arena.**

The architecture is:

```text
absolute sources establish/correct pose
                +
propagation sources carry pose between corrections
                |
                v
        Localisation Arbitrator
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
8. Hardware-specific details do not belong in localisation.
9. All providers use one common arena coordinate convention.
10. The architecture should remain suitable for eventual EKF-based fusion.
