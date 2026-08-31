percetption/README.md

# perception — Object Interpretation and Tracking

This folder owns the robot's interpretation of **objects and targets** seen through
Vision.

Perception answers questions such as:

```text
What objects are visible?
What kind of object is this?
Where is the object relative to the robot or gripper?
Is the selected target still visible?
```

Perception does **not** own the robot's global arena pose.

That belongs to Localisation.

---

# Architectural Boundary

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

Perception and Localisation are therefore sibling consumers of Vision.

Perception must not become an accidental intermediate layer between Vision and
Localisation.

---

# What Perception Owns

Perception owns robot-useful interpretation of observed objects.

Typical responsibilities include:

- classifying visible object tags;
- separating arena markers from game objects where required;
- converting camera observations into object records;
- applying object/manipulator-relative geometry;
- maintaining target visibility information;
- supporting target locking/tracking;
- exposing observations which behaviours can use.

Perception should not contain:

- camera device paths;
- camera capture;
- Student Robotics board access;
- final robot-pose arbitration;
- motor control;
- servo control;
- competition strategy.

---

# Input — Vision

Perception consumes the current canonical Vision observation.

Conceptually:

```text
camera
   |
   v
VISION
   |
   v
detections
   |
   v
PERCEPTION
```

Vision is responsible for preserving:

```text
camera
timestamp
corrected distance
corrected bearing
status
```

and any additional marker information needed by other consumers.

Perception should not reopen the camera or independently reconstruct the same
hardware-specific observation.

---

# Canonical Detection

A simplified Vision detection conceptually looks like:

```python
{
    "id": 171,
    "distance_mm": 1100.0,
    "bearing_deg": -6.5,
    "camera": "front",
}
```

Perception adds meaning to that observation.

For example:

```text
tag 171
   |
   v
basic game object
   |
   v
candidate target
```

The exact game-object classification rules belong in Perception/configuration, not
inside the camera backend.

---

# Object Types

Perception can distinguish different semantic categories of visible markers.

Examples in the current competition stack include:

```text
arena
acidic
basic
```

The camera does not inherently know that a particular tag is an acidic or basic
object.

That is robot/competition interpretation performed above Vision.

Conceptually:

```text
Vision:
    tag_id = 171

Perception:
    tag_id = 171
    kind = basic
```

---

# Camera-Relative Geometry

Vision provides measurements relative to a logical camera.

For example:

```text
distance from camera
bearing from camera
vertical angle from camera
```

These are useful, but behaviours often need geometry relative to the robot's
manipulator.

Perception is an appropriate place to expose that robot-useful object geometry.

---

# Gripper-Relative Geometry

For object approach, the important point is often not the camera position but the
pickup mechanism.

Conceptually:

```text
camera observation
      |
      v
camera-relative object
      |
      | known camera/gripper geometry
      v
gripper-relative target
      |
      v
AcquireObject
```

This is why runtime diagnostics may distinguish:

```text
cam_dist
cam_bearing
```

from:

```text
grip_dist
grip_bearing
```

The camera values describe the observation.

The gripper values describe how that observation relates to the pickup mechanism.

---

# Perception Does Not Own Robot Pose

Perception should not answer:

```text
Where is the robot in the arena?
```

That belongs to:

```text
/localisation/
```

Incorrect architecture:

```text
Vision
   |
   v
Perception
   |
   v
Localisation
```

Correct architecture:

```text
            Vision
           /     \
          /       \
         v         v
Perception     Localisation
```

This matters because arena-marker observations may be useful to Localisation even
when they have no object-tracking meaning.

---

# Perception Does Not Own Camera Hardware

Perception should not contain hardware-specific capture code such as:

```text
Picamera2
OpenCV VideoCapture
V4L2 device paths
Student Robotics camera-board access
```

Those responsibilities belong under:

```text
hw_io/cameras/
```

Perception receives observations after the hardware boundary has already been crossed.

---

# Detection Pipeline

The current stack includes:

```text
perception/vision/detection_pipeline.py
```

which builds corrected camera-labelled detection messages.

Conceptually:

```text
raw marker
    |
    v
optical correction
    |
    v
canonical detection
    |
    v
Perception
```

This file currently sits physically under `perception/vision/`, but the canonical
Vision message it constructs is shared by both Perception and Localisation.

As the architecture is cleaned up further, shared Vision concerns should remain
conceptually neutral even if some historical files have not yet moved.

---

# Runtime Flow

At runtime the controller obtains one current Vision message.

That same measurement is then made available independently to Perception and
Localisation.

Conceptually:

```text
Controller tick
      |
      v
current Vision message
      |
      +-----------------------+
      |                       |
      v                       v
 Perception              Localisation
      |                       |
objects / targets           robot pose
```

Perception should not request a second camera frame merely because Localisation also
needs Vision.

Both systems should reason about the same measurement where practical.

---

# Object Visibility

Perception is responsible for interpreting whether an object is currently visible.

A target observation should retain enough information to distinguish:

```text
visible now
recently visible
lost
```

Freshness is important because the robot may continue moving after the last camera
measurement.

The current camera measurement timestamp therefore remains important even after the
observation has been interpreted as an object.

---

# Target Tracking

Perception supports behaviours by maintaining object identity over time.

Conceptually:

```text
Vision frame 1: tag 171 seen
Vision frame 2: tag 171 seen
Vision frame 3: tag 171 not seen
```

can become:

```text
target id = 171
visible = false
last_seen_age = ...
```

This allows a behaviour to distinguish:

```text
new target
same locked target
brief dropout
target lost
```

Target tracking does not mean Perception owns behaviour policy.

---

# Behaviour Boundary

Behaviours consume Perception outputs.

For example:

```text
Perception
    |
    v
target object
    |
    v
AcquireObject
```

The behaviour decides:

```text
which target to pursue
whether to rotate
whether to drive
when to commit
when to grab
when to recover
```

Perception supplies evidence.

It does not decide the autonomous sequence.

---

# Height / Object Geometry

Object observations may include information used for classifications such as high/low
target geometry.

Raw visual quantities such as:

```text
vertical angle
distance
```

originate with Vision.

A higher-level model may then use those observations to infer object geometry needed
by the behaviour.

The important boundary is:

```text
camera measurement
    !=
behaviour decision
```

Perception can expose the measurement and object interpretation without embedding the
whole approach policy.

---

# Arena Markers

Arena markers can appear in the same camera observation as game-object markers.

Perception may classify or report them, but Localisation receives Vision independently.

Therefore:

```text
arena marker detected
```

does not need to pass through object Perception before it can contribute to robot
pose.

This avoids coupling localisation reliability to object-tracking code.

---

# Multi-Camera Perception

When multiple logical cameras are enabled, each should produce independent Vision
observations first.

Perception can then choose how to interpret object observations across cameras.

Conceptually:

```text
front Vision ----\
                  \
                   ---> Perception ---> object tracks
                  /
rear Vision -----/
```

Possible future policies include:

- newest observation wins;
- designated camera preference;
- best-confidence observation;
- robot-relative track fusion;
- separate front/rear tracking.

Those policies belong above camera capture.

---

# Calibration and Mount Geometry

Perception should distinguish between:

```text
camera intrinsic calibration
camera optical correction
camera mount
gripper/manipulator geometry
```

They are not interchangeable.

## Intrinsic calibration

Describes the lens/image geometry.

## Optical correction

Produces corrected canonical range/bearing observations.

## Camera mount

Describes where the camera is on the robot.

## Gripper/manipulator geometry

Describes where the pickup mechanism is relative to the robot/camera.

This separation lets the same calibrated camera be installed at a different robot
position without changing its lens calibration.

---

# Perception Output

Perception outputs should be robot-useful semantic observations rather than raw camera
objects.

Conceptually:

```python
{
    "id": 171,
    "kind": "basic",
    "distance_mm": ...,
    "bearing_deg": ...,
    "camera": "front",
    "timestamp": ...,
}
```

Additional object-relative fields can be attached where useful.

The important point is that behaviours should not need to know whether the source was:

```text
Picamera2
USB/OpenCV
Student Robotics camera
Webots
```

---

# Error Isolation

The architecture is designed so failures can be separated.

```text
camera does not open
    -> hw_io/cameras

tag not detected
    -> detector / Vision processing

distance/bearing wrong
    -> calibration / optical correction

object classified incorrectly
    -> Perception

robot pose wrong
    -> Localisation

robot chooses wrong action
    -> Behaviour / strategy
```

Keeping these boundaries explicit makes debugging substantially easier.

---

# What Does Not Belong Here

The Perception package should not own:

- physical camera opening;
- V4L2 configuration;
- Pi camera exposure setup;
- Student Robotics hardware mapping;
- robot global pose;
- localisation arbitration;
- motor PWM;
- drive calibration;
- lift/gripper servo output;
- autonomous state-machine sequencing.

---

# Relationship to Vision

Vision answers:

> What did the camera observe?

Perception answers:

> What does that observation mean as an object/target?

Example:

```text
VISION
    tag_id=171
    distance=1100 mm
    bearing=-6.5 deg
        |
        v
PERCEPTION
    object id=171
    kind=basic
    target geometry=...
```

See:

```text
/vision/README.md
```

---

# Relationship to Localisation

Localisation answers:

> Where is the robot?

It consumes Vision separately.

Perception should therefore never be required merely to make visual localisation
work.

See:

```text
/localisation/README.md
```

---

# Development Rules

1. Perception consumes Vision, not camera hardware.
2. Localisation does not consume Perception.
3. Preserve object identity and measurement time.
4. Keep camera-relative and manipulator-relative geometry explicit.
5. Do not hide behaviour policy inside object interpretation.
6. Do not hide robot-pose estimation inside object interpretation.
7. Do not apply Vision optical corrections a second time.
8. Keep multiple camera observations identifiable by camera.
9. Keep hardware-specific classes below the Vision boundary.
10. Make perception outputs useful to behaviours without exposing camera implementation details.

---

# Summary

Perception has one central responsibility:

> **Turn neutral Vision observations into robot-useful objects and targets.**

The architecture is:

```text
Vision
  |
  v
Perception
  |
  v
objects / target tracks
  |
  v
Behaviours
```

while robot pose follows the separate path:

```text
Vision
  |
  v
Localisation
  |
  v
robot pose
```

The most important design rule is therefore:

> **Perception consumes Vision. Localisation consumes Vision. Localisation does not consume Perception.**
