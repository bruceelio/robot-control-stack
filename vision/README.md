vision/README.md

# vision — Shared Vision Observations

This folder defines the **shared vision boundary** between camera/detector hardware and
the systems which interpret what the robot sees.

Vision answers:

> **What did the camera observe?**

It does not decide:

- which object the robot should collect;
- where the robot is in the arena;
- which behaviour should run;
- how motors or servos should move.

Those decisions belong to Perception, Localisation and Behaviours.

---

# Architectural Boundary

The central rule is:

> **Perception consumes Vision. Localisation consumes Vision. Localisation does not consume Perception.**

Conceptually:

```text
                     CAMERA / DETECTOR
                            |
                            v
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

Vision is therefore a neutral observation layer shared by two independent consumers.

---

# What Vision Owns

Vision owns the representation and reconciliation of camera observations.

Typical responsibilities include:

- retaining the logical camera name;
- retaining the actual measurement timestamp;
- carrying corrected range/bearing detections;
- carrying raw marker information where a downstream solver needs it;
- converting hardware-specific AprilTag marker objects into neutral observations;
- preserving enough geometry for object perception and robot localisation.

Vision should not contain:

- target-selection policy;
- approach behaviour;
- robot-pose arbitration;
- motor commands;
- servo commands;
- competition strategy.

---

# Camera Capture Is Below Vision

Camera hardware and capture live under:

```text
hw_io/cameras/
```

Examples include:

```text
Pi CSI / Picamera2
USB / OpenCV / V4L2
Student Robotics / Webots camera API
```

These backends answer:

> How do I obtain camera observations from this hardware?

Vision answers:

> How do I represent those observations consistently for the rest of the robot?

The distinction is:

```text
camera backend
    |
    | hardware-specific capture
    v
raw markers / detections
    |
    v
VISION
    |
    | hardware-independent observations
    v
Perception / Localisation
```

---

# Logical Camera Identity

A logical camera name is a role on the robot.

Examples:

```text
front
rear
```

It is not the same thing as:

```text
camera model
camera profile
device path
camera calibration
```

For example:

```python
CAMERAS = {
    "front": {
        "profile": "arducam_fullfov_640_400",
        "device": "/dev/v4l/by-id/...",
    }
}
```

Here:

```text
front
```

is the logical camera.

The logical camera identity must remain attached to the observation so downstream
code can choose the correct calibration and mount information.

---

# Vision Message

The normal runtime boundary is a camera-labelled Vision message.

Conceptually:

```python
{
    "camera": "front",
    "timestamp": 123.456,
    "detections": [...],
    "status": "ok",
    "markers": [...],        # retained where required
}
```

The core fields are:

```text
camera
timestamp
detections
status
```

Raw `markers` may also be attached when downstream AprilTag reconciliation or PnP
requires information which is not present in the simplified detections.

---

# Measurement Timestamp

The timestamp is the time the observation was made.

It is **not**:

```text
the time Perception processed it
the time Localisation processed it
the current controller time
```

This is important for freshness checks.

Conceptually:

```text
camera measurement
      |
      | timestamp belongs here
      v
Vision message
      |
      +----> Perception
      |
      +----> Localisation
```

Both consumers therefore evaluate the age of the same actual measurement.

---

# Corrected Detections

The simplified `detections` list contains camera-relative observations suitable for
normal robot interpretation.

A detection conceptually contains:

```python
{
    "id": 171,
    "distance_mm": 1100.0,
    "bearing_deg": -6.5,
    "camera": "front",
}
```

The current detection pipeline applies configured optical corrections such as:

```text
distance scale
bearing sign
bearing offset
camera yaw
```

before publishing the canonical detection.

That means downstream code should not silently apply the same corrections again.

---

# Canonical IO Principle

The same general rule used by the hardware layer applies here:

> Once an observation has crossed the Vision boundary, its canonical values should
> have one clear meaning.

For example:

```text
distance_mm
bearing_deg
timestamp
camera
```

should not acquire hidden additional corrections in unrelated downstream code.

If another representation is required, it should be an explicit transformation with
an explicit name.

---

# AprilTag Observations

AprilTag localisation may require more information than simple distance and bearing.

The neutral AprilTag representation is defined under:

```text
vision/apriltag/
```

The important abstraction is:

```text
AprilTagObservation
```

It can carry information such as:

```text
source_id
camera
timestamp
tag_id

distance
horizontal angle
vertical angle

yaw / pitch / roll

pixel centre
pixel corners

tag size
decision margin
tag family

per-tag translation
pose error
```

Not every camera/backend must provide every field.

A downstream provider should use only the fields it actually requires and validate
that they are present.

---

# AprilTag Reconciliation

Different camera backends do not necessarily return identical marker classes.

For example:

```text
Student Robotics / Webots marker
Pi/USB AprilTag marker
```

may expose different optional details.

The reconciliation layer converts those marker objects into neutral
`AprilTagObservation` objects.

Conceptually:

```text
SR/Webots marker --------\
                          \
Pi/USB marker -------------+--> reconcile --> AprilTagObservation
                          /
future marker backend ----/
```

The current reconciliation code lives under:

```text
vision/apriltag/reconcile.py
```

Its responsibilities include:

- identifying the configured Vision source for the logical camera;
- copying the actual camera measurement timestamp;
- converting raw marker attributes into the neutral observation;
- leaving unavailable attributes as unavailable rather than inventing values.

---

# Vision Source

Robot configuration can associate a logical camera with a Vision source.

Conceptually:

```python
VISION_SOURCES = {
    "vision1": {
        "camera": "front",
        "enabled": True,
    }
}
```

The source ID is provenance.

It allows downstream systems to know which configured observation stream produced
the measurement without depending on a hardware class or device path.

---

# Two Different Geometry Problems

There are two important vision geometry problems and they must remain separate.

## 1. Camera-to-object geometry

This answers questions such as:

```text
How far away is this tag?
What is its bearing?
What is its vertical angle?
```

These measurements are useful to Perception and object-approach behaviours.

Conceptually:

```text
camera
   |
   v
individual tag
```

---

## 2. Arena localisation geometry

This answers:

```text
Given known arena markers, where is the camera/robot in the arena?
```

Conceptually:

```text
known arena tags
      |
      v
camera pose
      |
      v
robot pose
```

This belongs to Localisation providers.

Therefore:

> Global robot-pose PnP does not belong in the camera backend or object Perception.

---

# Perception Consumer

Perception uses Vision to interpret objects.

Conceptually:

```text
Vision detection
      |
      v
object classification
      |
      v
target-relative geometry
      |
      v
tracking / visibility
      |
      v
behaviour
```

Perception may transform camera-relative observations into geometry useful to the
manipulator, for example:

```text
camera-relative target
        |
        v
gripper-relative target
```

That transformation is object interpretation, not robot localisation.

See:

```text
/perception/README.md
```

---

# Localisation Consumer

Localisation consumes Vision independently.

Conceptually:

```text
Vision message
      |
      v
VisionArbiter
      |
      +---- range/bearing marker provider
      |
      +---- AprilTag PnP provider
      |
      v
PoseObservation
      |
      v
Localisation Arbitrator
```

The Vision-level localisation arbiter may choose between multiple ways of deriving
pose from the same current Vision message.

It should preserve the selected provider's:

```text
source
timestamp
confidence
validity
```

See:

```text
/localisation/README.md
```

---

# Current Vision-to-Localisation Path

The current localisation Vision arbiter accepts the canonical Vision message.

Conceptually it performs two parallel adaptations:

```text
Vision message
      |
      +-------------------------------+
      |                               |
      v                               v
detections                      raw markers
      |                               |
      v                               v
range/bearing provider        reconcile AprilTags
                                      |
                                      v
                              AprilTag PnP provider
```

This keeps hardware-specific marker handling outside the pose solvers.

---

# Sync and Async Capture

Physical cameras may run asynchronously.

Webots / Student Robotics may provide observations synchronously.

These paths should still converge on the same conceptual Vision message.

```text
ASYNC CAMERA                     SYNC CAMERA
     |                               |
CameraProcessManager                 |
     |                               |
latest camera result                 |
     +---------------+---------------+
                     |
                     v
                Vision message
                     |
             +-------+-------+
             |               |
             v               v
        Perception      Localisation
```

Higher-level consumers should not need different object or localisation algorithms
merely because capture was synchronous or asynchronous.

---

# Freshness

Vision itself preserves measurement time.

Consumers decide how fresh an observation must be for their own purpose.

For example:

```text
close-range object control
    may require a very fresh observation

robot localisation
    may tolerate a slightly older pose measurement
```

Those are separate policies.

Vision should not force one universal freshness threshold onto all consumers.

---

# Calibration Boundaries

Several different concepts must remain separate:

```text
camera operating profile
camera intrinsic calibration
optical correction
robot camera mount
arena geometry
```

They answer different questions.

## Camera operating profile

How is the camera run?

Examples:

```text
resolution
capture format
exposure
gain
focus
```

## Intrinsic calibration

How does the lens/image map pixels to rays?

Examples:

```text
fx
fy
cx
cy
distortion
```

## Optical correction

What correction is required for the canonical measured distance/bearing?

Examples:

```text
distance_scale
bearing_sign
bearing_offset_deg
```

## Camera mount

Where is the camera on this robot?

Examples:

```text
x
y
z
roll
pitch
yaw
```

## Arena geometry

Where are known markers in the world?

This belongs to arena configuration, not the camera.

---

# Multi-Camera Rule

Each logical camera should produce independent observations first.

Do not combine cameras at capture level.

Preferred architecture:

```text
front camera ---> Vision observations ----\
                                            \
                                             ---> consumer arbitration/fusion
                                            /
rear camera ----> Vision observations -----/
```

This preserves:

- camera identity;
- camera calibration;
- mount transform;
- timestamp;
- provenance;
- independent diagnostics.

Perception and Localisation may make different decisions about how to combine those
streams.

---

# Directory Responsibility

The current architecture spans several folders:

| Area | Responsibility |
|---|---|
| `hw_io/cameras/` | Camera access, capture and worker/process integration |
| `perception/vision/detection_pipeline.py` | Build corrected canonical detection messages |
| `vision/apriltag/` | Neutral AprilTag observations and reconciliation |
| `vision/` | Shared Vision-side support/calibration as it is migrated here |
| `perception/` | Object interpretation and tracking |
| `localisation/providers/vision/` | Convert Vision evidence into robot-pose evidence |
| `localisation/` | Maintain/arbitrate robot pose |
| `behaviors/` | Decide what the robot should do |

Some historical Vision-related implementation may still physically live under
`perception/vision/`.

The architectural rule is more important than the temporary file location:

> Shared observation logic should not become owned by Perception merely because a
> file has not yet been moved.

---

# What Does Not Belong Here

The Vision package should not own:

- physical motor wiring;
- camera device opening;
- behaviour state machines;
- object-selection policy;
- final robot pose;
- localisation arbitration;
- motion commands;
- servo control;
- competition strategy.

---

# Development Rules

1. Preserve logical camera identity.
2. Preserve the actual measurement timestamp.
3. Do not hide hardware-specific marker classes inside localisation providers.
4. Do not make Localisation depend on Perception.
5. Do not apply the same optical correction twice.
6. Keep camera capture below Vision.
7. Keep robot-pose solving in Localisation.
8. Keep object interpretation in Perception.
9. Keep multi-camera observations independent until a consumer combines them.
10. Prefer one canonical observation representation over parallel hardware-specific
    representations.

---

# Summary

Vision has one central responsibility:

> **Provide a neutral, camera-labelled, time-labelled description of what was observed.**

The architecture is:

```text
camera hardware / simulation
          |
          v
        VISION
          |
     +----+----+
     |         |
     v         v
Perception  Localisation
     |         |
objects     robot pose
```

The most important boundary is:

> **Perception consumes Vision. Localisation consumes Vision. Localisation does not consume Perception.**
