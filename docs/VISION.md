# Vision Architecture

## Purpose

This document describes how the robot vision system is organised across the software stack.

The design goal is to keep three concerns separate:

1. **Camera capture** — obtain an image or camera observation from a specific hardware/backend.
2. **Vision processing** — detect and describe things in that camera data.
3. **Robot interpretation** — convert detections into object information or robot-localisation evidence.

This separation allows:

- multiple cameras to operate at the same time;
- different physical camera types to use the same higher-level vision logic;
- cameras to be pointed forward, backward, or elsewhere without changing detector code;
- different detectors to be added later without redesigning the camera hardware layer;
- localisation to use camera observations without knowing whether they came from a Pi Camera, USB camera, or Student Robotics API;
- camera-specific calibration and robot-specific mounting geometry to remain separate.

---

# 1. Three-Level Vision Model

```text
LEVEL 1 — CAMERA / CAPTURE
    Physical device + camera backend
                |
                v
         image / SR markers

LEVEL 2 — VISION PROCESSING
    detector / processor / observation adapter
                |
                v
        generic observations

LEVEL 3 — ROBOT INTERPRETATION
    object perception OR localisation provider
                |
                v
        behaviour / robot pose
```

The key rule is:

> Camera identity, camera profile, detector, and robot use of the result are separate concepts.

---

# 2. Level 1 — Camera / Capture

Level 1 owns the physical camera and the method used to obtain data from it.

## Logical camera identity

A robot profile gives each installed camera a logical name.

Example:

```python
CAMERAS = {
    "front": {
        "profile": "arducam_fullfov_640_400",
        "device": "/dev/v4l/by-id/...",
    }
}
```

The logical name `"front"` means:

> this is the robot camera currently assigned to the front-camera role.

It does **not** mean a particular camera model.

For a two-camera robot, logical identities might eventually be:

```python
CAMERAS = {
    "front": {...},
    "rear": {...},
}
```

For temporary comparison testing, both physical cameras can face forward while retaining separate logical identities:

```python
CAMERAS = {
    "front_pi3": {...},
    "front_arducam": {...},
}
```

This is preferable to pretending that both cameras are the same logical camera.

---

## Camera profile

The reusable camera profile describes how that camera should run.

Examples:

```text
config/cameras/pi3_fullfov_640_360.py
config/cameras/arducam_fullfov_640_400.py
```

A profile contains camera-specific settings such as:

- backend;
- capture resolution;
- processing resolution;
- frame rate;
- pixel format;
- AprilTag detector settings currently associated with that stream;
- calibration-profile name;
- optional camera-control overrides;
- reference/tuning information.

The profile is reusable between robots.

---

## Physical device

The robot profile owns the actual physical device assignment.

Examples:

```text
Pi Camera:
device = 0

Arducam USB:
device = /dev/v4l/by-id/usb-...-video-index0
```

This belongs to the robot because it describes that robot's wiring/device arrangement.

---

## Camera backends

Current camera backends live under:

```text
hw_io/cameras/
```

Important backends are:

```text
pi_libcamera_april.py
opencv_usb.py
sr_april.py
```

### `pi_libcamera_april.py`

Uses:

```text
Picamera2 / libcamera
```

for Raspberry Pi CSI cameras.

Responsibilities include:

- opening the Pi camera;
- selecting sensor/output configuration;
- applying supported camera controls;
- capturing frames;
- passing frames to the shared AprilTag processor.

### `opencv_usb.py`

Uses:

```text
OpenCV VideoCapture / V4L2
```

for USB cameras such as the Arducam OV9281.

Current Arducam flow:

```text
OV9281
    |
    | 1280 x 800 MJPG
    v
OpenCV VideoCapture
    |
    | resize
    v
640 x 400 RGB frame
```

Because 1280×800 and 640×400 have the same 1.6:1 aspect ratio, this resize does not itself crop the image.

### `sr_april.py`

Wraps the Student Robotics camera API.

The important architectural difference is that SR may already provide marker observations rather than requiring this stack to capture a raw frame and run `pupil_apriltags`.

This means SR/Webots can enter the architecture later in the pipeline without being forced through a physical-camera capture implementation.

---

## Camera resolution

`hw_io/cameras/resolve.py` selects the appropriate backend from the named camera profile.

Conceptually:

```text
robot profile
    |
    | logical camera
    | profile
    | device
    v
camera resolver
    |
    v
camera backend
```

The resolver should remain a construction layer rather than becoming vision logic itself.

---

# 3. Camera Processes and Multiple Cameras

The normal robot uses asynchronous camera processing.

Relevant files:

```text
hw_io/cameras/camera_process.py
hw_io/cameras/vision_worker.py
```

## `camera_process.py`

`CameraProcessManager` creates one worker process for each logical camera in:

```python
CONFIG.cameras
```

Conceptually:

```text
front camera  ---> worker process A
rear camera   ---> worker process B
```

This is an important part of the multi-camera architecture.

A slow camera/detector should not directly block the robot's main control loop.

---

## `vision_worker.py`

Each worker:

1. receives its logical camera name;
2. looks up the configured camera profile and physical device;
3. calls the camera resolver;
4. repeatedly calls the camera's vision interface;
5. publishes the latest result as a `vision_message`.

Conceptually:

```text
CONFIG.cameras["front"]
        |
        v
vision_worker
        |
        v
resolve_camera(...)
        |
        v
camera.see()
        |
        v
vision_message
```

The main process obtains the most recent result through the camera-process manager rather than performing image processing itself.

---

# 4. Level 2 — Vision Processing

Level 2 turns camera data into generic observations.

At present the main detector is AprilTag.

The direction of the architecture is that the capture backend and detector should remain logically separate so that a camera could later run:

- AprilTag detection;
- object detection;
- colour/blob processing;
- another fiducial detector;
- multiple processors.

---

## Shared AprilTag processor

The common AprilTag implementation is:

```text
perception/vision/apriltag_processor.py
```

It contains the common detector/data model used by physical camera backends.

Responsibilities include:

- RGB to grayscale conversion;
- `pupil_apriltags` detection;
- decision-margin filtering;
- tag-ID decoding;
- tag-size lookup;
- image-centre and corner extraction;
- horizontal and vertical angle calculation;
- current per-marker pose/distance calculation;
- conversion to the shared `Marker` representation.

The physical Pi and USB camera backends should not each contain independent copies of this logic.

Conceptually:

```text
Pi frame -------\
                 \
                  ---> AprilTagProcessor ---> Marker objects
                 /
USB frame ------/
```

---

## Marker data

The shared marker representation carries information such as:

```text
tag ID
distance
horizontal angle
vertical angle
yaw / pitch / roll
pixel centre
pixel corners
tag size
decision margin
per-tag translation
pose error
```

This is intentionally similar to the useful parts of the Student Robotics marker interface.

---

## Important PnP distinction

There are currently two different geometry roles which must not be confused.

### Per-marker geometry

The AprilTag processor can estimate:

```text
camera -> individual tag
```

This provides values useful for object approach, such as:

```text
distance
bearing
vertical angle
```

### Robot localisation

The localisation provider estimates:

```text
known arena tags -> camera pose -> robot pose
```

That is a separate higher-level PnP problem.

Different cameras or localisation methods may ultimately use different PnP algorithms.

Therefore:

> global robot localisation PnP does not belong in the camera backend.

---

# 5. Vision Message

The worker publishes a camera-specific `vision_message`.

Conceptually:

```python
{
    "camera": "front",
    "timestamp": ...,
    "markers": [...]
}
```

This is the bridge between:

```text
camera worker process
```

and:

```text
main robot process
```

The logical camera name stays attached to the message so downstream code knows which calibration and mount geometry apply.

---

# 6. Observation Adapter

For localisation, the raw marker objects are converted into plain AprilTag observations by:

```text
localisation/providers/vision/apriltag_observations.py
```

Its job is to map:

```text
vision_message
```

to:

```text
source_id + observations
```

using:

```python
CONFIG.vision_sources
```

Example:

```python
VISION_SOURCES = {
    "vision1": {
        "camera": "front",
        "provider": "apriltag_pnp",
        "enabled": True,
    }
}
```

The adapter passes information such as:

```text
source_id
camera
timestamp
tag_id
distance
angles
orientation
pixel centre
pixel corners
tag size
decision margin
pose values
```

This prevents the localisation provider from depending directly on a physical camera class.

---

# 7. Level 3A — Object Perception

One consumer of camera detections is the normal perception/object-tracking system.

Relevant code is under:

```text
perception/
```

and includes vision helpers such as:

```text
perception/vision/detection_pipeline.py
```

For object tags, the camera-relative marker measurement is converted into robot-useful object information.

For example:

```text
camera detects object tag
        |
        v
camera-relative distance/bearing
        |
        v
camera mount / correction
        |
        v
gripper-relative target
        |
        v
AcquireObject behaviour
```

This is why logs may contain both:

```text
cam_dist
```

and:

```text
grip_dist
```

The camera distance describes where the tag is relative to the camera.

The gripper distance describes where the object is relative to the robot's pickup geometry and is the value needed by the approach behaviour.

Object perception therefore consumes camera observations, but it does not own camera capture.

---

# 8. Level 3B — Robot Localisation

The other major consumer is localisation.

Relevant code is under:

```text
localisation/providers/vision/
```

The current architecture is:

```text
AprilTag observations
        |
        v
AprilTag PnP provider
        |
        v
PoseObservation
        |
        v
Localisation arbitration
        |
        v
robot pose
```

The provider uses known arena-tag geometry plus camera calibration to estimate the robot's absolute pose.

---

## Vision calibration

PnP calibration is assembled through:

```text
perception/vision/vision_calibration.py
```

It associates:

```text
vision source
    +
logical camera
    +
camera intrinsic calibration
    +
distortion coefficients
    +
camera-to-robot mount
```

The two calibration concepts are deliberately separate.

### Camera intrinsic calibration

Stored under:

```text
calibration/cameras/
```

Examples:

```text
pi3_fullfov_640_360.py
arducam_fullfov_640_400.py
```

Contains values such as:

```text
fx
fy
cx
cy
distortion coefficients
```

These describe the camera/lens/image configuration.

### Robot camera mount

Stored in the robot profile and resolved into robot calibration.

Example:

```python
CAMERA_MOUNTS = {
    "front": {
        "x_mm": ...,
        "y_mm": ...,
        "z_mm": ...,
        "roll_deg": ...,
        "pitch_deg": ...,
        "yaw_deg": ...,
    }
}
```

This describes where that logical camera is physically mounted on this robot.

Thus:

```text
camera calibration != camera mount calibration
```

A calibrated camera can be moved to another robot without changing its intrinsic calibration, but the mount transform must change.

---

# 9. Arena Geometry

Known arena-tag positions belong to configuration rather than to the camera or PnP provider.

The localisation provider uses arena geometry to associate:

```text
detected tag ID
```

with:

```text
known world position/orientation
```

The vision provider then solves for camera/robot pose from those known landmarks.

This keeps:

```text
camera hardware
```

separate from:

```text
competition arena geometry
```

---

# 10. Localisation Arbitration

Vision does not directly overwrite the robot pose merely because a solve was produced.

Instead:

```text
vision provider
    |
    v
pose evidence
    |
    v
localisation arbitration
```

The arbitration layer decides whether that evidence is suitable to accept.

This allows other localisation sources to coexist later, for example:

```text
commanded motion
encoders
OTOS
IMU
vision
```

The intended long-term system is therefore evidence-based rather than camera-owned.

---

# 11. Full Runtime Flow

For a physical camera running AprilTags:

```text
config/profiles/bob_bot.py
    |
    | CAMERAS
    | VISION_SOURCES
    | CAMERA_MOUNTS
    v
CameraProcessManager
    |
    v
vision_worker.py
    |
    v
hw_io/cameras/resolve.py
    |
    +-------------------------------+
    |                               |
    v                               v
pi_libcamera_april.py         opencv_usb.py
    |                               |
    +---------------+---------------+
                    |
                    v
       AprilTagProcessor
                    |
                    v
              Marker objects
                    |
                    v
             vision_message
                    |
          +---------+---------+
          |                   |
          v                   v
 object perception      AprilTag observation
 / approach             adapter
                              |
                              v
                       PnP localisation
                              |
                              v
                       PoseObservation
                              |
                              v
                    localisation arbitration
```

---

# 12. Student Robotics / Webots Flow

The Student Robotics API is a special case because its camera API can already provide marker observations.

Conceptually:

```text
SR / Webots camera API
        |
        v
SRAprilCamera
        |
        v
shared marker / observation interface
        |
        +----------------------+
        |                      |
        v                      v
object perception       localisation provider
```

This is intentional.

Webots should not require a fake OpenCV or fake Pi-camera backend when the Student Robotics simulation already provides the appropriate API.

---

# 13. Current Two-Camera Direction

## Final robot arrangement

The likely useful physical arrangement is:

```text
front camera  ---> forward field of view
rear camera   ---> rear field of view
```

Both feeds can contribute observations to the robot's world model/localisation system.

The robot should not fundamentally care which direction a camera faces; the camera mount transform provides that information.

A rear camera would normally have a yaw close to 180 degrees relative to the robot frame.

---

## Temporary comparison arrangement

For camera evaluation, both cameras can be mounted forward:

```text
Pi Camera 3  ---> same scene
Arducam      ---> same scene
```

but they should remain separate logical cameras:

```text
front_pi3
front_arducam
```

This allows direct comparison of:

- horizontal FOV;
- vertical FOV;
- full-sensor coverage;
- maximum reliable AprilTag range;
- minimum reliable AprilTag range;
- detection success rate;
- motion-blur sensitivity;
- bearing consistency;
- distance consistency;
- frame/update rate;
- lighting sensitivity.

No autonomous behaviour is required for this test.

In fact, camera comparison is better performed without autonomous movement so that both cameras can be evaluated against exactly the same stationary scene.

---

# 14. Multi-Camera Combination — Intended Direction

Two cameras observing the same robot environment should ultimately produce separate observations first.

Do **not** combine them at the camera-backend level.

Preferred architecture:

```text
front camera ---> observations ----\
                                     \
                                      ---> perception / localisation arbitration
                                     /
rear camera ----> observations ----/
```

This preserves:

- which camera generated each observation;
- each camera's calibration;
- each camera's mount transform;
- timestamps;
- independent confidence/quality information.

If both cameras see the same arena tag, localisation can decide how to use that evidence.

If both cameras see the same object, perception can decide whether to:

- select the newest observation;
- select the best-quality observation;
- prefer a designated camera;
- combine measurements;
- track the object in robot/world coordinates.

Those policies belong above Level 1.

---

# 15. Directory Responsibility Summary

| Directory / file | Primary responsibility |
|---|---|
| `config/profiles/<robot>.py` | Logical cameras, physical device assignments, vision sources, camera mounts |
| `config/cameras/*.py` | Reusable camera operating profiles |
| `calibration/cameras/*.py` | Camera intrinsics and distortion calibration |
| `hw_io/cameras/resolve.py` | Construct selected camera backend |
| `hw_io/cameras/pi_libcamera_april.py` | Pi Camera capture backend |
| `hw_io/cameras/opencv_usb.py` | USB/OpenCV capture backend |
| `hw_io/cameras/sr_april.py` | Student Robotics camera adapter |
| `hw_io/cameras/camera_process.py` | Manage one asynchronous worker per logical camera |
| `hw_io/cameras/vision_worker.py` | Run capture/detection loop and publish latest camera message |
| `perception/vision/apriltag_processor.py` | Shared AprilTag detection and per-marker geometry |
| `localisation/providers/vision/apriltag_observations.py` | Convert marker messages into generic localisation observations |
| `perception/vision/vision_calibration.py` | Assemble PnP calibration for a vision source/camera |
| `perception/vision/detection_pipeline.py` | Adapt/correct detections for robot/object perception |
| `localisation/providers/vision/pose_apriltag_pnp.py` | Produce robot-pose evidence from arena AprilTags |
| `localisation/` | Arbitrate and maintain robot pose |
| `behaviors/` | Consume perception/localisation results; never own camera hardware |

---

# 16. Architectural Rules

## Rule 1 — Logical camera is not camera model

```text
"front"
```

is a robot role.

```text
"arducam_fullfov_640_400"
```

is a camera profile.

```text
/dev/v4l/by-id/...
```

is a physical device.

Keep these separate.

---

## Rule 2 — Camera backend owns capture

The backend should answer:

> How do I get usable camera data from this hardware?

It should not decide:

> What should the robot do with this object?

---

## Rule 3 — Detector owns detection

The detector/processor should answer:

> What is visible in this image?

It should not decide:

> Is this localisation trustworthy?

---

## Rule 4 — Observation adapters isolate consumers

Localisation and perception should consume generic observations rather than depend on:

```text
Picamera2
OpenCV VideoCapture
V4L2
USB paths
```

---

## Rule 5 — Calibration follows the data source

Every observation must retain its camera identity so the correct:

```text
intrinsics
distortion
mount transform
```

can be applied.

---

## Rule 6 — Multi-camera fusion happens above capture

Each camera should remain independently measurable and debuggable.

Combination/fusion should happen in perception or localisation.

---

## Rule 7 — PnP is pluggable

There is no requirement that every camera use the same pose-solving algorithm.

A camera/detector combination may support:

- simple single-tag geometry;
- multi-tag PnP;
- an advanced ambiguity-aware solver;
- no PnP at all.

The higher-level provider should own that choice.

---

# 17. Current Development Status

## Working

- Pi Camera 3 capture;
- Arducam OV9281 USB capture;
- full-aspect 1280×800 to 640×400 Arducam processing path;
- shared AprilTag processing;
- asynchronous camera worker;
- object-tag distance/bearing;
- object approach using gripper-relative correction;
- camera calibration profiles;
- camera mount calibration;
- arena-tag observation adapter;
- AprilTag PnP localisation provider architecture;
- Student Robotics camera adapter path;
- architecture capable of starting more than one logical camera worker.

## Still to refine

- simultaneous two-camera runtime configuration;
- camera-to-camera comparison diagnostics;
- explicit multi-camera observation arbitration/fusion;
- rear-camera deployment;
- full-FOV empirical comparison;
- robust long-range AprilTag comparison;
- distortion handling consistency in all per-marker geometry;
- detector independence for future non-AprilTag processing;
- final localisation-provider/PnP algorithm selection.

---

# 18. Immediate Two-Camera Test Goal

Before changing autonomous behaviour, test the Pi Camera 3 and Arducam side-by-side while both face forward.

The test should:

1. open both cameras;
2. avoid running autonomous motion;
3. display or save frames from both cameras;
4. identify each image by logical camera name;
5. report AprilTag detections independently;
6. report tag ID, distance, bearing and decision margin;
7. allow direct FOV comparison;
8. allow stationary range tests;
9. later allow controlled manual movement/motion-blur tests.

This should be implemented as a diagnostic/test path rather than by running the normal autonomous state machine.

---

# Summary

The vision stack follows a three-level model:

```text
CAMERA / CAPTURE
        |
        v
VISION PROCESSING / OBSERVATIONS
        |
        v
ROBOT INTERPRETATION
(object perception or localisation)
```

The architecture deliberately keeps:

```text
logical camera
camera profile
physical device
camera calibration
robot mount
detector
localisation provider
behaviour
```

as separate concepts.

That is what allows the same software stack to scale from one Pi camera to a forward/rear multi-camera robot, while still supporting USB cameras, Student Robotics hardware, Webots, different detector types, and alternative localisation algorithms.
