# Perception File Hierarchy

The Perception package is expected to grow beyond the current AprilTag-based object
interpretation.

Its purpose is to convert sensor observations into **robot-useful interpretations of the
surrounding world** while remaining independent of navigation and behaviour policy.

The intended structure separates:

- generic object and obstacle interpretation;
- dynamic-object tracking and prediction;
- track/lane interpretation;
- the sensor modality which produced the evidence;
- scene representations;
- tracking, fusion and observation quality.

The hierarchy below is a **future architecture**, not a requirement to create empty files
or directories before they are needed.

---

## Planned File Hierarchy

```text
perception/
├── __init__.py
├── perception.py
├── README.md
├── README_PERCEPTION_FILE_HIERARCHY.md
│
├── objects/
│   ├── __init__.py
│   ├── observations.py
│   ├── classification.py
│   ├── geometry.py
│   ├── tracking.py
│   └── target_lock.py
│
├── obstacles/
│   ├── __init__.py
│   ├── observations.py
│   ├── tracking.py
│   └── fusion.py
│
├── dynamic_objects/
│   ├── __init__.py
│   ├── observations.py
│   ├── tracking.py
│   ├── velocity_estimation.py
│   └── prediction.py
│
├── track/
│   ├── __init__.py
│   ├── observations.py
│   ├── boundaries.py
│   ├── centreline.py
│   ├── lane_tracking.py
│   └── track_model.py
│
├── scene/
│   ├── __init__.py
│   ├── local_scene.py
│   ├── free_space.py
│   ├── drivable_area.py
│   ├── occupancy.py
│   └── boundaries.py
│
├── vision/
│   ├── __init__.py
│   ├── README.md
│   ├── frame_pipeline.py
│   ├── apriltag_adapter.py
│   ├── object_detection_adapter.py
│   ├── lane_detection_adapter.py
│   └── depth_adapter.py
│
├── lidar/
│   ├── __init__.py
│   ├── README.md
│   ├── observations.py
│   ├── preprocessing.py
│   ├── clustering.py
│   ├── geometry.py
│   ├── obstacles.py
│   ├── free_space.py
│   ├── gaps.py
│   └── boundaries.py
│
├── pointcloud/
│   ├── __init__.py
│   ├── observations.py
│   ├── preprocessing.py
│   ├── clustering.py
│   ├── segmentation.py
│   └── obstacles.py
│
├── range/
│   ├── __init__.py
│   ├── ultrasonic.py
│   └── tof.py
│
├── tracking/
│   ├── __init__.py
│   ├── association.py
│   ├── kalman_filter.py
│   └── models.py
│
├── fusion/
│   ├── __init__.py
│   ├── object_fusion.py
│   ├── obstacle_fusion.py
│   ├── track_fusion.py
│   ├── scene_fusion.py
│   └── association.py
│
└── quality/
    ├── __init__.py
    ├── confidence.py
    └── observation_health.py
```

---

## Generic Perception

The top-level Perception packages represent robot-useful interpretations which should not
depend on a particular sensor.

Examples include:

```text
ObjectObservation
ObstacleObservation
DynamicObjectObservation
ObjectTrack
ObstacleTrack
DynamicObjectTrack
TrackModel
LocalScene
```

A behaviour or Navigation component should ideally consume these representations rather
than depending directly on:

```text
camera
AprilTag detector
YOLO detector
ultrasonic sensor
ToF sensor
LiDAR
depth camera
```

This keeps downstream autonomy independent of the hardware or detector which produced the
evidence.

---

## Vision / Perception Boundary

The distinction between top-level `vision/` and `perception/vision/` is fundamental.

```text
vision/
    What did the image or visual detector observe?

perception/vision/
    What does that visual evidence mean to the robot?
```

Pixel-level and detector algorithms therefore belong under top-level `vision/`:

```text
vision/apriltag/
vision/object_detection/
vision/lane_detection/
vision/segmentation/
vision/optical_flow/
vision/depth/
```

`perception/vision/` owns the adapters which turn those neutral outputs into generic
Perception representations:

```text
perception/vision/
├── frame_pipeline.py
├── apriltag_adapter.py
├── object_detection_adapter.py
├── lane_detection_adapter.py
└── depth_adapter.py
```

This prevents Perception from becoming a second image-processing package.

---

## Frame Pipeline

```text
perception/vision/frame_pipeline.py
```

coordinates consumption of a common camera frame.

The intended architecture is:

```text
camera.capture()
      |
      v
    frame
      |
      +--------------------------+
      |                          |
      v                          v
vision/apriltag/          vision/object_detection/
      |                          |
      v                          v
neutral detections
      |
      v
perception/vision/ adapters
```

Where possible, AprilTag processing and object detection should consume the **same captured
frame and timestamp**.

This supports both:

```text
Webots simulation
real Bobbot camera hardware
```

without changing higher-level Perception ownership.

---

## AprilTags

Neutral AprilTag detection remains under:

```text
vision/apriltag/
```

because an AprilTag measurement may be consumed independently by both Perception and
Localisation.

For example:

```text
AprilTagDetection
        |
        +-----------------------> Localisation
        |
        v
perception/vision/apriltag_adapter.py
        |
        v
ObjectObservation
```

Perception may interpret a tag as a game object.

Localisation may independently use the same neutral AprilTag observation to estimate
robot pose.

Therefore AprilTag detection must not become owned exclusively by object Perception.

---

## Object Perception

```text
perception/objects/
```

owns sensor-independent object meaning and state.

Expected responsibilities include:

```text
object classification
robot-relative object geometry
persistent object identity
visibility state
target locking
object tracking
```

Evidence may originate from:

```text
AprilTags
direct object detection
colour recognition
shape recognition
depth
future sensors
```

Competition-specific behaviours may decide which object to pursue, but the underlying
object representation should remain reusable.

For example:

```text
vision/object_detection/
        |
        v
ObjectDetection2D
        |
        v
perception/vision/object_detection_adapter.py
        |
        v
ObjectObservation
        |
        v
perception/objects/
```

---

## Obstacle Perception

```text
perception/obstacles/
```

owns sensor-independent obstacle observations and tracks.

Obstacle evidence may originate from:

```text
camera
ultrasonic
ToF
LiDAR
depth camera
segmentation
future range sensors
```

The final obstacle representation should not require Navigation to know which sensor
produced it.

Conceptually:

```text
camera --------\
                \
ultrasonic ------> Perception ---> ObstacleObservation / ObstacleTrack
                /
ToF ------------/
                \
LiDAR -----------/
```

This provides the Perception boundary required by both:

```text
Stage 2 reactive obstacle avoidance
Stage 3 localisation-aware navigation
future F1TENTH-style local planning
```

---

## Dynamic Objects

```text
perception/dynamic_objects/
```

owns objects whose motion through the environment matters.

This includes the multi-robot use case which motivates current object-detection work.

Typical responsibilities include:

```text
dynamic-object observations
persistent identity
temporal tracking
relative velocity estimation
short-horizon prediction
```

Conceptually:

```text
object detector
      |
      v
ObjectDetection2D
      |
      v
object_detection_adapter
      |
      v
DynamicObjectObservation
      |
      v
tracking
      |
      v
velocity_estimation
      |
      v
prediction
```

A tracked opponent robot might ultimately expose:

```text
relative position
relative bearing
relative velocity
collision radius
confidence
timestamp
```

Navigation may then use that state for algorithms such as Velocity Obstacles.

The VO algorithm itself remains under Navigation.

---

## Shared Tracking

```text
perception/tracking/
```

contains reusable temporal-estimation mechanisms which are not specific to one semantic
object type.

Examples include:

```text
data association
Kalman filtering
generic track state
track lifecycle helpers
```

Semantic packages such as:

```text
objects/
obstacles/
dynamic_objects/
```

may use these shared mechanisms while retaining ownership of their domain-specific track
representations.

This avoids duplicating generic tracking mathematics without collapsing all perception
state into one undifferentiated tracker.

---

## Track and Lane Perception

```text
perception/track/
```

owns persistent interpretation of lanes, circuit boundaries and drivable track geometry.

Pixel-level lane or boundary detection belongs in:

```text
vision/lane_detection/
```

Perception then turns those measurements into robot-useful track state:

```text
visible lane detections
        |
        v
track observations
        |
        v
temporal lane tracking
        |
        +---- left / right boundaries
        |
        +---- centreline
        |
        +---- track width
        |
        +---- confidence
        |
        v
TrackModel
```

Expected responsibilities include:

```text
lane persistence
boundary persistence
centreline estimation
track-width estimation
confidence over time
short-term recovery through missing detections
```

Navigation decides how to drive using that model.

Therefore the following do **not** belong in Perception:

```text
lane switching policy
Pure Pursuit
Stanley control
MPC
Follow-the-Gap
trajectory tracking
```

---

## Scene Perception

```text
perception/scene/
```

combines local spatial interpretations into a scene representation.

Possible outputs include:

```text
LocalScene
FreeSpace
DrivableArea
OccupancyRepresentation
BoundaryModel
```

This package may consume observations from several modalities.

For example:

```text
object observations
obstacle observations
LiDAR geometry
lane / track boundaries
depth
segmentation
        |
        v
   local_scene.py
```

The scene model describes the environment.

It does not choose navigation actions.

---

## LiDAR Perception

```text
perception/lidar/
```

interprets 2-D LiDAR scans.

Possible responsibilities include:

```text
scan preprocessing
range validation
clustering
line extraction / geometry
obstacle extraction
free-space extraction
gap extraction
boundary extraction
```

The distinction from Navigation remains important.

For example:

```text
perception/lidar/gaps.py
    identifies free-space gaps

navigation/local_planning/follow_the_gap.py
    chooses how to drive using those gaps
```

Likewise:

```text
perception/lidar/obstacles.py
    reports obstacle geometry

navigation/local_planning/velocity_obstacle.py
    decides which velocities are safe
```

---

## Point Clouds

```text
perception/pointcloud/
```

is reserved for future 3-D perception.

Possible sources include:

```text
3-D LiDAR
depth camera
stereo reconstruction
future fused depth sources
```

Potential responsibilities include:

```text
point-cloud preprocessing
clustering
segmentation
obstacle extraction
```

This remains separate from 2-D LiDAR so the simpler scan-based path does not need to adopt
3-D abstractions prematurely.

---

## Range Perception

```text
perception/range/
```

interprets range-sensor measurements such as ultrasonic and ToF observations.

Hardware access remains below Perception.

Therefore:

```text
hw_io sensor
      |
      v
raw range measurement
      |
      v
perception/range/
      |
      v
ObstacleObservation
```

Range Perception may handle:

```text
validity
sensor geometry
confidence
temporal interpretation
conversion to obstacle evidence
```

It should not contain navigation or avoidance policy.

---

## Fusion

```text
perception/fusion/
```

is reserved for combining evidence which may come from different modalities.

Examples include:

```text
visual object + depth range
visual obstacle + ultrasonic range
camera object + LiDAR cluster
front camera + rear camera object observations
multiple obstacle observations describing the same physical object
lane evidence from camera + LiDAR boundaries
```

Fusion should preserve:

```text
timestamp
source / provenance
confidence
geometry
identity where known
```

Possible responsibilities are separated by output domain:

```text
object_fusion.py
obstacle_fusion.py
track_fusion.py
scene_fusion.py
association.py
```

Fusion should not decide what manoeuvre the robot performs.

---

## Observation Quality

```text
perception/quality/
```

contains cross-cutting perception-quality mechanisms.

Possible responsibilities include:

```text
confidence normalisation
observation health
source degradation
staleness
consistency checks
```

This is intentionally separate from Navigation.

A perception component may say:

```text
this observation is stale
this source is unreliable
this track confidence is falling
```

Navigation then decides how that affects robot motion.

---

## Relationship to Localisation

Perception describes the world around the robot.

Localisation estimates the robot's own pose.

The two may exchange neutral observations, but they remain separate responsibilities.

For example:

```text
vision/apriltag/
      |
      +-----------------------> localisation/
      |
      v
perception/
```

The following remain Localisation responsibilities:

```text
SLAM
particle filters
scan matching for robot pose
visual odometry state estimation
visual-inertial odometry
map-relative robot pose estimation
```

Perception may still use the current localisation estimate to express observations in a
map or arena frame where appropriate.

---

## Relationship to Navigation

Navigation should consume Perception outputs rather than raw sensor implementations where
a reusable Perception representation exists.

Examples:

```text
Perception output                    Navigation consumer

ObstacleObservation      ------->    local avoidance
DynamicObjectTrack       ------->    velocity obstacle
FreeSpace                ------->    local planning
TrackModel               ------->    lane / path planning
OccupancyRepresentation  ------->    path planning
```

Navigation retains ownership of:

```text
Follow-the-Gap
Vector Field Histogram
Dynamic Window
Velocity Obstacles
wall following
lane selection
Pure Pursuit
path planning
trajectory tracking
```

Perception describes.

Navigation decides.

---

## Current SR Development Path

The immediate vision-object-detection path is:

```text
camera
  |
  v
shared frame
  |
  +---------------------------+
  |                           |
  v                           v
AprilTag detection      object detection
  |                           |
  v                           v
AprilTagDetection       ObjectDetection2D
  |                           |
  v                           v
apriltag_adapter        object_detection_adapter
  |                           |
  +-------------+-------------+
                |
                v
           Perception
                |
                +---- game objects
                |
                +---- dynamic robots
                |
                +---- obstacles
```

The first dynamic-object use case is other robots in multi-robot simulation and later IRL.

The hierarchy also reserves locations for future:

```text
lane / track perception
LiDAR
point clouds
segmentation
depth
multi-sensor fusion
F1TENTH-style scene perception
```

These capabilities should be added only when required.

The presence of a future location in this hierarchy does **not** imply that empty modules
should be created in advance.

---

## Architectural Rule

The long-term data flow is:

```text
HARDWARE / SIMULATION
        |
        v
      VISION
        |
        +--------------------> LOCALISATION
        |
        v
    PERCEPTION
        |
        +---- objects
        |
        +---- obstacles
        |
        +---- dynamic objects
        |
        +---- track / lanes
        |
        +---- scene
        |
        +---- fused observations
        |
        v
BEHAVIOURS / NAVIGATION
```

The corresponding ownership rule is:

> **Vision reports neutral observations. Perception assigns robot-useful meaning and
> maintains world state. Localisation estimates robot pose. Behaviours and Navigation
> decide what to do.**
