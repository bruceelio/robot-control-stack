# Planned File Hierarchy

The Perception package is expected to grow beyond the current AprilTag-based object
interpretation.

The intended structure separates:

- generic object and obstacle interpretation;
- the sensor modality which produced the evidence;
- tracking and fusion which may combine multiple modalities.

The hierarchy below is a **future architecture**, not a requirement to create empty
files or directories before they are needed.

perception/
├── README.md
│
├── objects/
│   ├── observations.py
│   ├── classification.py
│   ├── geometry.py
│   ├── tracking.py
│   └── target_lock.py
│
├── obstacles/
│   ├── observations.py
│   ├── tracking.py
│   └── fusion.py
│
├── dynamic_objects/                    # NEW
│   ├── observations.py
│   ├── tracking.py
│   ├── velocity_estimation.py
│   └── prediction.py
│
├── scene/                              # NEW / IMPORTANT
│   ├── local_scene.py
│   ├── free_space.py
│   ├── drivable_area.py
│   ├── occupancy.py
│   └── boundaries.py
│
├── vision/
│   ├── README.md
│   │
│   ├── objects/
│   │   ├── detector.py
│   │   ├── tagged_objects.py
│   │   ├── colour.py
│   │   ├── shape.py
│   │   └── geometry.py
│   │
│   ├── obstacles/
│   │   ├── detector.py
│   │   └── free_space.py
│   │
│   ├── track/                          # NEW
│   │   ├── boundary_detection.py
│   │   ├── drivable_area.py
│   │   ├── lane_detection.py
│   │   └── cone_detection.py
│   │
│   ├── depth/
│   │   ├── observations.py
│   │   └── range_from_depth.py
│   │
│   ├── motion/
│   │   ├── optical_flow.py
│   │   └── moving_objects.py           # NEW
│   │
│   ├── segmentation/                   # NEW
│   │   ├── semantic.py
│   │   └── instance.py
│   │
│   └── scene/
│       └── scene_model.py
│
├── lidar/                              # BIG NEW SECTION
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
├── pointcloud/                         # FUTURE 3-D/generalised LiDAR
│   ├── observations.py
│   ├── preprocessing.py
│   ├── clustering.py
│   ├── segmentation.py
│   └── obstacles.py
│
├── range/
│   ├── ultrasonic.py
│   └── tof.py
│
├── fusion/
│   ├── object_fusion.py
│   ├── obstacle_fusion.py
│   ├── track_fusion.py                 # NEW
│   ├── scene_fusion.py                 # NEW
│   └── association.py                  # NEW
│
└── quality/                            # FUTURE
    ├── confidence.py
    └── observation_health.py



## Generic Perception

The top-level object and obstacle packages represent robot-useful interpretations
which should not depend on a particular sensor.

For example:

```text
ObjectObservation
ObstacleObservation
ObjectTrack
ObstacleTrack
```

A behaviour or navigation component should ideally consume these representations
rather than depending directly on a camera, ultrasonic sensor, ToF sensor or detector.

---

## Visual Perception

```text
perception/vision/
```

contains interpretation which specifically depends on visual evidence.

It may eventually include:

```text
object detection
tagged-object interpretation
colour classification
shape classification
visual object geometry
obstacle detection
free-space estimation
depth interpretation
visual motion
scene interpretation
```

Visual Perception is therefore not limited to AprilTags.

The distinction is:

```text
vision/
    What did the camera or detector observe?

perception/vision/
    What does the visual evidence mean?
```

For example:

```text
camera / detector
        |
        v
vision/
    neutral observation
        |
        v
perception/vision/objects/
    object interpretation
        |
        v
perception/objects/
    object observation / track
        |
        v
behaviour
```

---

## AprilTags

Neutral AprilTag observations remain under:

```text
vision/apriltag/
```

because an AprilTag measurement may be consumed independently by both Perception and
Localisation.

Perception may interpret a tag as a game object:

```text
AprilTagObservation
        |
        v
perception/vision/objects/tagged_objects.py
        |
        v
ObjectObservation
```

Localisation may independently use the same neutral AprilTag observation to estimate
robot pose.

Therefore AprilTag detection and reconciliation must not become owned exclusively by
object Perception.

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

Competition-specific behaviours may decide which object to pursue, but the underlying
object representation should remain reusable.

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
depth camera
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
```

This provides the Perception boundary required by both reactive Stage 2 obstacle
avoidance and localisation-aware Stage 3 navigation.

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

Range Perception should not contain navigation or avoidance policy.

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
front camera + rear camera object observations
multiple obstacle observations describing the same physical object
```

Fusion should preserve:

```text
timestamp
source / provenance
confidence
geometry
identity where known
```

It should not decide what manoeuvre the robot performs.

---

## Future Vision Capabilities

The planned hierarchy intentionally allows visual perception to grow beyond the
current AprilTag-oriented implementation.

Possible future capabilities include:

```text
direct object detection
colour recognition
shape recognition
semantic segmentation
free-space detection
depth perception
optical flow
moving-object detection
robot detection
scene interpretation
```

These capabilities should be added only when required.

The presence of a future location in this hierarchy does **not** imply that empty
modules should be created in advance.

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
        +---- tracks / fused observations
        |
        v
BEHAVIOURS / NAVIGATION
```

The corresponding ownership rule is:

> **Vision reports observations. Perception assigns robot-useful meaning. Localisation
> estimates robot pose. Behaviours and Navigation decide what to do.**