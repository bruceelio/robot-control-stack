# Vision File Hierarchy

The `vision/` package owns camera-image processing and camera geometry.

Its job is to convert image pixels into **neutral visual measurements** which may then be
consumed by Perception, Localisation, diagnostics, or other higher-level systems.

The hierarchy below is a **future architecture**, not a requirement to create empty files
or directories before they are needed.

The core ownership rule is:

> **Vision reports what the image contains. It does not decide what the robot should do,
> and it does not assign competition-specific meaning unless that meaning is intrinsic
> to the visual measurement itself.**

For example, Vision may report:

```text
AprilTagDetection
ObjectDetection2D
LaneMarkingDetection
SegmentationMask
OpticalFlowField
DepthObservation
```

Perception may then interpret those measurements as:

```text
game object
opponent robot
dynamic obstacle
track boundary
drivable area
local scene
```

---

## Planned File Hierarchy

```text
vision/
├── __init__.py
├── README.md
├── README_VISION_FILE_HIERARCHY.md
├── calibration.py
│
├── preprocessing/
│   ├── __init__.py
│   ├── colour.py
│   ├── normalize.py
│   ├── resize.py
│   └── undistort.py
│
├── geometry/
│   ├── __init__.py
│   ├── camera_model.py
│   ├── ground_plane.py
│   ├── homography.py
│   ├── pixel_bearing.py
│   ├── pnp.py
│   └── projection.py
│
├── apriltag/
│   ├── __init__.py
│   ├── models.py
│   └── processor.py
│
├── object_detection/
│   ├── __init__.py
│   ├── base.py
│   ├── models.py
│   ├── postprocess.py
│   └── yolo.py
│
├── lane_detection/
│   ├── __init__.py
│   ├── colour_threshold.py
│   ├── edge_detection.py
│   ├── lane_marking_detector.py
│   ├── models.py
│   ├── perspective_transform.py
│   └── polynomial_fit.py
│
├── segmentation/
│   ├── __init__.py
│   ├── drivable_area_segmentation.py
│   ├── models.py
│   └── semantic_segmentation.py
│
├── optical_flow/
│   ├── __init__.py
│   ├── dense_flow.py
│   ├── models.py
│   └── sparse_lk.py
│
├── feature_tracking/
│   ├── __init__.py
│   ├── feature_detector.py
│   ├── feature_tracker.py
│   └── models.py
│
└── depth/
    ├── __init__.py
    ├── models.py
    ├── monocular.py
    └── stereo.py
```

---

## Package Boundary

The distinction between `vision/` and `perception/vision/` is intentional.

```text
vision/
    What did the image contain?

perception/vision/
    What does that visual evidence mean to the robot?
```

The normal data flow is:

```text
camera
  |
  v
image frame
  |
  +--------------------+
  |                    |
  v                    v
AprilTag detector   object detector
  |                    |
  v                    v
neutral visual observations
  |
  v
perception/vision/
  |
  v
robot-useful interpretations
```

This allows the same Vision implementation to be used with:

```text
Webots simulated camera
Pi Camera
USB webcam
global-shutter camera
future stereo/depth cameras
```

provided that camera acquisition produces a compatible image frame.

---

## Frame Processing

Vision algorithms should preferably operate on a supplied image frame rather than
capture from a camera themselves.

Conceptually:

```text
camera.capture()
      |
      v
    frame
      |
      +----------------------+
      |                      |
      v                      v
AprilTag processing     object detection
      |
      +----------------------+
      |
      v
other visual algorithms
```

This permits multiple algorithms to consume the **same frame and timestamp**.

It avoids:

```text
duplicate camera capture
inconsistent timestamps
different exposure states
unnecessary camera I/O
```

Frame acquisition itself remains below the algorithmic Vision layer.

---

## Calibration

```text
vision/calibration.py
```

owns generic camera calibration data and calibration-related helpers.

Camera-specific calibration profiles remain configuration/data rather than being embedded
inside individual detection algorithms.

Typical calibration values include:

```text
fx
fy
cx
cy
distortion coefficients
image width / height
field of view
```

Calibration is shared by AprilTag pose estimation, pixel-bearing conversion, object
geometry, ground-plane projection, lane geometry and other camera-based algorithms.

---

## Preprocessing

```text
vision/preprocessing/
```

contains generic image transformations that do not themselves assign semantic meaning.

Examples include:

```text
undistortion
colour-space conversion
normalisation
resizing
cropping
```

Preprocessing should remain optional and composable rather than becoming one mandatory
pipeline for all cameras and detectors.

---

## Camera Geometry

```text
vision/geometry/
```

contains camera-model and projection mathematics.

Expected responsibilities include:

```text
pixel -> bearing
pixel -> ground-plane ray
camera projection
PnP helpers
homography
ground-plane intersection
```

These operations are neutral geometric transformations.

They may be consumed by:

```text
AprilTag processing
object detection adapters
lane detection
Localisation
visual servoing
diagnostics
```

---

## AprilTags

```text
vision/apriltag/
```

owns neutral AprilTag image processing.

The intended flow is:

```text
RGB image
   |
   v
vision/apriltag/processor.py
   |
   v
AprilTagDetection
```

A neutral AprilTag observation may contain:

```text
tag id
pixel corners
pixel centre
camera-relative pose
range
bearing
timestamp
source
```

The interpretation of that tag belongs elsewhere.

For example:

```text
AprilTagDetection
       |
       +--------------------> Localisation
       |
       v
perception/vision/apriltag_adapter.py
       |
       v
game-object interpretation
```

AprilTag processing therefore must not become owned exclusively by object Perception.

---

## Object Detection

```text
vision/object_detection/
```

owns generic image-based object detectors.

The detector should answer questions such as:

```text
what class appears in this image?
where is it in the image?
how confident is the detector?
```

A generic output might be:

```text
ObjectDetection2D
    class_id
    class_name
    confidence
    bounding_box
    timestamp
    source
```

The detector should not decide:

```text
whether the object is an obstacle
whether it is an opponent
whether it should be avoided
whether it should be pursued
whether it is inside a home base
```

Those are Perception or higher-level responsibilities.

`base.py` is reserved for a detector interface so different implementations may be
substituted without changing higher-level Perception code.

Possible implementations include:

```text
YOLO
TensorFlow Lite detector
OpenCV DNN detector
future hardware-accelerated detector
```

---

## Lane and Track-Marking Detection

```text
vision/lane_detection/
```

owns image-level detection of lane lines, track markings and visible boundaries.

Possible approaches include:

```text
colour thresholding
edge detection
Hough-style line detection
perspective transformation
polynomial lane fitting
learned lane detection
```

The output should remain a neutral visual measurement, for example:

```text
LaneMarkingDetection
BoundaryPixelCurve
LanePolynomial
```

Persistent lane state, track width, centreline estimation and temporal confidence belong
in Perception.

This distinction allows F1TENTH-style track perception without moving planning or control
into Vision.

---

## Segmentation

```text
vision/segmentation/
```

is reserved for pixel-level classification.

Possible outputs include:

```text
semantic segmentation mask
drivable-area mask
instance mask
```

Segmentation reports image evidence.

The conversion of a mask into obstacle geometry, free space or a scene model belongs in
Perception.

---

## Optical Flow

```text
vision/optical_flow/
```

contains visual motion estimation between images.

Possible implementations include:

```text
sparse Lucas-Kanade flow
dense optical flow
future learned optical flow
```

Optical flow may support:

```text
moving-object detection
feature tracking
visual odometry
dynamic-scene analysis
```

However, robot pose estimation from optical flow belongs in `localisation/`, not in
Vision.

Vision provides the visual motion measurement.

---

## Feature Tracking

```text
vision/feature_tracking/
```

contains generic image-feature detection and temporal feature correspondence.

Typical outputs are neutral image-space tracks such as:

```text
feature id
pixel position
previous pixel position
track age
confidence
```

Possible consumers include:

```text
Localisation / visual odometry
motion estimation
scene analysis
camera diagnostics
```

---

## Depth

```text
vision/depth/
```

contains visual depth-estimation algorithms.

Potential sources include:

```text
stereo cameras
monocular depth estimation
depth cameras
```

The output should be a visual depth observation or depth map.

The conversion of depth into obstacles, free space or fused scene geometry belongs in
Perception.

---

## Relationship to Localisation

Vision may provide measurements used by Localisation, but Localisation owns estimation of
the robot's own pose.

Therefore the following should **not** live in `vision/`:

```text
SLAM
visual odometry state estimation
visual-inertial odometry
particle filtering
pose filtering
map-relative robot pose
```

The relationship is:

```text
vision/
   |
   v
neutral visual measurement
   |
   v
localisation/
   |
   v
robot pose estimate
```

---

## Relationship to Navigation

Navigation should not consume raw pixels where a reusable Vision or Perception
representation exists.

The following remain Navigation responsibilities:

```text
Follow-the-Gap
Vector Field Histogram
Dynamic Window
Velocity Obstacles
Pure Pursuit
wall following
path planning
lane selection
trajectory tracking
```

Vision may supply evidence required by those algorithms, but it does not choose the
robot's motion.

---

## Current Development Path

The immediate SR / multi-robot path is intentionally small:

```text
camera frame
    |
    +----------------------+
    |                      |
    v                      v
AprilTag processor     object detector
    |                      |
    v                      v
AprilTagDetection      ObjectDetection2D
    |                      |
    +----------+-----------+
               |
               v
          Perception
```

The future F1TENTH-oriented branches exist in the hierarchy so the package does not need
to be reorganised later.

They should be implemented only when needed.

---

## Architectural Rule

The long-term data flow is:

```text
HARDWARE / SIMULATION CAMERA
            |
            v
           FRAME
            |
            v
          VISION
            |
            +------------------> LOCALISATION
            |
            v
        PERCEPTION
            |
            v
BEHAVIOURS / NAVIGATION
```

The corresponding ownership rule is:

> **Vision converts images into neutral measurements. Perception assigns robot-useful
> meaning. Localisation estimates robot pose. Behaviours and Navigation decide what to
> do.**
