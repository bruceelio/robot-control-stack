Here is the cleaned-up procedure, written to work alongside the final table.

# OV9281 Camera Setup, Calibration and OpenCV Transfer Procedure

## Purpose

This procedure defines how to take a new Arducam OV9281 camera from initial setup through PhotonVision testing, camera calibration, configuration selection and final transfer into the Raspberry Pi Picamera2/OpenCV pipeline.

The accompanying configuration table remains the detailed working reference for individual settings. This document defines the order in which the work should be completed.

The procedure uses three image modes:

| Resolution   | Purpose                                                          |
| ------------ | ---------------------------------------------------------------- |
| `1280 × 800` | Authoritative full-field PhotonVision setup and calibration mode |
| `640 × 480`  | Fast PhotonVision tuning and comparative test mode               |
| `640 × 400`  | Final full-field Picamera2/OpenCV processing mode                |

---

# 1. Prepare the Camera

## 1.1 Inspect the Camera

Before connecting the camera:

* Check the lens and sensor board for damage.
* Confirm that the ribbon cable is correctly seated.
* Confirm that the cable contacts face the correct direction at both ends.
* Check that the lens is secure but still adjustable if manual focus is required.
* Record an identifier for the camera so that its calibration can be stored separately from other cameras.

Do not assume that two cameras of the same model can share the same calibration.

---

## 1.2 Connect the Camera

Connect the OV9281 to the Raspberry Pi CSI camera connector.

Power off the Raspberry Pi before inserting or removing the ribbon cable.

After connecting the camera, boot the Raspberry Pi and confirm that the camera is detected.

Useful checks include:

```bash
rpicam-hello --list-cameras
```

or:

```bash
libcamera-hello --list-cameras
```

The exact command depends on the Raspberry Pi OS version.

Confirm that the OV9281 appears with a native sensor mode of approximately:

```text
1280 × 800
```

---

# 2. Confirm the Full-Sensor Image

The OV9281 native image has an aspect ratio of:

```text
1280 × 800
```

which is:

```text
8:5
```

The final OpenCV image should preserve this aspect ratio:

```text
640 × 400
```

The intended relationship is:

```text
1280 × 800
      ↓ uniform 0.5 scale
640 × 400
```

This is important because it preserves the complete sensor field of view.

The final Picamera2 configuration should therefore use the full sensor mode as its source:

```python
SENSOR_OUTPUT_SIZE = (1280, 800)
FORCE_FULL_SENSOR_SCALER_CROP = True
```

The final processing stream will be:

```python
main = {
    "size": (640, 400),
}
```

At this stage, confirm visually that the `1280 × 800` image shows the complete expected field of view.

Do not proceed to calibration until full-sensor operation has been confirmed.

---

# 3. Mount the Camera in Its Final Configuration

Camera calibration belongs to the combination of:

* the physical camera;
* the lens;
* the focus position;
* the sensor mode;
* the image crop;
* the mounting orientation.

Before calibration, place the camera in the same physical condition in which it will be used on the robot.

Confirm:

* camera orientation;
* lens selection;
* lens tightness;
* focus position;
* camera bracket;
* camera angle;
* expected operating height;
* any protective cover or lens hood.

If the lens, focus or sensor crop changes later, repeat the calibration.

---

# 4. Establish the Test Environment

Set up a repeatable test area.

Include:

* stable lighting;
* an AprilTag at known distances;
* a measuring tape or marked floor;
* test positions at the image centre;
* test positions near each edge;
* test positions near each corner;
* a rigid ChArUco calibration board;
* a flat support for the calibration board.

Avoid:

* changing sunlight;
* reflective surfaces;
* moving shadows;
* flickering lights;
* a bent calibration board;
* motion blur during calibration capture.

The lighting used during camera setup should be reasonably representative of the final robot environment.

---

# 5. Set the Lens Focus

The OV9281 uses a manually focused lens unless supplied with a fixed-focus configuration.

Use PhotonVision at:

```text
1280 × 800
```

Place an AprilTag or high-contrast target at a representative operating distance.

Adjust the lens until:

* tag edges are sharp;
* the black and white boundaries are clearly separated;
* corner detail is visible;
* the image remains acceptably sharp over the intended distance range.

Do not optimize focus only for a very close target unless that is the intended operating distance.

After setting focus:

* secure the lens if appropriate;
* mark the lens position;
* avoid moving it during the remaining tests.

---

# 6. Configure PhotonVision at 1280 × 800

Set PhotonVision to:

```text
1280 × 800
```

This is the authoritative mode for:

* field of view;
* focus;
* exposure;
* gain;
* calibration;
* intrinsic parameters;
* distortion coefficients;
* tag pixel measurements;
* edge-of-image performance;
* pose testing.

Use the accompanying configuration table to identify which PhotonVision controls should be changed.

Settings that remain at a valid default and are not used by the final system do not need to be tested or recorded.

---

# 7. Determine Exposure

Begin with automatic exposure only long enough to obtain a usable image and identify a reasonable starting point.

Then disable automatic exposure.

Set exposure manually.

The objective is to use the shortest exposure that still provides reliable detection.

A shorter exposure:

* reduces motion blur;
* improves tag detection while the robot is moving;
* reduces smearing during rotation;
* may require additional gain or lighting.

A longer exposure:

* produces a brighter image;
* may improve stationary detection;
* increases motion blur.

Test exposure at:

* representative tag distances;
* the centre of the image;
* the image edges;
* expected robot motion;
* expected lighting levels.

Record the selected exposure time in microseconds.

Example Picamera2 control:

```python
picam2.set_controls({
    "AeEnable": False,
    "ExposureTime": 3000,
})
```

In this example:

```text
3000 µs = 3 ms
```

The selected PhotonVision exposure becomes the starting value for the final OpenCV pipeline.

---

# 8. Determine Camera Gain

Begin with the lowest practical gain.

Increase gain only when the required image brightness cannot be achieved through exposure and lighting alone.

Higher gain:

* brightens the image;
* increases image noise;
* may reduce edge quality;
* may reduce pose stability.

Test gain together with the selected exposure.

Do not optimize exposure and gain independently. They form a combined operating point.

Record the selected gain value.

Example Picamera2 control:

```python
picam2.set_controls({
    "AeEnable": False,
    "AnalogueGain": 1.0,
})
```

The PhotonVision gain becomes the starting value for the final OpenCV pipeline.

---

# 9. Verify Detection Across the Full Image

At `1280 × 800`, test AprilTag detection at:

* the image centre;
* the left edge;
* the right edge;
* the top edge;
* the bottom edge;
* all four corners.

For each location, check:

* whether the tag is detected;
* decision margin;
* pose stability;
* corner accuracy;
* apparent distortion;
* maximum reliable distance;
* minimum useful tag pixel width.

This stage establishes the authoritative full-field performance.

Do not use `640 × 480` for this test because it does not represent the same image geometry as the final `640 × 400` pipeline.

---

# 10. Measure Tag Pixel Width

At known distances, record the detected width of the AprilTag in pixels.

Use the `1280 × 800` image as the authoritative reference.

For a tag width measured as:

```text
P1280 pixels
```

the expected width at `640 × 400` is:

```text
P640 = P1280 × 0.5
```

For example:

```text
1280 × 800 measurement: 80 pixels
640 × 400 equivalent:   40 pixels
```

Use this relationship when estimating final detection thresholds and expected detection range.

The final OpenCV pipeline must still verify the actual result.

---

# 11. Configure AprilTag Detection

Use:

```text
tag36h11
```

for the SR arena and object tags.

Start with:

```text
Decimate: 1.0
Refine Edges: ON
Blur: 0
Max Error Bits: 0
Decision Margin: 30
```

These are initial values, not guaranteed final values.

Use the `1280 × 800` mode to evaluate:

* maximum range;
* pose quality;
* corner accuracy;
* edge performance.

Use `640 × 480` later for faster comparative tuning.

---

# 12. Calibrate the Camera at 1280 × 800

Perform the authoritative camera calibration in PhotonVision at:

```text
1280 × 800
```

Use a ChArUco calibration board.

Before starting, confirm:

* the board dictionary;
* number of squares across;
* number of squares vertically;
* square size;
* marker size;
* printed dimensions;
* calibration resolution.

Measure the printed board rather than relying only on nominal PDF dimensions.

The calibration board must remain flat.

---

# 13. Capture Calibration Images

Capture approximately:

```text
15 to 20 good images
```

More images may be used if they add genuinely different viewpoints.

The calibration set should cover:

* image centre;
* left edge;
* right edge;
* top edge;
* bottom edge;
* top-left corner;
* top-right corner;
* bottom-left corner;
* bottom-right corner;
* several distances;
* several board angles.

Use varied views rather than many nearly identical views.

Avoid:

* blur;
* repeated positions;
* extreme angles where the board is poorly detected;
* partly folded or curved boards;
* glare;
* poor lighting.

The board does not need to fill the complete image in every frame.

The objective is broad sensor coverage.

---

# 14. Review Calibration Quality

After completing calibration, record:

* mean reprojection error;
* horizontal FOV;
* vertical FOV;
* `fx`;
* `fy`;
* `cx`;
* `cy`;
* distortion coefficients;
* per-snapshot error if available.

A mean reprojection error below approximately one pixel is normally desirable, but it is not the only acceptance criterion.

Also verify that:

* `cx` is reasonably near the horizontal image centre;
* `cy` is reasonably near the vertical image centre;
* calculated FOV is plausible;
* pose estimates behave correctly;
* edge detections are stable;
* no single calibration snapshot has a much larger error than the others.

For a `1280 × 800` image, the nominal image centre is:

```text
cx ≈ 640
cy ≈ 400
```

The calibrated values do not need to be exactly equal to these values.

---

# 15. Save the Master Calibration

Download and retain the PhotonVision calibration file.

Store it with:

* camera identifier;
* camera model;
* lens;
* focus position;
* calibration resolution;
* date;
* PhotonVision version;
* board details;
* calibration error.

Treat the `1280 × 800` calibration as the master calibration for this camera.

Do not overwrite it with a `640 × 480` calibration.

---

# 16. Use PhotonVision at 640 × 480 for Fast Testing

After the authoritative setup is complete, switch PhotonVision to:

```text
640 × 480
```

Use this mode for rapid comparative testing only.

Suitable tests include:

* exposure comparisons;
* gain comparisons;
* decimation comparisons;
* blur testing;
* decision-margin thresholds;
* error-bit thresholds;
* detection stability;
* camera-loss testing;
* approximate FPS comparisons;
* motion testing;
* long-duration PhotonVision stability.

This mode is useful because it may run faster and make repeated tuning easier.

However, it uses different image geometry from the final `640 × 400` pipeline.

Do not use `640 × 480` as the source of:

* final FOV;
* final camera matrix;
* final principal point;
* final distortion coefficients;
* final edge-of-image coverage;
* final tag pixel thresholds.

---

# 17. Select Detector Settings

Use the PhotonVision tests to select starting values for:

* decimation;
* blur, if required;
* decision margin;
* maximum error bits;
* refine edges;
* exposure;
* gain.

Prefer the simplest configuration that performs reliably.

Do not adjust a setting merely because it exists.

A setting should be changed only when testing demonstrates a benefit.

---

# 18. Convert the Calibration to 640 × 400

The authoritative calibration is:

```text
1280 × 800
```

The final OpenCV image is:

```text
640 × 400
```

Both dimensions are reduced by exactly half.

Therefore:

```text
scale_x = 640 / 1280 = 0.5
scale_y = 400 / 800  = 0.5
```

Convert the intrinsic values as follows:

```python
fx_640 = fx_1280 * 0.5
fy_400 = fy_800 * 0.5
cx_640 = cx_1280 * 0.5
cy_400 = cy_800 * 0.5
```

Create the final camera matrix:

```python
camera_matrix_640x400 = np.array([
    [fx_1280 * 0.5, 0.0,             cx_1280 * 0.5],
    [0.0,             fy_800 * 0.5,   cy_800 * 0.5],
    [0.0,             0.0,             1.0],
], dtype=np.float64)
```

Initially copy the distortion coefficients unchanged:

```python
distortion_640x400 = distortion_1280x800.copy()
```

This is valid as an initial conversion because `640 × 400` is a uniform half-scale representation of the same full sensor area.

---

# 19. Configure Picamera2

Create the Picamera2 configuration using the full OV9281 sensor mode and a final processing stream of `640 × 400`.

Example:

```python
from picamera2 import Picamera2

picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={
        "size": (640, 400),
        "format": "RGB888",
    },
    raw={
        "size": (1280, 800),
    },
    buffer_count=1,
    queue=False,
)

picam2.configure(config)
```

Apply the selected exposure and gain:

```python
picam2.set_controls({
    "AeEnable": False,
    "ExposureTime": exposure_time_us,
    "AnalogueGain": analogue_gain,
})
```

Use the actual values determined during PhotonVision testing.

---

# 20. Configure the AprilTag Detector

Create the `pupil_apriltags` detector using the selected starting values.

Example:

```python
from pupil_apriltags import Detector

detector = Detector(
    families="tag36h11",
    nthreads=3,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=True,
)
```

Filter detections after processing.

Example:

```python
accepted_detections = [
    detection
    for detection in detections
    if detection.hamming == 0
    and detection.decision_margin >= 30.0
]
```

The final values should be taken from the completed configuration table.

---

# 21. Configure Pose Estimation

Use the converted camera parameters:

```python
camera_params = (
    fx_640,
    fy_400,
    cx_640,
    cy_400,
)
```

Use the correct physical tag size.

For SR arena tags:

```python
tag_size_m = 0.15
```

For SR object tags:

```python
tag_size_m = 0.08
```

Example:

```python
detections = detector.detect(
    gray,
    estimate_tag_pose=True,
    camera_params=camera_params,
    tag_size=tag_size_m,
)
```

Confirm that PhotonVision and the Python implementation use the same definition of tag size.

The required value is normally the black square size, not the outside dimensions of the printed paper.

---

# 22. Validate the Final 640 × 400 Pipeline

The final validation should confirm the transferred configuration rather than repeat the complete PhotonVision setup.

Test:

* actual full field of view;
* tag detection at known distances;
* tag pose at known distances;
* tag detection at image centre;
* tag detection near each edge;
* tag detection near each corner;
* exposure;
* gain;
* minimum reliable tag pixel width;
* maximum reliable detection range;
* decimation;
* CPU usage;
* FPS;
* end-to-end latency;
* dropped frames;
* camera reconnect behaviour;
* long-duration camera stability.

Use the same physical test positions used during the PhotonVision tests wherever practical.

---

# 23. Compare PhotonVision and OpenCV Results

At several known camera-to-tag positions, compare:

* detected tag ID;
* range;
* horizontal offset;
* vertical offset;
* pose translation;
* pose rotation;
* detection stability;
* decision margin;
* lost-frame behaviour.

Small differences are expected because PhotonVision and the Python detector may not use identical processing and pose-estimation implementations.

The purpose is not to force identical outputs.

The purpose is to confirm that the transferred configuration produces correct and stable robot behaviour.

---

# 24. Adjust Only Where Final Testing Shows a Difference

The following values may require limited final adjustment:

* exposure;
* gain;
* decimation;
* decision margin;
* thread count;
* buffering;
* tag acceptance thresholds.

The intrinsic matrix should not be manually adjusted merely to make individual test results look better.

If the converted calibration produces systematic pose errors:

1. confirm that the final image is genuinely full-field `640 × 400`;
2. confirm that no crop has been introduced;
3. confirm that the camera matrix was scaled correctly;
4. confirm that the distortion coefficient order is correct;
5. confirm the physical tag size;
6. confirm the calibration board measurements;
7. repeat the calibration only if the source calibration is suspect.

---

# 25. Record the Final Configuration

When validation is complete, record the final values in the configuration table.

At minimum, retain:

* camera identifier;
* sensor mode;
* processing resolution;
* exposure;
* gain;
* detector family;
* decimation;
* blur;
* thread count;
* refine edges;
* maximum error bits;
* decision margin;
* `fx`;
* `fy`;
* `cx`;
* `cy`;
* distortion coefficients;
* arena tag size;
* object tag size;
* measured maximum range;
* measured minimum tag pixel width;
* measured FPS;
* measured latency;
* calibration file name;
* calibration date.

---

# Final Workflow Summary

```text
Unbox and inspect camera
        ↓
Connect to Raspberry Pi
        ↓
Confirm OV9281 detection
        ↓
Confirm full 1280 × 800 sensor mode
        ↓
Mount camera in final physical configuration
        ↓
Set focus
        ↓
Establish exposure and gain at 1280 × 800
        ↓
Test full-field AprilTag performance
        ↓
Calibrate at 1280 × 800
        ↓
Save master calibration
        ↓
Use 640 × 480 for fast comparative tuning
        ↓
Select final detector starting values
        ↓
Scale fx, fy, cx and cy by 0.5
        ↓
Copy distortion coefficients
        ↓
Configure Picamera2 at 640 × 400
        ↓
Configure pupil_apriltags
        ↓
Validate final pose, range, FPS and stability
        ↓
Record final values in the configuration table
```

The key rule throughout the process is:

```text
1280 × 800 provides the authoritative camera geometry.

640 × 480 is used only for fast PhotonVision testing.

640 × 400 is the final Picamera2/OpenCV operating mode.
```
