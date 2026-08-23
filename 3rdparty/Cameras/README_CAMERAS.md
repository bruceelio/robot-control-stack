# Cameras

This document covers camera bring-up, configuration, calibration, AprilTag testing,
runtime integration, and multi-camera testing for all supported camera interfaces.

The documentation is organised by **interface/backend**, not by individual camera model:

```text
Pi CSI camera
    ↓
libcamera / Picamera2
    ↓
RGB frame
        \
         \
          → shared vision processing → observations → perception / localisation
         /
        /
USB camera
    ↓
V4L2 / OpenCV
    ↓
RGB frame
```

Current examples are:

- Raspberry Pi Camera Module 3 on the Pi CSI interface
- Arducam OV9281 on the USB/V4L2 interface

The same procedures should apply to other compatible Pi CSI or USB cameras, with
camera-specific values substituted where necessary.


## Table of Contents

Use this to jump directly to the section you need.

- [1. Architecture Overview](#1-architecture-overview)
- [2. Key Design Principle](#2-key-design-principle)
- [3. Python Environment](#3-python-environment)
- [4. Common Camera Workflow](#4-common-camera-workflow)
- [5. Pi CSI Cameras](#5-pi-csi-cameras)
- [6. Pi CSI Focus](#6-pi-csi-focus)
- [7. Pi CSI Exposure, Gain and Other Controls](#7-pi-csi-exposure-gain-and-other-controls)
- [8. USB Cameras](#8-usb-cameras)
- [9. USB Camera Discovery — Command Sequence](#9-usb-camera-discovery-command-sequence)
- [10. USB Exposure and Gain](#10-usb-exposure-and-gain)
- [11. USB Capture Path](#11-usb-capture-path)
- [12. Full-FOV Verification](#12-full-fov-verification)
- [13. Common Calibration Rules](#13-common-calibration-rules)
- [14. Chessboard Calibration](#14-chessboard-calibration)
- [15. Calibration Scripts](#15-calibration-scripts)
- [16. Calibration Quality](#16-calibration-quality)
- [17. Calibration Storage](#17-calibration-storage)
- [18. AprilTag Validation](#18-apriltag-validation)
- [19. Current Detector Tuning](#19-current-detector-tuning)
- [20. Static Detection Test](#20-static-detection-test)
- [21. Motion Blur Test](#21-motion-blur-test)
- [22. Close-Range Dropout Test](#22-close-range-dropout-test)
- [23. Occlusion Test](#23-occlusion-test)
- [24. Lighting Robustness](#24-lighting-robustness)
- [25. Standalone Testing vs Integrated Testing](#25-standalone-testing-vs-integrated-testing)
- [26. Two-Camera Comparison Test](#26-two-camera-comparison-test)
- [27. Final Multi-Camera Arrangement](#27-final-multi-camera-arrangement)
- [28. Camera Calibration vs Robot Mount Calibration](#28-camera-calibration-vs-robot-mount-calibration)
- [29. When to Redo Camera Setup](#29-when-to-redo-camera-setup)
- [30. Camera Lifecycle Rule](#30-camera-lifecycle-rule)
- [31. Useful Debugging Commands](#31-useful-debugging-commands)
- [32. Typical End-to-End Workflow](#32-typical-end-to-end-workflow)
- [33. Recommended Testing Philosophy](#33-recommended-testing-philosophy)
- [34. Final Rules](#34-final-rules)
- [35. Summary](#35-summary)

---

# 1. Architecture Overview

## 1.1 Camera roles

The robot separates:

```text
logical camera identity
camera profile
physical device
camera calibration
camera mount
detector / processor
localisation provider
behaviour
```

These are intentionally different concepts.

Example:

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
"front"
```

is the logical robot camera role.

```text
"arducam_fullfov_640_400"
```

is the reusable camera profile.

```text
/dev/v4l/by-id/...
```

is the physical device.

The same camera profile could be used on another robot, while the logical role or
physical device path could change.

---

## 1.2 Full runtime flow

```text
main.py
  ↓
Controller
  ↓
IOMap / resolve_io
  ↓
robot-specific IO
  ↓
camera process manager
  ↓
one vision worker per logical camera
  ↓
camera resolver
  ↓
capture backend
  ↓
shared detector / processor
  ↓
vision message
  ↓
perception and/or localisation
```

The real-hardware camera is not magically created by the SR API. The robot stack
constructs it explicitly.

For example, a Pi CSI camera is ultimately opened by the Picamera2 backend:

```python
self._picam2 = Picamera2()
...
self._picam2.start()
```

A USB camera is opened through OpenCV/V4L2:

```python
cv2.VideoCapture(device, cv2.CAP_V4L2)
```

---

## 1.3 Webots / Student Robotics comparison

In Student Robotics / Webots, code commonly looks like:

```python
markers = robot.camera.see()
```

The SR API has already:

- created the camera;
- connected it to hardware or simulation;
- performed the marker-processing path;
- returned SR marker objects.

In the native robot stack, those responsibilities are explicit:

```text
camera resolver
    ↓
capture backend
    ↓
shared AprilTag processing
    ↓
Marker objects / observations
```

This gives the same useful high-level behaviour while allowing direct control of:

- camera mode;
- exposure;
- focus;
- frame format;
- calibration;
- detector parameters;
- multi-camera operation.

---

# 2. Key Design Principle

The camera backend owns **capture**.

The shared detector/processor owns **vision processing**.

The higher-level perception/localisation system owns **robot interpretation**.

So the preferred split is:

```text
LEVEL 1 — CAMERA / CAPTURE
        ↓
LEVEL 2 — DETECTION / OBSERVATIONS
        ↓
LEVEL 3 — OBJECT PERCEPTION / LOCALISATION
```

A backend should not decide what the robot should do with a detection.

A localisation provider should not know whether the image came from Picamera2,
OpenCV, USB, CSI, or simulation.

---

# 3. Python Environment

The project currently uses the virtual environment:

```bash
source ~/apriltag-env/bin/activate
```

## 3.1 Pi CSI camera requirement

Picamera2 is normally installed through the Raspberry Pi system packages, not only
through pip.

Therefore, if a virtual environment is used, create it with access to system packages:

```bash
python3 -m venv --system-site-packages ~/apriltag-env
```

Otherwise Picamera2 may fail with:

```text
ModuleNotFoundError: No module named 'picamera2'
```

Verify:

```bash
python3 -c "from picamera2 import Picamera2; print('OK')"
```

Verify the AprilTag library:

```bash
python3 -c "from pupil_apriltags import Detector; print('OK')"
```

USB/OpenCV cameras do not require Picamera2, but they still need the project Python
environment and the OpenCV/V4L2 dependencies used by the stack.

---

# 4. Common Camera Workflow

Use the same high-level workflow regardless of camera interface:

```text
1. Confirm camera is visible
2. Confirm the physical device/interface
3. Determine supported capture modes
4. Select final capture resolution and FPS
5. Select final processing resolution
6. Fix focus if the camera supports focus
7. Tune exposure/gain/other controls
8. Verify actual state
9. Calibrate at the exact optical/image configuration
10. Validate AprilTag detection
11. Integrate into runtime
12. Re-check after hardware or configuration changes
```

The most important rule is:

> Fix the optical and image configuration before calibration.

---

# 5. Pi CSI Cameras

This section applies to Raspberry Pi CSI cameras using:

```text
libcamera / Picamera2
```

The current example is a Raspberry Pi Camera Module 3.

---

## 5.1 Basic hardware smoke test

To simply test that the Pi camera is working and show what it sees:

```bash
rpicam-still
```

To save an image:

```bash
rpicam-still -o test.jpg
```

This is useful before involving the project Python stack.

---

## 5.2 Activate environment

```bash
source ~/apriltag-env/bin/activate
```

Then verify:

```bash
python3 -c "from picamera2 import Picamera2; print('OK')"
python3 -c "from pupil_apriltags import Detector; print('OK')"
```

---

## 5.3 Pi camera bring-up

A standalone test script can be used to confirm:

- camera opens;
- frames stream;
- preview works;
- no robot control is involved.

Example historical project command:

```bash
python3 test_camera.py --preview drm
```

The exact script name may change as diagnostics are consolidated, but the test goal
remains the same.

---

## 5.4 Preview modes

Pi camera test utilities have historically supported preview modes such as:

```bash
--preview drm
--preview none
--preview save
```

Typical use:

| Use case | Mode |
|---|---|
| Development with Pi monitor | `drm` |
| Robot runtime | `none` |
| Debugging / saved frames | `save` |

If a particular script does not support these exact options, use its equivalent
preview/save mode.

---

## 5.5 Fix camera mode first

Before focus testing, calibration, AprilTag tuning, or robot testing, select and keep
constant:

- capture resolution;
- processing resolution;
- frame rate;
- sensor/crop mode where applicable.

Historical Pi3 work used 30 FPS and originally experimented with 640×480. The
current runtime profile should be treated as authoritative for the actual production
resolution.

A previous project helper was:

```bash
python3 set_camera_mode.py
```

with a target around:

```text
30 FPS
FrameDuration ≈ 33333 µs
```

The important rule is not the old script name or old 640×480 value; the important
rule is that the final runtime mode must be selected **before calibration**.

---

# 6. Pi CSI Focus

Some Pi CSI cameras, including Camera Module 3, have autofocus.

Focus must be fixed before calibration if the runtime will use a fixed focus.

## 6.1 Historical focus workflow

Autofocus baseline:

```bash
python3 focus/focus_autofocus_baseline.py
```

Candidate-value testing:

```bash
python3 focus/test_focus_values.py
```

Lock selected focus:

```bash
python3 focus/set_fixed_focus.py
```

The exact scripts may be replaced by newer diagnostics, but the procedure remains:

1. establish autofocus baseline;
2. test candidate lens positions;
3. choose a value covering the intended working range;
4. lock the focus;
5. do not change it after calibration.

---

## 6.2 Focus selection criteria

Evaluate:

- object detection at longer range;
- stable tracking during approach;
- acceptable arena-tag detection;
- useful close-range sharpness.

Historical Pi3 work targeted approximately:

```text
0.5 m to 1.0 m
```

as the most important object-tracking range.

A typical Camera Module 3 fixed `LensPosition` may fall around:

```text
1.0 to 1.5
```

but this is only a practical starting region, not a universal camera value.

---

## 6.3 Focus rules

- Do not recalibrate with autofocus changing the lens during capture.
- If the lens position changes materially, recalibrate.
- Do not confuse camera focus with camera mounting position.

Moving the entire calibrated camera on the robot changes the **mount transform**,
not necessarily the intrinsic calibration.

---

# 7. Pi CSI Exposure, Gain and Other Controls

Typical Picamera2/libcamera controls include:

```text
AeEnable
ExposureTime
AnalogueGain
AwbEnable
ColourGains
AfMode
LensPosition
```

A configuration-state tool may report values such as:

```text
AeEnable = False
ExposureTime ≈ 4000–7000 µs
AnalogueGain ≈ 1.0–4.0
AwbEnable = False
AfMode = 0
LensPosition = selected fixed value
FrameDuration ≈ 33333 µs
```

A previous project helper was:

```bash
python3 read_camera_state.py
```

Use the current equivalent whenever available.

The key principle is:

> Always verify what the camera actually applied, not only what was requested.

---

## 7.1 Camera configuration testing

Historical Pi-camera testing used candidate configuration files and test scripts such as:

```text
camera_test_configs.py
run_camera_config_tests.py
```

Example:

```bash
python3 run_camera_config_tests.py --preview drm
```

or:

```bash
python3 run_camera_config_tests.py --preview save
```

Evaluate:

- AprilTag detection reliability;
- distance stability;
- bearing stability;
- motion blur;
- brightness consistency;
- exposure response.

---

# 8. USB Cameras

This section applies to USB/UVC cameras using:

```text
V4L2 / OpenCV
```

The current example is the Arducam OV9281 USB camera.

The same command sequence is useful for most Linux USB cameras.

---

# 9. USB Camera Discovery — Command Sequence

This is the practical sequence to use when connecting or changing a USB camera.

---

## Step 1 — List camera/video devices

```bash
v4l2-ctl --list-devices
```

Example output may show:

```text
unicam:
    /dev/video0
    /dev/video1

Arducam Technology Co., Ltd. Arducam OV9281 USB Camera:
    /dev/video2
    /dev/video3
    /dev/media5
```

Do not assume `/dev/video2` will always remain `/dev/video2`.

---

## Step 2 — Find stable USB device paths

```bash
ls -l /dev/v4l/by-id/
```

For the current OV9281 this produced a stable path similar to:

```text
/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC599-video-index0
```

and a second entry:

```text
...-video-index1
```

Use the actual capture interface, currently:

```text
video-index0
```

Prefer the stable `/dev/v4l/by-id/...` path in robot configuration instead of a
temporary `/dev/videoN` number.

---

## Step 3 — Store the device path in a shell variable

This makes later commands easier to read:

```bash
DEVICE=/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC599-video-index0
```

For another USB camera, substitute its own stable path.

---

## Step 4 — Inspect supported formats, resolutions and frame rates

```bash
v4l2-ctl -d "$DEVICE" --list-formats-ext
```

For the current OV9281, useful advertised modes include MJPG at:

```text
1280 × 800
```

with 30 FPS available.

The camera also exposes YUYV modes, but the usable FPS differs by resolution.

The current robot configuration uses:

```text
capture:    1280 × 800 MJPG @ 30 FPS
processing:  640 × 400
```

This preserves the 1.6:1 aspect ratio:

```text
1280 / 800 = 1.6
640 / 400 = 1.6
```

Therefore the software resize does not itself crop the image.

---

## Step 5 — Inspect current V4L2 controls and ranges

```bash
v4l2-ctl -d "$DEVICE" --list-ctrls-menus
```

For the current OV9281, observed controls include:

```text
brightness
contrast
saturation
hue
white_balance_automatic
gamma
gain
power_line_frequency
white_balance_temperature
sharpness
backlight_compensation
auto_exposure
exposure_time_absolute
exposure_dynamic_framerate
```

Observed ranges included:

```text
brightness                  -64 .. 64
contrast                      0 .. 64
saturation                    0 .. 128
hue                         -40 .. 40
gamma                        72 .. 500
gain                          0 .. 100
sharpness                     0 .. 6
backlight_compensation        0 .. 2
exposure_time_absolute        1 .. 5000
```

Observed exposure mode menu:

```text
1 = Manual
3 = Aperture Priority
```

Observed mains-frequency menu:

```text
0 = Disabled
1 = 50 Hz
2 = 60 Hz
```

These values are camera/driver-specific. Always inspect the actual USB camera rather
than assuming all UVC cameras expose the same ranges.

---

# 10. USB Exposure and Gain

The current OV9281 test configuration is:

```text
auto_exposure = 1
exposure_time_absolute = 45
gain = 0
```

Standard UVC exposure absolute values are normally in 100 µs units, therefore:

```text
45 ≈ 4.5 ms
```

This is also similar to the Pi-camera 4.5 ms exposure reference used during earlier
AprilTag testing.

---

## Step 6 — Set manual exposure

```bash
v4l2-ctl \
  -d "$DEVICE" \
  --set-ctrl=auto_exposure=1,exposure_time_absolute=45,gain=0
```

This is the current working OV9281 test setting.

For another USB camera, first check that these control names and ranges exist.

---

## Step 7 — Verify the applied controls

Minimal verification:

```bash
v4l2-ctl \
  -d "$DEVICE" \
  --get-ctrl=auto_exposure,exposure_time_absolute,gain
```

Expected current OV9281 result:

```text
auto_exposure: 1 (Manual Mode)
exposure_time_absolute: 45
gain: 0
```

Expanded verification:

```bash
v4l2-ctl \
  -d "$DEVICE" \
  --get-ctrl=auto_exposure,exposure_time_absolute,gain,power_line_frequency,brightness,contrast,gamma,sharpness,backlight_compensation
```

Observed during current testing:

```text
gain: 0
power_line_frequency: 2
brightness: 0
contrast: 32
gamma: 100
sharpness: 3
backlight_compensation: 1
auto_exposure: 1 (Manual Mode)
exposure_time_absolute: 45
```

Values not explicitly being controlled by the project should be treated as observed
state, not as mandatory target values.

---

## Step 8 — Check persistence

If intending to set USB controls once and leave them alone, explicitly test whether
they survive:

```text
camera close/reopen
USB unplug/replug
Pi reboot
camera power cycle
```

After each, rerun:

```bash
v4l2-ctl -d "$DEVICE" --get-ctrl=auto_exposure,exposure_time_absolute,gain
```

Some UVC devices/drivers retain controls and some reset them.

Do not add startup-control code unless persistence proves necessary.

---

# 11. USB Capture Path

Current OV9281 runtime flow:

```text
Arducam OV9281
    ↓
USB / UVC
    ↓
V4L2
    ↓
OpenCV VideoCapture
    ↓
1280 × 800 MJPG @ 30 FPS
    ↓
software resize
    ↓
640 × 400 processing frame
    ↓
RGB conversion
    ↓
shared AprilTagProcessor
```

The runtime backend reports requested and actual capture mode so the program can
confirm that the camera accepted the desired configuration.

---

# 12. Full-FOV Verification

Matching aspect ratio proves that the software resize is not cropping, but it does not
by itself prove that the USB firmware is exposing the entire physical sensor area.

To verify actual field of view:

1. capture or display the native camera image;
2. compare scene boundaries;
3. compare the native frame with the processed frame;
4. verify that left/right/top/bottom boundaries are preserved.

For a camera comparison test, mount two cameras facing the same direction and compare
their scene coverage directly.

---

# 13. Common Calibration Rules

Calibration determines the intrinsic camera parameters:

```text
fx
fy
cx
cy
distortion coefficients
```

These are camera/optical/image-mode properties.

They are not the same as the robot camera mount.

---

## 13.1 Fix these before calibration

Keep constant:

- camera/lens;
- focus or lens position;
- sensor mode;
- capture resolution;
- processing resolution;
- crop/scaler configuration;
- any image transformation that changes the calibrated geometry.

Frame rate should also match the intended runtime configuration where practical,
although intrinsic geometry is primarily determined by the optical/image mode.

---

## 13.2 Camera mounting and calibration

The camera does **not** need to remain in the same robot mounting position for its
intrinsic calibration to remain valid.

These are separate:

```text
intrinsic calibration:
    fx, fy, cx, cy, distortion

robot mount:
    x, y, z, roll, pitch, yaw
```

If the entire calibrated camera is moved rigidly to a different location, update the
mount transform.

Recalibrate intrinsics if the optical configuration changes.

---

# 14. Chessboard Calibration

A standard practical method is chessboard calibration.

A useful board configuration is:

```text
9 columns × 6 rows of inner corners
```

Important:

```text
9 × 6 refers to inner corners, not squares
```

A 9×6 inner-corner pattern contains:

```text
10 squares across
7 squares down
```

A practical square size is around:

```text
20–30 mm
```

A commonly used project value is:

```text
25 mm
```

The exact real square size must be known.

---

## 14.1 Board preparation

The board should be:

- flat;
- rigid or firmly backed;
- sharp;
- high contrast;
- accurately printed.

Avoid:

- wrinkled paper;
- glare;
- unknown scaling;
- warped boards.

---

## 14.2 Capture distance

Use a range of distances rather than one fixed position.

A practical starting range is approximately:

```text
20 cm to 100 cm
```

The board should generally occupy about:

```text
30% to 80% of the image
```

depending on the view.

Avoid:

- board too tiny;
- board filling the whole frame every time;
- only one distance;
- only straight-on images.

---

## 14.3 Required views

Capture varied views with the board:

- centred;
- near left edge;
- near right edge;
- near top/bottom/corners;
- tilted left/right;
- tilted up/down;
- rotated;
- close;
- farther away.

A good rough target is:

```text
15–25 good images
```

More is not useful if they are all nearly identical.

---

## 14.4 Simple capture routine

Repeat a sequence similar to:

```text
Centre, straight → capture
Move to left edge → capture
Move to right edge → capture
Tilt left → capture
Tilt right → capture
Move closer → capture
Move farther → capture
```

Repeat roughly three times to produce about 20 varied images.

---

## 14.5 Practical capture advice

For every image:

- keep the full chessboard visible;
- avoid motion blur;
- avoid glare;
- hold the board still;
- ensure corner detection is successful;
- keep the board flat.

If chessboard detection is unreliable:

- improve lighting;
- move closer;
- use a larger board;
- reduce blur;
- verify the pattern dimensions match the script.

---

# 15. Calibration Scripts

Historical Pi-camera calibration used:

```bash
python3 calibrate_pi_camera.py --capture --preview
```

and:

```bash
python3 calibrate_pi_camera.py --solve
```

Some project versions also used:

```bash
python3 calibrate_pi_camera.py --capture
```

without live preview.

The important workflow is:

```text
capture images
    ↓
detect chessboard corners
    ↓
refine corners
    ↓
solve calibration
    ↓
inspect reprojection error
    ↓
copy results into calibration/cameras/<camera_profile>.py
```

As the camera diagnostics are generalised, this should eventually become a common
calibration tool rather than a Pi-specific tool.

---

# 16. Calibration Quality

After solving, inspect:

- number of usable images;
- RMS reprojection error;
- mean reprojection error;
- coverage across the image.

Lower error is generally better.

Poor calibration can result from:

- blur;
- board warp;
- wrong square size;
- too few views;
- insufficient angle variation;
- insufficient edge/corner coverage;
- focus changing during capture;
- using a different resolution from runtime.

If quality is poor:

- capture more varied images;
- add tilted views;
- add off-centre views;
- re-check square size;
- verify fixed focus;
- verify the runtime resolution.

---

# 17. Calibration Storage

Calibration belongs under:

```text
calibration/cameras/
```

Example current profiles:

```text
pi3_fullfov_640_360.py
arducam_fullfov_640_400.py
```

The calibration name should match the camera/image configuration it represents.

Do not call a calibration simply `"camera"` if the same physical camera could later
run a different resolution or crop mode.

---

# 18. AprilTag Validation

After calibration, validate AprilTag performance before autonomous integration.

Check:

- correct IDs;
- detection stability;
- maximum useful distance;
- minimum useful distance;
- distance consistency;
- bearing consistency;
- vertical-angle consistency;
- motion blur;
- lighting sensitivity;
- edge-of-frame behaviour.

---

## 18.1 Historical Pi3 AprilTag test

A previous standalone command was:

```bash
python3 apriltag_pi3_test.py --preview drm
```

With explicit calibration values:

```bash
python3 apriltag_pi3_test.py \
  --preview drm \
  --fx 950 \
  --fy 950 \
  --cx 320 \
  --cy 240 \
  --tag-size-m 0.08
```

Debug frame capture:

```bash
python3 apriltag_pi3_test.py --preview save
```

The long-term goal should be a common camera diagnostic tool that selects a camera
profile rather than maintaining separate Pi-only and USB-only AprilTag scripts.

---

# 19. Current Detector Tuning

Current shared AprilTag processing uses parameters such as:

```text
FAMILIES = tag36h11
MIN_DECISION_MARGIN = 20
QUAD_DECIMATE = 1.5
NTHREADS = 2
QUAD_SIGMA = 0.0
REFINE_EDGES = 1
DECODE_SHARPENING = 0.25
```

These are detector parameters, not physical camera interface settings.

They may eventually be tuned differently for different camera profiles.

A useful diagnostic comparison is:

```text
QUAD_DECIMATE = 1.5
vs
QUAD_DECIMATE = 1.0
```

particularly when testing long-range detection.

---

# 20. Static Detection Test

Place the target tag at several known distances.

A useful sequence is:

```text
0.5 m
1.0 m
1.5 m
2.0 m
```

or whatever range is relevant to the robot.

Record:

- detection count;
- missed detections;
- decision margin;
- measured distance;
- bearing;
- frame/update rate.

---

# 21. Motion Blur Test

Test increasing relative motion.

Historical test guidance used speeds approximately:

```text
0.1 m/s → 0.5 m/s
```

Record:

- detection success rate;
- frame dropouts;
- measured-distance stability;
- bearing stability.

If blur is excessive:

- reduce exposure time;
- add lighting;
- increase gain only as required.

General principle:

> Short exposure is preferable to a bright but motion-blurred image.

---

# 22. Close-Range Dropout Test

Slowly approach the tag and record where reliable detection stops.

This establishes the minimum usable range before the robot must commit to its final
blind/geometry-based approach.

The exact distance is camera/lens/tag-size dependent.

---

# 23. Occlusion Test

Simulate the real gripper/lift geometry.

Check whether:

- the manipulator blocks the tag;
- the tag leaves the field of view;
- the final approach can still complete safely.

This should be repeated after mechanical camera-mount changes.

---

# 24. Lighting Robustness

Test under:

- bright lighting;
- dim lighting;
- uneven lighting;
- artificial mains-powered lighting.

For USB cameras, anti-flicker settings may be relevant.

In the UK:

```text
mains frequency = 50 Hz
```

For the current OV9281, `power_line_frequency=1` corresponds to 50 Hz and
`power_line_frequency=2` corresponds to 60 Hz.

During current testing the camera reported:

```text
power_line_frequency = 2
```

This is worth evaluating later, but should not be changed casually during a controlled
test where exposure is the only intended variable.

---

# 25. Standalone Testing vs Integrated Testing

There are two useful levels.

## 25.1 Standalone camera testing

Use without autonomous robot logic.

Verify:

- camera opens;
- capture mode is correct;
- FOV is correct;
- detector works;
- geometry is reasonable;
- calibration is consistent.

This is the preferred mode for camera comparison.

---

## 25.2 Integrated diagnostics

Run through the robot stack to verify:

- configuration profile resolution;
- camera resolver;
- async worker;
- shared AprilTag processor;
- vision messages;
- perception integration;
- localisation integration.

Do not use a full autonomous run merely to answer a basic camera question if a
standalone diagnostic can answer it more safely.

---

# 26. Two-Camera Comparison Test

For short-term Pi-vs-USB comparison, point both cameras forward.

Give them separate logical names, for example:

```python
CAMERAS = {
    "front_pi": {
        "profile": "pi3_fullfov_640_360",
        "device": ...,
    },
    "front_usb": {
        "profile": "arducam_fullfov_640_400",
        "device": ...,
    },
}
```

Do not give both dictionary entries the same `"front"` key.

---

## 26.1 What to compare

With both cameras viewing the same scene:

- horizontal FOV;
- vertical FOV;
- image boundaries;
- tag detection rate;
- maximum reliable range;
- minimum reliable range;
- decision margin;
- measured distance;
- measured bearing;
- motion-blur sensitivity;
- lighting sensitivity;
- update rate.

Keep the streams independent during comparison.

Do not fuse them before understanding their individual behaviour.

---

# 27. Final Multi-Camera Arrangement

A more useful final robot arrangement is likely:

```text
front camera → forward-facing
rear camera  → backward-facing
```

Each camera should produce independent observations.

Combination happens later in perception/localisation:

```text
front observations ----\
                         \
                          → perception / localisation arbitration
                         /
rear observations -----/
```

Camera fusion should not occur inside the capture backend.

---

# 28. Camera Calibration vs Robot Mount Calibration

This distinction is critical.

## Camera intrinsic calibration

Describes the camera/lens/image configuration:

```text
fx
fy
cx
cy
distortion
```

Stored under:

```text
calibration/cameras/
```

## Robot camera mount

Describes where the logical camera is attached to the robot:

```text
x
y
z
roll
pitch
yaw
```

Stored in the robot profile / resolved robot calibration.

Changing mount position requires updating the mount transform.

Changing focus, lens, crop, or calibrated resolution may require intrinsic recalibration.

---

# 29. When to Redo Camera Setup

Redo or re-check focus/calibration if:

- the camera module is replaced;
- the lens is changed or disturbed;
- fixed focus is changed;
- resolution changes;
- sensor crop/scaler mode changes;
- processing geometry changes;
- working range changes enough to require a different focus strategy.

Moving a rigid calibrated camera to a different robot location does not automatically
require intrinsic recalibration, but the mount transform must be updated.

---

# 30. Camera Lifecycle Rule

A camera device should normally be opened once.

Do not repeatedly call camera/IO resolvers from diagnostics while the runtime already
owns the device.

For Pi cameras, repeatedly constructing Picamera2 can attempt to open the same sensor
again.

For USB cameras, repeatedly opening the same V4L2 device can similarly create resource
or access conflicts.

Use the existing runtime camera instance or run a standalone diagnostic when the main
robot process is not using the camera.

---

# 31. Useful Debugging Commands

## Pi CSI

Test camera:

```bash
rpicam-still
```

Save image:

```bash
rpicam-still -o test.jpg
```

Verify Picamera2:

```bash
python3 -c "from picamera2 import Picamera2; print('OK')"
```

Verify AprilTag package:

```bash
python3 -c "from pupil_apriltags import Detector; print('OK')"
```

---

## USB / V4L2

List devices:

```bash
v4l2-ctl --list-devices
```

Stable device paths:

```bash
ls -l /dev/v4l/by-id/
```

Store path:

```bash
DEVICE=/dev/v4l/by-id/<camera-video-index0>
```

Formats and FPS:

```bash
v4l2-ctl -d "$DEVICE" --list-formats-ext
```

Controls:

```bash
v4l2-ctl -d "$DEVICE" --list-ctrls-menus
```

Set current OV9281 manual exposure:

```bash
v4l2-ctl \
  -d "$DEVICE" \
  --set-ctrl=auto_exposure=1,exposure_time_absolute=45,gain=0
```

Verify:

```bash
v4l2-ctl \
  -d "$DEVICE" \
  --get-ctrl=auto_exposure,exposure_time_absolute,gain
```

Expanded verification:

```bash
v4l2-ctl \
  -d "$DEVICE" \
  --get-ctrl=auto_exposure,exposure_time_absolute,gain,power_line_frequency,brightness,contrast,gamma,sharpness,backlight_compensation
```

---

# 32. Typical End-to-End Workflow

## Pi CSI example

```text
1. rpicam-still
2. activate apriltag-env
3. verify Picamera2 + pupil_apriltags imports
4. select final capture/processing mode
5. establish autofocus baseline
6. test fixed focus values
7. lock focus
8. tune exposure/gain/white balance
9. verify actual camera state
10. capture calibration images
11. solve calibration
12. store calibration profile
13. run standalone AprilTag validation
14. integrate into robot stack
```

Historical helper names included:

```text
test_camera.py
set_camera_mode.py
focus/focus_autofocus_baseline.py
focus/test_focus_values.py
focus/set_fixed_focus.py
camera_test_configs.py
run_camera_config_tests.py
read_camera_state.py
calibrate_pi_camera.py
apriltag_pi3_test.py
```

These names are retained here so none of the previous Pi3 setup knowledge is lost,
even if some tools are later consolidated or renamed.

---

## USB example

```text
1. connect camera
2. v4l2-ctl --list-devices
3. ls -l /dev/v4l/by-id/
4. select video-index0 capture path
5. v4l2-ctl --list-formats-ext
6. choose final capture format/resolution/FPS
7. v4l2-ctl --list-ctrls-menus
8. set exposure/gain as needed
9. verify applied controls
10. verify persistence if relying on manual one-time setup
11. verify native and processing FOV
12. capture calibration images
13. solve calibration
14. store calibration profile
15. run standalone AprilTag validation
16. integrate into robot stack
```

---

# 33. Recommended Testing Philosophy

Before autonomous testing, establish:

```text
camera opens reliably
capture mode is correct
FOV is understood
exposure/focus are fixed
calibration is valid
AprilTag detection is reliable
geometry is stable
```

Only then use the camera for behaviour testing.

This makes it possible to distinguish:

```text
camera problem
detector problem
calibration problem
perception problem
behaviour problem
```

instead of debugging all five at once.

---

# 34. Final Rules

1. **Fix camera mode before calibration.**
2. **Fix focus before calibration when focus is adjustable.**
3. **Do not silently change calibrated resolution or crop.**
4. **Verify actual applied settings.**
5. **Use stable USB `/dev/v4l/by-id` paths.**
6. **Keep camera intrinsic calibration separate from robot mount calibration.**
7. **Keep camera capture separate from detector logic.**
8. **Keep multi-camera observations independent until the higher-level system combines them.**
9. **Use standalone camera tests before autonomous runs.**
10. **Short exposure plus sufficient lighting is preferable to motion-blurred imagery.**

---

# 35. Summary

The camera system supports two main physical interface families:

```text
Pi CSI camera
    ↓
Picamera2 / libcamera

USB camera
    ↓
V4L2 / OpenCV
```

Both ultimately feed the same higher-level vision architecture:

```text
capture
    ↓
shared detector / processor
    ↓
camera-labelled observations
    ↓
object perception and/or localisation
```

The current Pi Camera 3 and Arducam OV9281 are examples of those two interfaces,
not separate architectural systems.

The practical camera workflow is therefore shared:

```text
discover
configure
verify
calibrate
validate
integrate
```

with only the interface-specific commands changing.
