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

## 4.1 Camera setup tool organisation

Camera setup utilities live under:

```text
tools/setup/cameras/
```

Interface-specific implementations live below their interface:

```text
tools/setup/cameras/pi/
tools/setup/cameras/usb/
```

Utilities intended to work across camera interfaces live directly under:

```text
tools/setup/cameras/
```

Current examples are:

```text
tools/setup/cameras/
    camera_calibrate.py
    camera_apriltag_validate.py

    pi/
        camera_smoke_test.py
        camera_state.py
        camera_test_configs.py
        camera_config_tests.py

        focus/
            README.md
            autofocus_baseline.py
            test_focus_values.py
            set_fixed_focus.py

    usb/
        ...
```

The organisation rule is:

> A concept may be generic, but if a Python implementation depends specifically on
> Picamera2/libcamera or V4L2/USB, it belongs in the corresponding interface directory.

`camera_calibrate.py` and `camera_apriltag_validate.py` are intended to become
interface-independent common tools. Their current implementations may still contain
Pi-specific capture code while that generalisation is completed.

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

Current project command:

```bash
python3 tools/setup/cameras/pi/camera_smoke_test.py --preview drm
```

This is a setup/bring-up tool. It is kept under `pi/` because its implementation uses
the Pi CSI / Picamera2 interface.

---

## 5.4 Preview modes

Pi camera setup utilities may support preview modes such as:

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
current runtime camera profile is authoritative for the production capture and
processing mode.

There is no separate maintained `set_camera_mode.py` tool. Camera mode belongs in the
camera profile so setup, calibration, validation, and runtime all refer to the same
intended configuration.

For a 30 FPS mode, frame duration is approximately:

```text
33333 µs
```

The important rule is that the final runtime mode must be selected **before
calibration**.

---

# 6. Pi CSI Focus

Some Pi CSI cameras, including Camera Module 3, have autofocus.

Focus must be fixed before calibration if the runtime will use a fixed focus.

## 6.1 Pi focus workflow

Autofocus baseline:

```bash
python3 tools/setup/cameras/pi/focus/autofocus_baseline.py
```

Candidate-value testing:

```bash
python3 tools/setup/cameras/pi/focus/test_focus_values.py
```

Lock selected focus:

```bash
python3 tools/setup/cameras/pi/focus/set_fixed_focus.py
```

The detailed Pi focus procedure is maintained in:

```text
tools/setup/cameras/pi/focus/README.md
```

The procedure is:

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

Current project tool:

```bash
python3 tools/setup/cameras/pi/camera_state.py
```

The key principle is:

> Always verify what the camera actually applied, not only what was requested.

---

## 7.1 Camera configuration testing

Pi-camera setup testing uses:

```text
tools/setup/cameras/pi/camera_test_configs.py
tools/setup/cameras/pi/camera_config_tests.py
```

`camera_test_configs.py` contains the candidate configurations. `camera_config_tests.py`
runs them.

Example:

```bash
python3 tools/setup/cameras/pi/camera_config_tests.py --preview drm
```

or:

```bash
python3 tools/setup/cameras/pi/camera_config_tests.py --preview save
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

This section is a **bring-up and configuration procedure for a new USB/UVC camera**.

It is intentionally not tied to a particular camera model. The goal is that a new
camera — for example a Logitech C270, another Arducam, or any other Linux UVC camera —
can be connected and taken through the same sequence to determine its correct robot
configuration.

The process is:

```text
connect camera
    ↓
identify Linux device
    ↓
find stable device path
    ↓
discover supported formats / resolutions / FPS
    ↓
discover available camera controls
    ↓
select candidate capture mode
    ↓
verify Python/OpenCV capture
    ↓
select processing resolution
    ↓
tune focus / exposure / gain if supported
    ↓
calibrate that exact optical + processing configuration
    ↓
validate AprilTag performance
    ↓
record the successful values in a camera profile
```

Do not begin by copying settings from another USB camera. Different UVC cameras may
have different:

- image formats;
- maximum frame rates;
- resolutions;
- exposure-control names and ranges;
- gain ranges;
- autofocus behaviour;
- field of view;
- distortion;
- calibration.

The commands below are intended to discover those values from the camera itself.

---

# 9. USB Camera Discovery — Step by Step

## Step 1 — Confirm Linux can see the camera

Connect the camera, then run:

```bash
v4l2-ctl --list-devices
```

Typical output looks like:

```text
<USB camera name>:
    /dev/video2
    /dev/video3
    /dev/media4
```

At this stage, only establish:

1. whether the expected camera appears;
2. which `/dev/videoN` nodes belong to it.

If the camera is absent here, do not continue into Python. Resolve the USB/Linux
device problem first.

---

## Step 2 — Find a stable device name

Linux `/dev/videoN` numbers can change between boots or when another camera is
connected.

Run:

```bash
ls -l /dev/v4l/by-id/
```

Look for entries belonging to the new camera, for example:

```text
usb-<manufacturer>_<camera>-video-index0
usb-<manufacturer>_<camera>-video-index1
```

Use the stable `/dev/v4l/by-id/...` path whenever possible.

Set it once in the current shell:

```bash
DEVICE=/dev/v4l/by-id/<camera-video-index0>
```

Verify it:

```bash
ls -l "$DEVICE"
```

If the camera exposes more than one video index, do not assume which one is the image
capture stream. Test each candidate using the next steps.

---

## Step 3 — Discover all supported formats, resolutions and frame rates

Run:

```bash
v4l2-ctl -d "$DEVICE" --list-formats-ext
```

This is the primary command for determining what the camera can actually deliver.

Record the useful combinations of:

```text
pixel format
capture width
capture height
frame rate
```

For example, a camera might advertise combinations such as:

```text
MJPG   1280×720 @ 30 FPS
MJPG    640×480 @ 30 FPS
YUYV    640×480 @ 30 FPS
YUYV   1280×720 @ 10 FPS
```

Those are only examples. Use the values reported by the actual camera.

### Choosing the first candidate mode

For AprilTag work, initially favour a mode which:

- preserves the camera's useful field of view;
- provides enough pixels for distant tags;
- can maintain the desired frame rate;
- does not create excessive USB bandwidth;
- can be resized cleanly to the intended processing resolution.

MJPG is often useful for higher-resolution USB2 capture because it reduces USB
bandwidth. YUYV can be simpler but requires much more bandwidth at the same
resolution/FPS.

Do not decide between them by assumption; use the camera's advertised modes and then
test the candidates.

---

## Step 4 — Confirm the selected mode directly with V4L2

Once a candidate has been chosen, inspect the device while using that exact node:

```bash
v4l2-ctl -d "$DEVICE" --all
```

This gives a broad snapshot of the active format and camera state.

If troubleshooting a requested mode, return to:

```bash
v4l2-ctl -d "$DEVICE" --list-formats-ext
```

and confirm that the requested resolution, pixel format and FPS are actually an
advertised combination.

---

## Step 5 — Discover the camera controls

Run:

```bash
v4l2-ctl -d "$DEVICE" --list-ctrls-menus
```

Do this **before writing any control values**.

Depending on the camera, this may expose controls for:

```text
auto/manual exposure
exposure time
gain
autofocus
manual focus position
white balance
brightness
contrast
gamma
sharpness
power-line / anti-flicker frequency
backlight compensation
```

Control names and ranges are driver-specific.

A Logitech camera, an Arducam and another generic UVC camera may use different control
names for conceptually similar features. The output of `--list-ctrls-menus` is the
authoritative starting point.

---

## Step 6 — Record the untouched control state

Before changing anything, save or copy the current state:

```bash
v4l2-ctl -d "$DEVICE" --all
```

For individual controls, use the exact names discovered above:

```bash
v4l2-ctl -d "$DEVICE" --get-ctrl=<control_name>
```

or several at once:

```bash
v4l2-ctl \
  -d "$DEVICE" \
  --get-ctrl=<control_1>,<control_2>,<control_3>
```

This provides a baseline and makes it much easier to identify which change caused an
improvement or regression.

---

# 10. USB Focus, Exposure and Gain

The aim of this section is not to prescribe values. It is to determine the correct
values for the new camera.

## 10.1 Focus

First inspect whether the camera exposes autofocus or manual-focus controls:

```bash
v4l2-ctl -d "$DEVICE" --list-ctrls-menus
```

If the camera has autofocus:

1. establish that autofocus can produce a sharp image at the intended working range;
2. determine whether runtime autofocus is sufficiently stable;
3. if a fixed focus will be used, disable autofocus and test candidate manual focus
   values;
4. calibrate only after the final focus strategy has been chosen.

Use the exact control names reported by the camera. A typical command shape is:

```bash
v4l2-ctl \
  -d "$DEVICE" \
  --set-ctrl=<autofocus_control>=0,<focus_control>=<candidate>
```

Then verify:

```bash
v4l2-ctl \
  -d "$DEVICE" \
  --get-ctrl=<autofocus_control>,<focus_control>
```

Do not copy focus values from another camera model.

---

## 10.2 Exposure

For moving robots and AprilTags, exposure is usually a trade-off:

```text
longer exposure → brighter image but more motion blur
shorter exposure → darker image but less motion blur
```

Start by identifying the exact exposure controls:

```bash
v4l2-ctl -d "$DEVICE" --list-ctrls-menus
```

Then, if manual exposure is supported, test one variable at a time.

Generic command shape:

```bash
v4l2-ctl \
  -d "$DEVICE" \
  --set-ctrl=<auto_exposure_control>=<manual_value>,<exposure_control>=<candidate>
```

Read the values back immediately:

```bash
v4l2-ctl \
  -d "$DEVICE" \
  --get-ctrl=<auto_exposure_control>,<exposure_control>
```

Never assume:

- the numeric value means the same thing on another camera;
- the driver accepted the requested value;
- the manual-mode enumeration is the same between cameras.

Use `--list-ctrls-menus` to determine the valid values first.

---

## 10.3 Gain

If the image becomes too dark after reducing exposure, test gain separately.

Generic form:

```bash
v4l2-ctl \
  -d "$DEVICE" \
  --set-ctrl=<gain_control>=<candidate>
```

Verify:

```bash
v4l2-ctl \
  -d "$DEVICE" \
  --get-ctrl=<gain_control>
```

Prefer enough light and a short usable exposure before relying heavily on gain.
Evaluate the result through actual AprilTag detection rather than image appearance
alone.

---

## 10.4 Other controls

If results change unexpectedly, inspect all controls again:

```bash
v4l2-ctl -d "$DEVICE" --all
```

and:

```bash
v4l2-ctl -d "$DEVICE" --list-ctrls-menus
```

This is especially useful for:

- automatic white balance;
- brightness;
- contrast;
- gamma;
- sharpness;
- backlight compensation;
- anti-flicker / power-line frequency.

Do not change several of these at once during a controlled test.

---

## 10.5 Check whether controls persist

USB cameras differ in whether settings survive:

```text
camera close/reopen
USB unplug/replug
Pi reboot
camera power cycle
```

After each boundary, read the important controls again:

```bash
v4l2-ctl \
  -d "$DEVICE" \
  --get-ctrl=<control_1>,<control_2>,<control_3>
```

If the required state resets, the runtime or startup procedure must explicitly apply
those controls.

If it persists reliably, additional startup-control code may be unnecessary.

---

# 11. USB Capture and Python Validation

After the Linux/V4L2 layer is understood, test the Python capture path independently
of the robot controller.

Activate the project environment:

```bash
source ~/apriltag-env/bin/activate
```

Verify OpenCV:

```bash
python3 -c "import cv2; print(cv2.__version__)"
```

Verify the AprilTag package:

```bash
python3 -c "from pupil_apriltags import Detector; print('OK')"
```

---

## 11.1 Run the common camera validator with explicit values

Do not begin with hidden profile defaults while bringing up a new camera. Pass the
candidate capture settings explicitly so the test is reproducible.

Generic command:

```bash
PYTHONPATH=. python3 tools/setup/cameras/camera_apriltag_validate.py \
  --backend usb \
  --device "$DEVICE" \
  --capture-width <capture_width> \
  --capture-height <capture_height> \
  --processing-width <processing_width> \
  --processing-height <processing_height> \
  --fps <fps> \
  --format <pixel_format> \
  --calibration-module calibration.cameras.<camera_profile> \
  --tag-size-m <tag_size_m> \
  --min-decision-margin <margin> \
  --quad-decimate <decimate> \
  --duration 10
```

For a newly connected camera **before calibration exists**, first establish that
capture itself works using the available camera setup/smoke-test path. Do not interpret
PnP distance from a calibration belonging to another camera.

The validator becomes a geometry test only after the new camera has its own
calibration profile.

---

## 11.2 Determine capture resolution separately from processing resolution

These are different choices:

```text
USB capture resolution
        ↓
optional software resize
        ↓
AprilTag processing resolution
```

For example, a camera may capture at a larger native mode but process a smaller frame
to reduce CPU load.

When selecting the pair:

1. preserve the intended aspect ratio unless cropping is deliberate;
2. verify the resize is not accidentally changing the useful FOV;
3. calibrate the geometry actually used by the AprilTag/PnP path;
4. record both capture and processing dimensions in the camera profile.

Do not assume that "higher capture resolution" automatically means better AprilTag
performance. Test it.

---

## 11.3 Test candidate processing resolutions

Keep the physical camera mode and exposure fixed while changing only the processing
resolution.

Example command shape:

```bash
PYTHONPATH=. python3 tools/setup/cameras/camera_apriltag_validate.py \
  --backend usb \
  --device "$DEVICE" \
  --capture-width <fixed_capture_width> \
  --capture-height <fixed_capture_height> \
  --processing-width <candidate_width> \
  --processing-height <candidate_height> \
  --fps <fixed_fps> \
  --format <fixed_format> \
  --calibration-module calibration.cameras.<matching_calibration> \
  --tag-size-m <tag_size_m> \
  --min-decision-margin <fixed_margin> \
  --quad-decimate <fixed_decimate> \
  --duration 10
```

Change only one controlled variable between runs.

If a matching calibration does not exist for the candidate processing geometry,
detection count and decision margin may still be useful diagnostically, but **PnP
distance/bearing must not be treated as valid**.

---

## 11.4 Test detector decimation

Once camera mode, exposure and calibration are fixed, compare detector load versus
range using `--quad-decimate`.

Example:

```bash
--quad-decimate 1.5
```

then:

```bash
--quad-decimate 1.0
```

Keep everything else unchanged.

Lower decimation means more detector pixels and generally more CPU work. It is not
automatically better; choose it from measured detection performance and processing
rate.

---

## 11.5 Use short and long runs for different purposes

Use a short run while iterating:

```bash
--duration 10
```

Use a longer run when a candidate configuration looks promising:

```bash
--duration 60
```

The longer run is useful for confirming:

- sustained processing rate;
- intermittent detection loss;
- stability over time.

---

## 11.6 Troubleshooting order when the Python test fails

Work downward rather than changing random Python settings:

```text
1. Is DEVICE still set correctly?
2. Does ls -l "$DEVICE" resolve?
3. Does v4l2-ctl --list-devices show the camera?
4. Does --list-formats-ext advertise the requested mode?
5. Does --all show sensible camera state?
6. Do the important controls read back correctly?
7. Does OpenCV import?
8. Does pupil_apriltags import?
9. Does the standalone camera capture path work?
10. Does the calibration match this camera/lens/processing geometry?
11. Only then debug the AprilTag validator or runtime integration.
```

This sequence separates:

```text
USB problem
device-path problem
driver/V4L2 problem
capture-mode problem
control-state problem
Python dependency problem
calibration problem
AprilTag problem
runtime problem
```

---

# 12. Full-FOV Verification

Resolution numbers alone do not tell you whether a camera is using its full useful
field of view.

After selecting a candidate capture mode:

1. place the camera in a fixed position;
2. note objects at the left, right, top and bottom edges;
3. capture or preview the native mode;
4. compare it with the intended processing frame;
5. repeat for other candidate capture modes if necessary.

If capture and processing use the same aspect ratio, a simple resize should not
intentionally crop the frame. However, the camera firmware itself may expose different
sensor crops for different modes.

This matters for AprilTags because field of view and pixels-per-degree trade against
one another:

```text
wider FOV at the same processing width
    → fewer pixels per degree
    → smaller tag image at a given distance

narrower FOV at the same processing width
    → more pixels per degree
    → larger tag image at the same distance
```

Therefore, when comparing candidate modes or cameras, do not judge only by resolution.
Verify actual scene boundaries and then measure AprilTag performance.

Once the final optical/capture/processing combination is chosen, proceed to
calibration. Do not calibrate one mode and silently switch to another afterward.

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

The common calibration tool is:

```text
tools/setup/cameras/camera_calibrate.py
```

Current commands are:

```bash
python3 tools/setup/cameras/camera_calibrate.py --capture --preview
```

and:

```bash
python3 tools/setup/cameras/camera_calibrate.py --solve
```

Capture without the preview option can also be requested where supported:

```bash
python3 tools/setup/cameras/camera_calibrate.py --capture
```

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

This tool is intended to be common across camera interfaces. The calibration solve
logic is camera-independent; any remaining interface-specific capture code should be
kept temporary while the Pi and USB capture paths are unified behind the configured
camera/backend.

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
- decision margin;
- measured distance;
- bearing;
- vertical angle where used;
- motion behaviour;
- lighting sensitivity;
- edge-of-frame behaviour;
- actual processing rate.

## 18.1 USB baseline validation command

For the current OV9281 wide-lens configuration:

```bash
PYTHONPATH=. python3 tools/setup/cameras/camera_apriltag_validate.py \
  --backend usb \
  --device "$DEVICE" \
  --capture-width 1280 \
  --capture-height 800 \
  --processing-width 640 \
  --processing-height 400 \
  --fps 30 \
  --format MJPG \
  --calibration-module calibration.cameras.arducam_fullfov_640_400 \
  --tag-size-m 0.08 \
  --min-decision-margin 15 \
  --quad-decimate 1.5 \
  --duration 10
```

The current preferred processing baseline is therefore:

```text
capture: 1280×800 MJPG @ 30
processing: 640×400
quad_decimate: 1.5
minimum decision margin: 15
```

## 18.2 Pi Camera 3 comparison command

```bash
PYTHONPATH=. python3 tools/setup/cameras/camera_apriltag_validate.py \
  --backend pi \
  --capture-width 640 \
  --capture-height 360 \
  --processing-width 640 \
  --processing-height 360 \
  --fps 30 \
  --calibration-module calibration.cameras.pi3_fullfov_640_360 \
  --tag-size-m 0.08 \
  --min-decision-margin 15 \
  --quad-decimate 1.5 \
  --duration 10
```

The direct Pi validator may select a different internal libcamera sensor mode while still delivering the requested 640×360 processing frame. When exact raw sensor-mode equivalence matters, inspect the validator startup output rather than assuming the internal mode from the processing resolution.

## 18.3 Troubleshooting detector resolution / decimation

When a tag is weak or long-range behaviour is uncertain, change one processing variable at a time.

Baseline:

```text
640×400, quad_decimate=1.5
```

Higher detector workload at the same processing resolution:

```bash
PYTHONPATH=. python3 tools/setup/cameras/camera_apriltag_validate.py \
  --backend usb \
  --device "$DEVICE" \
  --capture-width 1280 \
  --capture-height 800 \
  --processing-width 640 \
  --processing-height 400 \
  --fps 30 \
  --format MJPG \
  --calibration-module calibration.cameras.arducam_fullfov_640_400 \
  --tag-size-m 0.08 \
  --min-decision-margin 15 \
  --quad-decimate 1.0 \
  --duration 10
```

Full processing resolution was also used diagnostically:

```bash
PYTHONPATH=. python3 tools/setup/cameras/camera_apriltag_validate.py \
  --backend usb \
  --device "$DEVICE" \
  --capture-width 1280 \
  --capture-height 800 \
  --processing-width 1280 \
  --processing-height 800 \
  --fps 30 \
  --format MJPG \
  --calibration-module calibration.cameras.arducam_fullfov_640_400 \
  --tag-size-m 0.08 \
  --min-decision-margin 15 \
  --quad-decimate 2.0 \
  --duration 10
```

and:

```bash
PYTHONPATH=. python3 tools/setup/cameras/camera_apriltag_validate.py \
  --backend usb \
  --device "$DEVICE" \
  --capture-width 1280 \
  --capture-height 800 \
  --processing-width 1280 \
  --processing-height 800 \
  --fps 30 \
  --format MJPG \
  --calibration-module calibration.cameras.arducam_fullfov_640_400 \
  --tag-size-m 0.08 \
  --min-decision-margin 15 \
  --quad-decimate 1.5 \
  --duration 10
```

**Important:** the current Arducam calibration module is for the 640×400 processing geometry. Therefore PnP distances from the 1280×800 diagnostic runs are not valid. Those runs are useful for detection rate, decision margin, and processing-rate comparisons only unless a matching 1280×800 calibration profile is created.

## 18.4 Use longer runs when checking stability

For a sustained check, change only the duration:

```text
--duration 60
```

This is useful for checking intermittent camera/detector behaviour while leaving the actual vision configuration unchanged.

---

# 19. Current Detector Tuning

Current comparison baseline:

```text
FAMILIES = tag36h11
MIN_DECISION_MARGIN = 15
QUAD_DECIMATE = 1.5
NTHREADS = 2
QUAD_SIGMA = 0.0
REFINE_EDGES = 1
DECODE_SHARPENING = 0.25
```

The currently preferred Arducam operating point is:

```text
capture: 1280×800 MJPG @ 30 FPS
processing: 640×400
quad_decimate: 1.5
minimum decision margin: 15
manual exposure: 75
gain: 0
```

The decimation experiments established that simply increasing detector work is not automatically better. `640×400 / 1.5` held about 30 FPS and remains the preferred baseline; `640×400 / 1.0` reduced processing rate without demonstrating a consistent detection-margin advantage in the comparison runs.

Lighting changed between several tests, so decision margins from runs performed under visibly different lighting should not be treated as controlled algorithmic comparisons. Prefer back-to-back tests where only one setting changed.

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

For a new or troublesome USB camera, use this sequence from the outside in:

```bash
v4l2-ctl --list-devices
```

```bash
ls -l /dev/v4l/by-id/
```

```bash
DEVICE=/dev/v4l/by-id/<camera-video-index0>
```

```bash
ls -l "$DEVICE"
```

```bash
v4l2-ctl -d "$DEVICE" --list-formats-ext
```

```bash
v4l2-ctl -d "$DEVICE" --list-ctrls-menus
```

```bash
v4l2-ctl -d "$DEVICE" --all
```

Then query the exact controls discovered for that camera:

```bash
v4l2-ctl -d "$DEVICE" --get-ctrl=<control_name>
```

Set only the control being tested:

```bash
v4l2-ctl -d "$DEVICE" --set-ctrl=<control_name>=<candidate>
```

and read it back:

```bash
v4l2-ctl -d "$DEVICE" --get-ctrl=<control_name>
```

Finally, validate the selected capture configuration through the standalone Python
camera/AprilTag tools before involving the robot runtime.

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

Current camera setup tool layout:

```text
tools/setup/cameras/
    camera_calibrate.py
    camera_apriltag_validate.py

    pi/
        camera_smoke_test.py
        camera_state.py
        camera_test_configs.py
        camera_config_tests.py

        focus/
            README.md
            autofocus_baseline.py
            test_focus_values.py
            set_fixed_focus.py

    usb/
        ...
```

Pi-specific files remain under `pi/` because they depend on Picamera2/libcamera.
USB-specific setup tools belong under `usb/`. Common tools remain directly under
`tools/setup/cameras/`.

---

## USB camera — new-camera procedure

```text
1. connect camera
2. v4l2-ctl --list-devices
3. ls -l /dev/v4l/by-id/
4. set DEVICE to the stable capture path
5. verify DEVICE resolves
6. v4l2-ctl --list-formats-ext
7. record candidate format / resolution / FPS combinations
8. v4l2-ctl --list-ctrls-menus
9. record the untouched control state
10. choose one candidate capture mode
11. verify Python/OpenCV capture
12. choose processing resolution
13. determine focus strategy if the camera supports focus
14. tune exposure for motion performance
15. tune gain only as required
16. verify all requested controls by reading them back
17. verify full FOV / crop behaviour
18. fix the optical and image configuration
19. capture calibration images
20. solve calibration
21. store a camera-specific calibration profile
22. run standalone AprilTag validation
23. test useful ranges and motion
24. compare detector settings one variable at a time
25. record the successful values in the reusable camera profile
26. integrate the logical camera into the robot profile/runtime
27. re-check control persistence after reboot/power-cycle
```

The result of this procedure should be a camera profile discovered from the hardware,
not a copy of another camera's settings.

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
