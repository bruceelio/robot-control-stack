3rdparty/cameras/Pi3/README_PI3 .md

# Pi Camera 3

# ⚠️ CRITICAL: Virtual Environment Setup

Picamera2 is installed via system packages (APT), not pip.

If you use a Python virtual environment, it MUST be created with:

```bash
python3 -m venv --system-site-packages ~/apriltag-env
```

Otherwise, you will get:

ModuleNotFoundError: No module named 'picamera2'

# Required setup

Activate environment:

```bash
source ~/apriltag-env/bin/activate
```
Verify:

```bash
python3 -c "from picamera2 import Picamera2; print('OK')"
python3 -c "from pupil_apriltags import Detector; print('OK')"
```


---

# Camera Configuration (Target vs Observed)

| Parameter     | Target             | Observed (from `read_camera_state.py`) |
| ------------- | ------------------ | -------------------------------------- |
| Resolution    | 640×480            | (verify via config / metadata)         |
| Frame rate    | 30 FPS             | FrameDuration ≈ 33333 µs               |
| Exposure      | Manual             | AeEnable = False                       |
| Shutter       | ~1/150 – 1/250     | ExposureTime ≈ 4000–7000 µs            |
| Gain          | Moderate           | AnalogueGain ≈ 1.0–4.0                 |
| White balance | Fixed              | AwbEnable = False + ColourGains set    |
| Focus         | Manual             | AfMode = 0                             |
| LensPosition  | (chosen, e.g. 1.2) | LensPosition ≈ chosen value            |

⚠️ Always verify actual values using:

```bash
python3 read_camera_state.py
```


---

# Critical Rules

### 1. Fix camera mode first

These must be fixed before all testing and calibration:

* resolution
* frame rate

### 2. Focus must be fixed before calibration

Changing focus after calibration can invalidate results.

### 3. Keep the optical setup constant

Do not change:

* camera mounting
* lens position
* resolution

after calibration.

---

# Preview Modes (Important)

All scripts support:

```bash
--preview drm   # show live camera on Pi monitor (recommended)
--preview none  # no preview (robot runtime mode)
--preview save  # save frames to disk for debugging
```

### Recommended usage

| Use Case                   | Mode   |
| -------------------------- | ------ |
| Development (with monitor) | `drm`  |
| Robot runtime              | `none` |
| Debugging / logging        | `save` |

---

# Overall Workflow

---

## Phase 1 — Camera Bring-Up

### Step 1: Smoke Test

```bash
python3 test_camera.py --preview drm
```

Verify:

* camera opens
* frames stream
* preview appears on Pi monitor

---

## Phase 2 — Fix Camera Mode

Set the camera to the required resolution and frame rate.

```bash
python3 set_camera_mode.py
```

This configures:

* Resolution = 640×480
* Frame rate ≈ 30 FPS

Verify:

* printed frame duration ≈ 33333 µs

### Why this matters

These must remain constant for:

* focus testing
* calibration
* AprilTag testing
* robot runtime

Changing them later can invalidate calibration.

---

## Phase 3 — Focus Setup (Single Fixed Focus)

See:

```
focus/README.md
```

### Process

1. Run autofocus baseline:

```bash
python3 focus/focus_autofocus_baseline.py
```

2. Test candidate values:

```bash
python3 focus/test_focus_values.py
```

3. Choose best value based on:

* object detection at 1–2 m
* stable tracking to ~50 cm
* acceptable wall-tag detection

4. Lock focus:

```bash
python3 focus/set_fixed_focus.py
```

### Rules

* Do NOT use autofocus after this step
* Do NOT change focus later

---

## Phase 4 — Camera Configuration Testing

After focus is fixed, tune:

* shutter
* gain
* white balance

### Edit candidate configurations

```
camera_test_configs.py
```

### Run tests

```bash
python3 run_camera_config_tests.py --preview drm
```

### Optional debug mode

```bash
python3 run_camera_config_tests.py --preview save
```

For each configuration, evaluate:

* AprilTag detection reliability
* pose stability
* motion blur
* brightness consistency

---

## Phase 5 — Calibration

## Phase 5 — Calibration

### Goal

Measure the camera intrinsics for the **exact setup you will use**:

* resolution fixed at **640×480**
* frame rate fixed at **30 FPS**
* focus already chosen and locked
* camera mounted in its final robot position

The calibration script is designed to:

* capture chessboard images from the Pi camera
* solve for `fx, fy, cx, cy`
* print values to copy into `calibration/cameras/pi3_640_480.py` 

---

### Chessboard needed

Use a **printed calibration chessboard** with:

* **9 columns × 6 rows of inner corners**
* square size known in the real world, for example **25 mm**

Those are the current defaults in `calibrate_pi_camera.py`:

* `--pattern-cols 9`
* `--pattern-rows 6`
* `--square-size-mm 25.0` 

#### Important detail

`pattern-cols` and `pattern-rows` are the number of **inner corners**, not the number of black/white squares.

So a “9×6 inner-corner” chessboard will have:

* **10 squares across**
* **7 squares down**

If you print a different board, change the script arguments to match it.

---

### Where to get the chessboard

You have three practical options:

1. Print a standard **camera calibration chessboard** on plain paper
2. Glue or tape it to a **flat rigid backing** like cardboard or foam board
3. Use a commercially printed calibration board if you already have one

The important things are:

* the board must be **flat**
* the squares must be **sharp and high contrast**
* you must know the real square size in **mm**

A wrinkled sheet of paper is not ideal. Flatness matters.

---

### Board size recommendation

For your setup, a good starting point is:

* **A4 or Letter sized print**
* square size around **20–30 mm**

That is usually big enough to fill a reasonable part of the frame at short-to-medium distances.

If you use the default script value:

```bash
--square-size-mm 25
```

then each square should actually be about **25 mm wide**.

---

### Before capturing images

Make sure all of these are already fixed:

* focus locked
* camera mode fixed to 640×480
* camera physically mounted
* intended lighting reasonably representative

Do **not** recalibrate with autofocus still active. The README and focus workflow both assume focus is fixed before calibration. 

---

### How far away should the chessboard be?

Use a range of distances, not just one.

For your camera and 640×480 mode, a good practical range is:

* **about 20 cm to 100 cm**

The board should usually occupy somewhere between:

* about **30% to 80%** of the frame

#### Avoid:

* extremely tiny board in the image
* board filling the entire image every time
* only one distance
* only perfectly straight-on views

---

### What views to capture

You want a **variety** of good images.

Capture images with the board:

* centered
* near each corner of the frame
* tilted left/right
* tilted up/down
* at several distances
* with different rotations

This is important because calibration needs corner observations across the image, not just in the center.

A good rough target is:

* **15 to 25 good images**

The script already warns that fewer than 15 images may be weak, and requires at least 5 usable images to solve.  

---

### Practical capture advice

For each image:

* keep the chessboard fully visible
* avoid motion blur
* avoid glare on the paper
* make sure the board is flat
* try to hold the board still for a moment before capture

If the board is not detected consistently:

* improve lighting
* move closer
* make the board larger in frame
* reduce blur
* check that the printed pattern matches the script settings

---

### Capture command

```bash
python3 calibrate_pi_camera.py --capture --preview
```

The calibration script supports interactive capture and shows live feedback during capture. It looks for the chessboard and lets you save images when detected. 

If you are using a non-GUI workflow, adapt this step to your preview/debug method. The key requirement is still the same: only save images where the chessboard is clearly detected.

---

### Solve command

```bash
python3 calibrate_pi_camera.py --solve
```

This will:

* load saved calibration images
* detect chessboard corners
* refine the corner positions
* solve camera calibration
* print `fx, fy, cx, cy`
* print distortion coefficients 

---

### What “good” calibration looks like

After solving, look at:

* number of images used
* RMS reprojection error
* mean reprojection error

Lower is better.

As a rough practical guide:

* **very low error** = good
* **large error** = likely weak image set, bad board coverage, blur, or incorrect board dimensions

If the error seems poor:

* capture more images
* add more tilted/off-center views
* make sure the square size is correct
* make sure focus stayed fixed

---

### Where to copy the result

Copy the printed result into:

```text
calibration/cameras/pi3_640_480.py
```

The script prints the exact `CAMERA_PARAMS = (...)` line to copy. 

---

### Rules

* focus must remain fixed
* resolution must not change
* camera mount must not move
* use the same optical setup later for AprilTag testing and runtime

If focus or resolution changes after calibration, the calibration may no longer be valid. 

---
Simple capture routine (do this)

Repeat this sequence:

Center, straight → capture
Move to left edge → capture
Move to right edge → capture
Tilt left → capture
Tilt right → capture
Move closer → capture
Move farther → capture

👉 Do this ~3 times → you’ll have ~20 good images


### Recommended calibration checklist

1. Lock focus first
2. Confirm focus with `read_camera_state.py`
3. Print or prepare a flat **9×6 inner-corner** chessboard
4. Confirm square size in mm
5. Capture **15–25** varied images
6. Solve calibration
7. Copy results into `calibration/cameras/pi3_640_480.py`
8. Run AprilTag validation with those intrinsics


### Capture images

```bash
python3 calibrate_pi_camera.py --capture
```

⚠️ If preview is needed during calibration, use saved frames instead of GUI.

### Solve calibration

```bash
python3 calibrate_pi_camera.py --solve
```

Copy output into:

```
calibration/cameras/pi3_640_480.py
```

### Rules

* focus must remain fixed
* resolution must not change
* camera must not move

---

## Phase 6 — AprilTag Validation

### Basic test

```bash
python3 apriltag_pi3_test.py --preview drm
```

### With calibration (replace with actual values)

```bash
python3 apriltag_pi3_test.py --preview drm --fx 950 --fy 950 --cx 320 --cy 240 --tag-size-m 0.08
```

### Debug frame capture

```bash
python3 apriltag_pi3_test.py --preview save
```

Verify:

* stable detection
* correct IDs
* reasonable distances
* low pose jitter

---

## Phase 7 — Integration

Only after validation:

* integrate into robot pipeline
* test with motion
* run full system

---

# Configuration Verification Tool

At any time, verify camera state:

```bash
python3 read_camera_state.py
```

This reports:

* exposure
* gain
* white balance
* lens position
* frame timing

Use this:

* before calibration
* before competition
* when debugging

---

# Recommended Focus Strategy

Robot behavior:

* detect object at ~1–2 m
* approach to ~50 cm
* lose tag at ~15 cm
* finish grab blindly

👉 Therefore:

Optimize focus for:

* **0.5 m to 1.0 m range**

Typical result:

* `LensPosition ≈ 1.0 – 1.5`

---

# Typical Full Workflow

```
1. test_camera.py --preview drm
2. set_camera_mode.py
3. focus/focus_autofocus_baseline.py
4. focus/test_focus_values.py
5. focus/set_fixed_focus.py
6. edit camera_test_configs.py
7. run_camera_config_tests.py --preview drm
8. calibrate_pi_camera.py --capture
9. calibrate_pi_camera.py --solve
10. apriltag_pi3_test.py --preview drm --fx ...
11. integrate into robot
```

---

# When to Redo Setup

Redo focus and calibration if:

* camera mount changes
* lens is disturbed
* resolution changes
* working distance changes significantly
* camera module is replaced

---

# Final Rule

**Autofocus is for setup only.
Manual focus is required for calibration and runtime.**

