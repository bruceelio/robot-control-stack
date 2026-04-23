config/cameras/README_CAMERA_TROUBLESHOOTING.md

# Pi Camera 3 / AprilTag Troubleshooting Table

Use this table when tuning a camera profile for AprilTag detection and localisation.

## How to use it

- Start with the symptom you see most clearly.
- Change **one setting at a time**.
- Re-test after each change.
- Prioritise settings marked **High** before touching **Medium** or **Low** ones.

## Importance scale

- **High** = often one of the main causes
- **Medium** = can help, but usually secondary
- **Low** = only matters in specific cases
- **—** = usually not relevant for that symptom

---

| Setting | What it controls | Too low does this | Too high does this | Blurry image | Can't see far tags | Can't see near tags | Tags flicker in/out | Motion blur | Image too dark | Image too noisy | Backlighting / window problems | Pose unstable / jumpy | False positives | Speed too slow |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `WIDTH`, `HEIGHT` | Output frame size used by detector | Too little detail per tag | More CPU load / slower | Medium | **High** | Medium | Medium | — | — | — | — | Medium | Low | **High** |
| `FPS` | Frame timing target | More exposure time possible but slower updates | Shorter exposure budget, darker image | Low | Low | Low | Medium | Medium | Medium | Low | Low | Low | — | Medium |
| `SENSOR_OUTPUT_SIZE` | Sensor mode / full FoV path | Cropped FoV, fewer tags visible | More processing / bandwidth | Medium | **High** | Medium | **High** | — | — | — | — | **High** | — | Medium |
| `SENSOR_BIT_DEPTH` | Sensor dynamic range | Less tonal range | More processing cost | Low | Low | Low | Low | — | Medium | Low | Medium | Low | — | Low |
| `FORCE_FULL_SENSOR_SCALER_CROP` | Uses full sensor area for widest FoV | Narrow FoV / hidden crop | Wider view but smaller tags in frame | Medium | **High** | Medium | **High** | — | — | — | — | **High** | — | Low |
| `MIN_DECISION_MARGIN` | Rejects weak detections | More weak/noisy detections accepted | Drops distant / weak tags | — | **High** | Medium | **High** | — | Medium | Medium | Medium | Medium | **High** | Low |
| `QUAD_DECIMATE` | Detection speed vs small-tag detail | Slower but keeps detail | Faster but loses small/far tags | — | **High** | Medium | **High** | — | — | — | Low | Medium | Low | **High** |
| `NTHREADS` | CPU threads for AprilTag detector | Slower | Little extra benefit after a point | — | — | — | — | — | — | — | — | — | — | **High** |
| `QUAD_SIGMA` | Blur before quad detection | More raw detail, less noise smoothing | Softens edges and small tags | Medium | Medium | Low | Medium | Low | Low | Medium | Low | Medium | Low | Low |
| `REFINE_EDGES` | Corner/edge precision | Less accurate corners | Slightly more CPU | Low | Medium | Medium | Medium | — | — | — | — | **High** | Low | Low |
| `DECODE_SHARPENING` | Sharpens decode stage | Flat/low-contrast decode | Can create artifacts / noise | Medium | Medium | Low | Medium | — | Low | Medium | Medium | Low | Medium | Low |
| `AF_MODE` | Autofocus behaviour | N/A | N/A | **High** if not manual | **High** | **High** | **High** | Medium | — | — | Low | **High** | — | — |
| `LENS_POSITION` | Manual focus distance | Distant tags blurry | Near tags blurry | **High** | **High** | **High** | **High** | Low | — | — | Low | **High** | — | — |
| `AE_ENABLE` | Auto exposure on/off | If off, can be wrong for scene | If on, brightness may vary frame to frame | Medium | Medium | Medium | **High** | Medium | **High** | Medium | **High** | Medium | Low | Low |
| `EXPOSURE_TIME_US` | Brightness vs blur | Darker but sharper | Brighter but blurrier | **High** | Medium | Medium | Medium | **High** | **High** | Low | **High** | Medium | Low | Low |
| `ANALOGUE_GAIN` | Brightness amplification | Darker image | Noisier image | Medium | Medium | Medium | Medium | — | **High** | **High** | Medium | Medium | Medium | — |
| `AWB_ENABLE` | Auto white balance | If off and wrong, color tone fixed | If on, frame-to-frame color changes | Low | Low | Low | Low | — | Low | Low | Medium | Low | Low | — |
| `COLOUR_GAINS` | Fixed white balance gains | Wrong tint / grayscale balance | Wrong tint / grayscale balance | Low | Low | Low | Low | — | Low | Low | Medium | Low | Low | — |
| `CALIBRATION_PROFILE` | Intrinsics for bearing / distance / pose | Wrong geometry | Wrong geometry | — | — | — | — | — | — | — | — | **High** | — | — |

---

## Symptom-first tuning advice

### 1. Blurry image
Prioritise:
1. `LENS_POSITION`
2. `AF_MODE`
3. `EXPOSURE_TIME_US`

Notes:
- If only far tags are blurry, increase `LENS_POSITION` slightly.
- If only near tags are blurry, decrease `LENS_POSITION` slightly.
- If blur happens only during motion, reduce `EXPOSURE_TIME_US`.

---

### 2. Can't see far tags
Prioritise:
1. `FORCE_FULL_SENSOR_SCALER_CROP`
2. `SENSOR_OUTPUT_SIZE`
3. `WIDTH`, `HEIGHT`
4. `QUAD_DECIMATE`
5. `MIN_DECISION_MARGIN`
6. `LENS_POSITION`

Notes:
- Full FoV helps see more distant tags in frame.
- Higher output resolution helps each distant tag occupy more pixels.
- Lower `QUAD_DECIMATE` helps preserve small-tag detail.
- Lower `MIN_DECISION_MARGIN` may recover weak detections.

---

### 3. Can't see near tags
Prioritise:
1. `LENS_POSITION`
2. `EXPOSURE_TIME_US`
3. `WIDTH`, `HEIGHT`

Notes:
- Near-tag failure is often a focus issue, not a detector issue.
- If the robot is moving while close, motion blur can also dominate.

---

### 4. Tags flicker in and out
Prioritise:
1. `LENS_POSITION`
2. `MIN_DECISION_MARGIN`
3. `QUAD_DECIMATE`
4. `AE_ENABLE`
5. `EXPOSURE_TIME_US`

Notes:
- Flicker often means detections are just above/below threshold.
- Backlighting and auto exposure can make this much worse.

---

### 5. Motion blur
Prioritise:
1. `EXPOSURE_TIME_US`
2. `FPS`
3. `ANALOGUE_GAIN`

Notes:
- Lower exposure first.
- If image gets too dark after lowering exposure, increase gain slightly.

---

### 6. Image too dark
Prioritise:
1. `EXPOSURE_TIME_US`
2. `ANALOGUE_GAIN`
3. `AE_ENABLE`

Notes:
- Increase exposure before gain if robot is mostly stationary.
- Increase gain before exposure if robot is moving and blur matters.

---

### 7. Image too noisy
Prioritise:
1. `ANALOGUE_GAIN`
2. `EXPOSURE_TIME_US`
3. `QUAD_SIGMA`

Notes:
- High gain is usually the first suspect.
- Slightly longer exposure may be better than excessive gain.

---

### 8. Backlighting / bright window behind target
Prioritise:
1. `AE_ENABLE`
2. `EXPOSURE_TIME_US`
3. `ANALOGUE_GAIN`
4. `MIN_DECISION_MARGIN`

Notes:
- Backlighting often crushes tag contrast.
- Fixed manual exposure usually behaves more consistently than auto.
- Try physically changing the scene first if possible.

---

### 9. Pose unstable / jumpy
Prioritise:
1. `CALIBRATION_PROFILE`
2. `LENS_POSITION`
3. `REFINE_EDGES`
4. `FORCE_FULL_SENSOR_SCALER_CROP`
5. `SENSOR_OUTPUT_SIZE`

Notes:
- If IDs are stable but pose jumps, calibration is often the problem.
- Poor focus also makes corner estimates unstable.

---

### 10. False positives
Prioritise:
1. `MIN_DECISION_MARGIN`
2. `DECODE_SHARPENING`
3. `QUAD_SIGMA`

Notes:
- Raise `MIN_DECISION_MARGIN` first.
- Over-sharpening can sometimes make decode noisier.

---

### 11. Speed too slow
Prioritise:
1. `QUAD_DECIMATE`
2. `WIDTH`, `HEIGHT`
3. `NTHREADS`
4. `SENSOR_OUTPUT_SIZE`

Notes:
- Increase `QUAD_DECIMATE` before dropping sensor mode.
- Reduce output resolution only if needed.
- Increasing threads helps, but only up to a point.

---

## Good tuning order in the field

1. `LENS_POSITION`
2. `EXPOSURE_TIME_US`
3. `ANALOGUE_GAIN`
4. `FORCE_FULL_SENSOR_SCALER_CROP` / `SENSOR_OUTPUT_SIZE`
5. `WIDTH`, `HEIGHT`
6. `QUAD_DECIMATE`
7. `MIN_DECISION_MARGIN`
8. `CALIBRATION_PROFILE`

---

## Practical defaults

### Reliable baseline
- `QUAD_DECIMATE = 1.5`
- `MIN_DECISION_MARGIN = 20.0`

### Long-range / weak-tag mode
- `QUAD_DECIMATE = 1.0`
- `MIN_DECISION_MARGIN = 12.0–18.0`

### Faster / less reliable mode
- `QUAD_DECIMATE = 2.0`
- `MIN_DECISION_MARGIN = 20.0+`

## Competition Quick Reference

Use this when you need to make fast decisions during a match or between rounds.

| Setting | Competition quick advice | When to change it |
|---|---|---|
| `LENS_POSITION` | Change this first if tags look soft or detection distance is poor. Small steps only. | Far tags blurry, near tags blurry, detections flicker even when lighting is stable. |
| `AF_MODE` | Prefer `manual` for competition. Avoid autofocus unless doing setup only. | Only change if focus is clearly hunting or not locked. |
| `EXPOSURE_TIME_US` | Lower it to reduce motion blur. Raise it only if image is too dark. | Tags disappear while moving, image smeared, or image too dark. |
| `ANALOGUE_GAIN` | Raise slightly if image is too dark. Keep as low as possible to avoid noise. | Dark image after lowering exposure, noisy image, unstable weak detections. |
| `AE_ENABLE` | Usually keep `False` for repeatability. Turn on only if lighting changes wildly and you cannot tune manually in time. | Big lighting swings between field areas, severe backlighting, manual settings unusable. |
| `AWB_ENABLE` | Usually keep `False`. White balance is less important than focus/exposure for tags. | Only change if grayscale contrast seems oddly poor under certain lights. |
| `COLOUR_GAINS` | Leave fixed once chosen. Change only if lighting color has changed a lot. | New venue lighting, strange color cast, poor contrast despite good brightness. |
| `FORCE_FULL_SENSOR_SCALER_CROP` | Keep `True` when wide FoV is needed for localisation. | Robot cannot see enough tags at once, FoV feels too narrow. |
| `SENSOR_OUTPUT_SIZE` | Use the known-good full-FoV mode unless performance forces a fallback. | Need wider FoV, or current mode is cropping too much. |
| `WIDTH`, `HEIGHT` | Raise for more tag detail, lower for more speed. | Distant tags too small, CPU too slow, frame rate too low. |
| `FPS` | Usually leave at 30. Lower only if you need more exposure budget. | Image too dark and manual exposure is already high, or motion tracking needs smoother updates. |
| `QUAD_DECIMATE` | Lower for more range/detail, raise for speed. | Small/far tags not detected, CPU overloaded. |
| `MIN_DECISION_MARGIN` | Lower to recover weak tags, raise to suppress false positives. | Tags flicker in/out, weak distant tags missed, or false detections appear. |
| `NTHREADS` | Increase a bit if CPU allows; do not expect miracles. | Detection too slow on Pi 4, but image quality is otherwise fine. |
| `QUAD_SIGMA` | Usually leave at `0.0`. | Only try changing if image is very noisy and sharp edges are still present. |
| `REFINE_EDGES` | Usually leave enabled. | Only change if desperate for speed. |
| `DECODE_SHARPENING` | Small tweaks only. Too much can hurt. | Low-contrast tags remain hard to decode after focus/exposure are good. |
| `CALIBRATION_PROFILE` | Must match the active mode. Do not forget this when changing resolution/FoV. | Pose/bearing/distance seem wrong even though IDs are stable and image is sharp. |

### Fast tuning order under pressure

1. `LENS_POSITION`
2. `EXPOSURE_TIME_US`
3. `ANALOGUE_GAIN`
4. `QUAD_DECIMATE`
5. `MIN_DECISION_MARGIN`
6. `WIDTH`, `HEIGHT`
7. `CALIBRATION_PROFILE`

### Rule of thumb

- **Bad image** → fix focus/exposure/gain first
- **Good image but weak detections** → tune `QUAD_DECIMATE` / `MIN_DECISION_MARGIN`
- **Good detections but bad pose** → fix `CALIBRATION_PROFILE`