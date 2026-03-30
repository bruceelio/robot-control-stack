3rdparty/cameras/Pi3/focus/README.md

# Camera Focus Setup Procedure for AprilTag Use

This procedure is intended for a robot that:
- uses AprilTags on objects to select the correct pickup target
- uses wall AprilTags for navigation / pose estimation
- will use **one fixed focus setting** during calibration and runtime

The goal is to choose a single focus value that works well for:
- initial object detection at about **1 to 2 m**
- guided approach down to about **50 cm**
- stable wall-tag detection for navigation

The final **blind grab** at about **15 cm** is **not** the focus priority, because the tag is expected to be lost by then.

---

## Summary of the approach

1. Mount the camera in its final robot position.
2. Use autofocus once to get a sensible baseline focus value.
3. Test a few nearby fixed manual focus values.
4. Pick the value that gives the best AprilTag performance.
5. Lock focus to that value.
6. Calibrate the camera with that focus fixed.
7. Use the same focus value during testing and competition.

Do **not** leave autofocus enabled during calibration or normal operation.

---

## Files in this package

- `focus_autofocus_baseline.py`  
  Runs autofocus once and prints the resulting `LensPosition`.

- `set_fixed_focus.py`  
  Sets a chosen fixed focus value in manual mode.

- `test_focus_values.py`  
  Lets you quickly test several fixed focus values one by one.

---

## Step 1: Mount the camera in its final position

Mount the camera exactly as it will be used on the robot.

This matters because calibration assumes a fixed optical setup. If the mount changes later, the effective camera geometry can change enough to reduce pose accuracy.

Checklist:
- camera module fully secured
- final camera angle chosen
- final robot mounting position chosen
- no loose cable strain pulling on the camera

---

## Step 2: Prepare a realistic focus test scene

Set up a test scene that matches real use as closely as possible.

Include:
- an AprilTag on an object at about **1 to 2 m**
- another AprilTag or target at about **50 cm to 1 m**
- a wall tag at a few metres if possible
- normal competition-like lighting if possible

The point is not to make the picture look nice. The point is to choose the focus that gives the most reliable **AprilTag detection and pose** over the useful operating range.

---

## Step 3: Run autofocus once to get a baseline

Run:

```bash
python3 focus_autofocus_baseline.py
```

What this does:
- starts the camera
- enables autofocus
- lets autofocus settle
- triggers an autofocus scan
- reads back the chosen `LensPosition`
- prints that value

Write down the reported value.

Example:

```text
Suggested baseline LensPosition: 1.27
```

That value is your starting point. It is **not automatically the final value**.

---

## Step 4: Choose values to test

Use the autofocus result as your centre point.

Recommended starting test values for this robot are usually around:
- `1.0`
- `1.33`
- `2.0`

If autofocus gives a value near one of these, test values around it.

Examples:
- if autofocus gives `1.2`, test `1.0`, `1.2`, `1.4`
- if autofocus gives `1.4`, test `1.1`, `1.4`, `1.7`

Do **not** try to compute the best value by extrapolation alone. Test actual AprilTag performance.

---

## Step 5: Test fixed focus values

Run:

```bash
python3 test_focus_values.py --preview drm
```

This script will apply one value at a time and pause so you can test it.

For each focus value, check:

### Object-tag acquisition
- can the robot detect the object tag reliably at **1 to 2 m**?
- can it identify the correct object consistently?

### Guided approach
- is the tag still stable during approach?
- does pose jitter stay reasonable?
- can the robot keep reading the tag down to about **50 cm**?

### Wall-tag navigation
- are wall tags still detected reliably enough for navigation?
- is pose stable enough to be useful?

### Important decision rule
Pick the focus value that gives the best overall vision performance in the range where vision still matters:
- object acquisition at **1 to 2 m**
- guided approach to about **50 cm**

Do **not** optimize for **15 cm** if the final grab is expected to be blind.

---

## Step 6: Set the chosen fixed focus

Once you have chosen the best value, edit `set_fixed_focus.py` and put your selected value into:

```python
LENS_POSITION = 1.33
```

Then run:

```bash
python3 set_fixed_focus.py
```

This sets the camera to:
- manual focus mode
- the chosen fixed `LensPosition`

This is the focus setting you should keep for calibration and runtime.

---

## Step 7: Calibrate the camera

Only calibrate **after** the focus value has been chosen and locked.

During calibration:
- do not enable autofocus
- do not change `LensPosition`
- do not move the camera mount

This matters because changing focus after calibration can shift the optical setup enough to reduce pose accuracy.

---

## Step 8: Use the same focus for runtime

During robot operation:
- use the same fixed `LensPosition`
- do not switch focus values during normal runs
- do not re-enable autofocus

The whole point is to keep the optical setup consistent between:
- focus setup
- calibration
- testing
- competition runs

---

## Step 9: When to redo this procedure

Repeat the procedure if any of these happen:
- camera mount changes
- camera angle changes
- lens or module is disturbed
- you move to a very different intended working distance
- you replace the camera module
- your AprilTag performance clearly changes after hardware handling

You do **not** need to redo the procedure just because you are at a different venue, unless the hardware setup has changed or you discover the previously chosen focus no longer performs well enough.

---

## Recommended practical choice for this robot

Given the current plan:
- detect/lock onto object at about **1 to 2 m**
- slow approach inside about **50 cm**
- lose the tag near about **15 cm**
- finish grab blindly

A sensible single-focus target is usually somewhere around:
- **0.75 m to 1.0 m equivalent**
- often roughly `LensPosition` between **1.0 and 1.5**

That is only a starting estimate. The final choice should come from the tests above.

---

## Suggested test log

It helps to record results like this:

| LensPosition | Object tag at 2 m | Stable at 1 m | Stable at 50 cm | Wall tag OK | Notes |
|---|---|---|---|---|---|
| 1.0 | yes/no | yes/no | yes/no | yes/no | |
| 1.33 | yes/no | yes/no | yes/no | yes/no | |
| 2.0 | yes/no | yes/no | yes/no | yes/no | |

Choose the value with the best overall result, not just the prettiest image.

---

## Typical workflow each time you rebuild software

1. Run `set_fixed_focus.py`
2. Start camera pipeline
3. Run calibration-based AprilTag code
4. Keep focus unchanged

---

## Final rule

**Autofocus is for finding a starting point. Manual focus is for calibration and competition use.**
