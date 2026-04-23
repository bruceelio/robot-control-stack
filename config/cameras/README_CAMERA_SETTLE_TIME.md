config/cameras/README_CAMERA_SETTLE_TIME.md

````markdown
# Camera Settle Time Tuning Guide (Pi Camera 3 + AprilTags)

This guide explains how to reduce **settle time** — the delay between robot motion and stable, usable vision.

---

## 🧠 What “settle time” really means

There are two different settle times:

### 1. Camera / image settle time
How long until the image stabilises after:
- robot motion
- lighting change
- exposure/focus adjustment

### 2. Detection / localisation settle time
How long until:
- AprilTag detections stop flickering
- pose estimate becomes stable

---

## 🎯 Settle Time Tuning Table

| Setting | Effect on settle time | What to do |
|---|---|---|
| `EXPOSURE_TIME_US` | **BIGGEST factor for motion settle** | Lower it (e.g. 5000 → 3000) to reduce motion blur |
| `ANALOGUE_GAIN` | Compensates for lower exposure | Increase slightly if image becomes too dark |
| `FPS` | Faster frame updates | Keep at 30 (or higher if possible) |
| `AE_ENABLE` | Causes brightness to drift over frames | Keep **False** for faster settling |
| `AF_MODE` | Autofocus causes long settling | Use **manual focus** |
| `LENS_POSITION` | Wrong focus causes unstable detection | Tune carefully once, then leave fixed |
| `QUAD_DECIMATE` | Affects detection stability | Lower (1.0) = more stable but slower |
| `MIN_DECISION_MARGIN` | Filters weak detections | Slightly higher = less flicker |
| `WIDTH`, `HEIGHT` | More pixels = more stable detection | Increase slightly if CPU allows |
| `FORCE_FULL_SENSOR_SCALER_CROP` | Wider FoV → more tags → faster localisation convergence | Keep True for multi-tag stability |

---

## 🔥 The Big 3 (Most Impact)

If you only change three things:

### 1. Reduce motion blur
```python
EXPOSURE_TIME_US = 3000
````

### 2. Restore brightness

```python
ANALOGUE_GAIN = 2.5
```

### 3. Disable auto-adjustments

```python
AE_ENABLE = False
AF_MODE = "manual"
```

---

## ⚠️ Common Symptom Explained

If you see:

> detection works while moving, then disappears when stationary

That usually means:

* motion temporarily increases edge contrast
* stationary image is slightly soft or low contrast

Root causes:

* focus not optimal
* exposure too high (blur)
* insufficient contrast

---

## 🧪 Two Practical Modes

### 🟢 Fast Settle Mode (for moving robot)

```python
EXPOSURE_TIME_US = 2500–3500
ANALOGUE_GAIN = 2.0–3.0
QUAD_DECIMATE = 1.5
MIN_DECISION_MARGIN = 18–22
```

**Result:**

* faster stabilisation
* less motion blur
* slightly reduced detection range

---

### 🔵 Stable Precision Mode (for localisation)

```python
EXPOSURE_TIME_US = 4000–6000
ANALOGUE_GAIN = 1.5–2.0
QUAD_DECIMATE = 1.0
MIN_DECISION_MARGIN = 16–18
```

**Result:**

* slower settle
* more stable pose
* better long-range detection

---

## ⚡ Hidden Factor: Multi-Tag Visibility

More visible tags = faster convergence.

```
FoV ↑ → tags ↑ → settle time ↓
```

Why:

* multiple heading estimates
* averaging reduces noise
* less dependence on a single tag

---

## 🧭 Practical Tuning Workflow

Follow this order:

### Step 1 — Reduce motion blur

Lower:

```python
EXPOSURE_TIME_US
```

---

### Step 2 — Fix brightness

Adjust:

```python
ANALOGUE_GAIN
```

---

### Step 3 — Lock camera behaviour

```python
AE_ENABLE = False
AF_MODE = "manual"
```

---

### Step 4 — Stabilise detections

```python
MIN_DECISION_MARGIN ↑ slightly
```

---

## 💡 Bonus Trick (Highly Effective)

After stopping the robot, wait briefly before using vision:

```text
Delay: 100–300 ms
```

This allows:

* vibration to settle
* rolling shutter effects to stabilise
* exposure pipeline to settle

---

## 🔚 Key Principle

```
Less blur + no auto-adjustment + more visible tags = faster convergence
```

---

## 🧠 Final Insight

You are no longer limited by:

* configuration
* architecture

You are now tuning:

* optics (focus)
* physics (light + motion)

---

## 🚀 Optional Advanced Strategy

Use two profiles:

* **Scan Mode**

  * fast detection
  * wider tolerance
* **Commit Mode**

  * high precision
  * stable pose

Switch between them during operation.

---

End of guide.

```
```
