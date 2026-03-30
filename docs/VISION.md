# Vision System Design – Student Robotics Robot

## Overview

This document describes the vision system architecture for the robot, focused on:

* **AprilTag-based object detection and pose estimation**
* **Reliable close-range tracking for object pickup**
* **Fallback detection using a secondary camera**
* Integration with **OTOS** for continuity between detections

---

## Hardware Configuration

### Primary Camera

* **Raspberry Pi Camera Module 3 (standard FoV)**
* Interface: CSI (ribbon cable)
* Role:

  * Primary AprilTag detection
  * Close-range tracking during pickup
  * Pose estimation

### Secondary Camera

* **Logitech C270 Webcam**
* Interface: USB (via hub)
* Role:

  * Fallback detection when primary camera loses target
  * Wider-area search / reacquisition

### Compute

* **Raspberry Pi 4 Model B (2GB)**
* Running:

  * Vision processing
  * Sensor fusion (vision + OTOS)
  * Robot control logic

---

## Software Stack

### Core Components

* **libcamera / Picamera2**

  * Camera interface for Pi Camera

* **AprilTag (pupil-apriltags)**

  * Tag detection and pose estimation

* **OpenCV (optional)**

  * Image conversion (RGB → grayscale)
  * Debug visualization


## Mechanical Mounting

### Primary Camera Placement

* Mounted **above gripper/intake**
* Positioned near **centerline**
* Tilted downward **20–30°**

#### Goals:

* Maintain tag visibility during approach
* Prevent occlusion by manipulator
* Keep tag in frame at ~8–10 cm

---

### Secondary Camera Placement

* Mounted **higher or wider**
* Forward-facing or wide-angle
* Not constrained by manipulator geometry

#### Goals:

* Broad visibility
* Tag reacquisition
* Backup detection

---

## Vision Strategy

### Detection Pipeline

```text
Primary Camera → AprilTag Detection → Pose
        ↓
If no detection:
        ↓
Secondary Camera → AprilTag Detection
```

---

### Behaviour Logic

1. Use **primary camera** for all tracking and pickup
2. If no tag detected:

   * Query **secondary camera**
3. If both fail:

   * Use **OTOS** for rough localization
4. During pickup:

   * Use **last known tag pose**
   * Complete motion with controlled approach

---

## Expected Performance

| Metric              | Pi Camera 3   | C270          |
| ------------------- | ------------- | ------------- |
| Close tracking loss | ~5–10 cm      | ~10–18 cm     |
| Max approach speed  | 0.30–0.50 m/s | 0.15–0.25 m/s |
| Role                | Primary       | Secondary     |

---

## Testing Procedures

### 1. Static Detection Test

**Goal:** Verify basic detection

* Place 80 mm AprilTag at:

  * 50 cm
  * 30 cm
  * 15 cm
* Confirm:

  * Detection stability
  * Pose output consistency

---

### 2. Motion Blur Test

**Goal:** Tune exposure vs motion

* Drive robot toward tag at increasing speeds:

  * 0.1 m/s → 0.5 m/s
* Record:

  * Detection success rate
  * Frame dropouts

**Adjust:**

* Reduce exposure (shorter shutter)
* Increase lighting if needed

---

### 3. Close-Range Dropout Test

**Goal:** Determine minimum usable distance

* Slowly approach tag
* Record distance where detection fails

**Target:**

* Maintain detection to ~8–10 cm (primary camera)

---

### 4. Occlusion Test

**Goal:** Ensure gripper does not block view

* Simulate pickup motion
* Check:

  * Tag visibility during final approach
  * Frame coverage

---

### 5. Secondary Camera Fallback Test

**Goal:** Validate failover

* Block primary camera
* Confirm:

  * Secondary detects tag
  * System switches correctly

---

### 6. Lighting Robustness Test

**Goal:** Ensure consistency under varying lighting

* Test under:

  * Bright light
  * Dim light
  * Uneven lighting

**Adjust:**

* Gain
* Add LED illumination if needed

---

### 7. End-to-End Pickup Test

**Goal:** Validate full system

* Detect object
* Approach using vision
* Maintain lock or fallback
* Complete pickup using:

  * vision OR
  * last known pose + OTOS

---

## Key Design Principles

* **Short exposure > bright image**
* **Lighting > increasing exposure**
* **Primary camera = precision**
* **Secondary camera = recovery**
* **Vision + OTOS = robust system**

---

## Future Improvements

* Add LED illumination for consistent exposure
* Tune AprilTag detection parameters (decimation, blur)
* Consider threading camera + detection pipelines
* Evaluate hybrid odometry (OTOS + dead wheel)

---

## Summary

This system provides:

* Reliable **close-range AprilTag tracking**
* Robust **fallback detection**
* Efficient use of **Pi 4B resources**
* Strong integration with **sensor fusion**

---
