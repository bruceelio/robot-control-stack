localisation/vison/README_VISION.md

# Vision Localisation (PnP Architecture)

This document describes the AprilTag-based vision localisation system used by:

localisation/providers/vision/

The system is designed to support:

- multiple cameras
- multiple localisation approaches
- incremental upgrades
- runtime arbitration
- both simple and advanced vision localisation methods

The current architecture is centered around:

AprilTag observations → generic PnP provider → arbitration

---

# Purpose

The vision localisation system provides:

absolute pose evidence

using:

- arena AprilTags
- calibrated camera geometry
- Perspective-n-Point (PnP) pose estimation

The system is intentionally structured so that:

- camera hardware is isolated from localisation
- multiple cameras may operate simultaneously
- localisation providers consume observations rather than raw hardware
- arbitration decides whether vision should be trusted

---

# High-Level Architecture

Camera Process
    →
AprilTag Detection
    →
vision_message
    →
CameraProcessManager.getlatest(camera_name)
    →
AprilTag Observations Adapter
    →
AprilTag PnP Pose Provider
    →
PoseObservation
    →
Localisation Arbitration

---

# Process Separation

The expensive computer vision operations occur inside the camera worker process.

Camera worker responsibilities:

- image capture
- ISP interaction
- RGB conversion
- AprilTag detection
- marker decoding
- vision_message creation

The main process handles:

- observation adaptation
- pose estimation
- arbitration
- localisation state

This separation ensures:

- camera processing does not block robot control
- multiple cameras may scale independently
- localisation remains hardware-agnostic

---

# Folder Structure

localisation/
    providers/
        vision/
            pose_apriltag_pnp.py

perception/
    perception.py

    vision/
        apriltag_observations.py
        detection_pipeline.py

hw_io/
    cameras/
        vision_worker.py
        camera_process.py

config/
    arena.py

calibration/

---

# Runtime Data Flow

## 1. Camera Worker

The worker process captures images and detects AprilTags.

Output:

vision_message

Containing:

- marker detections
- corner pixels
- timestamps
- camera identity

---

## 2. Observation Adapter

The adapter converts:

vision_message

into:

apriltag_observations

This layer isolates localisation from camera implementation details.

---

## 3. Generic PnP Provider

The provider:

AprilTagPnPPoseProvider

consumes:

- source_id
- apriltag_observations
- intrinsic_matrix
- distortion_coefficients
- camera_to_robot_transform

The provider also reads:

marker_poses(CONFIG.arena_size)

from:

config/arena.py

---

# Arena Geometry

Arena geometry is owned by:

config/arena.py

Arena configuration contains:

- tag x/y/z
- tag yaw/pitch/roll
- tag size
- arena dimensions

The localisation provider does NOT own arena geometry.

The provider only converts geometry into OpenCV-compatible point arrays.

---

# PnP Solve Pipeline

The provider performs:

1. observation filtering
2. object point generation
3. image point generation
4. solvePnP()
5. reprojection scoring
6. validity gating
7. pose estimation output

---

# Current Geometry Model

The current implementation assumes:

- known tag size
- known tag center position
- flat arena coordinate system
- calibrated camera intrinsics
- calibrated camera mount transform

Tag orientation support exists architecturally but is still under refinement.

---

# Reprojection Error

The provider computes:

reprojection_score

by:

- projecting solved 3D points back into image space
- comparing projected pixels against observed corner pixels

Low reprojection error indicates:

good geometric consistency

High reprojection error indicates:

- inconsistent field geometry
- incorrect tag positions
- incorrect orientation assumptions
- poor calibration
- ambiguous solves

---

# Current Validity Rules

Currently:

valid=True

requires:

- solvePnP success
- at least 2 visible arena tags
- reprojection error below threshold

Single-tag solves are currently considered:

local-only evidence

because planar ambiguity may exist.

---

# Single-Tag vs Multi-Tag Behaviour

## Single-Tag PnP

Advantages:

- works with minimal visibility
- often highly precise locally
- low reprojection error

Disadvantages:

- planar ambiguity
- weaker global consistency
- unstable heading under some conditions

---

## Multi-Tag PnP

Advantages:

- stronger global consistency
- improved heading estimation
- reduced ambiguity

Disadvantages:

- sensitive to field geometry accuracy
- requires accurate calibration
- inconsistent arena measurements degrade solves significantly

---

# Arbitration Philosophy

Vision does NOT directly own localisation state.

Instead:

vision providers produce evidence

Arbitration decides:

- whether the pose is trustworthy
- whether another provider should dominate
- whether vision should be ignored entirely

This prevents:

- unstable vision from corrupting localisation
- stale observations from persisting
- single bad frames from resetting pose

---

# Current Development Status

Implemented:

- camera worker separation
- AprilTag detection pipeline
- observation adapter
- generic PnP provider
- reprojection scoring
- validity gating
- localisation reseeding
- multi-camera architecture support

Partially implemented:

- camera-to-robot transform usage
- orientation-aware tag geometry
- multi-camera arbitration

Planned:

- ambiguity scoring
- solvePnPGeneric support
- confidence weighting
- temporal filtering
- spatial median filtering
- fused vision localisation
- IMU-assisted arbitration

---

# Key Architectural Principles

## 1. Providers Consume Observations

Providers should use:

observations

NOT direct camera hardware.

---

## 2. Config Owns Geometry

Arena geometry belongs to:

config/arena.py

NOT localisation providers.

---

## 3. Arbitration Owns Trust

Providers report evidence.

Arbitration decides trust.

---

## 4. Camera Workers Own Detection

Workers perform:

- capture
- detection
- decoding

Main process performs:

- localisation
- arbitration
- strategy

---

# Practical Reality

Multi-tag PnP is extremely sensitive to:

- marker placement accuracy
- tag orientation accuracy
- camera calibration accuracy

Small physical measurement errors may create large reprojection errors.

Single-tag solves are often much more tolerant.

This is expected behaviour in real robotics systems.

---

# Design Goal

The goal is NOT merely:

"make vision work"

The goal is to create:

a scalable, debuggable, multi-provider localisation architecture

capable of supporting:

- simple robots
- advanced robots
- experimental localisation methods
- multiple cameras
- multiple sensor fusion approaches

# References

https://docs.photonvision.org/en/v2025.3.1/docs/apriltag-pipelines/multitag.html
https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
https://github.com/FRC701/SimpleLimelight
