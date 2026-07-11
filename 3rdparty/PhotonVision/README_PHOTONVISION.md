# Raspberry Pi 4B Dual Camera Vision System Notes

## Overview

This project explores a Raspberry Pi 4B based robotics vision system using:

* Raspberry Pi 4B
* Pi Camera 3
* PhotonVision
* AprilTags
* Python/OpenCV experimentation

The primary design goal is:

* reliable target selection
* stable tag detection
* acceptable latency
* moderate range (~3 meters)
* full camera FOV preservation where possible

rather than full-field robot localization.

---

# Key Design Conclusions

## 1. Pi Camera 3 + PhotonVision FOV Issues

The Pi Camera 3 uses a native sensor resolution of:

4608 x 2592 (16:9)

PhotonVision/libcamera commonly exposes standard resolutions such as:

* 640x480
* 1280x720
* 1920x1080

However:

* 640x480 visibly crops the image
* even 1280x720 may still use a cropped sensor mode
* PhotonVision does not appear to expose full-FOV scaling controls

This differs from direct Python/libcamera/Picamera2 usage where:

* 640x360
* full_FOV scaling
* uncropped 16:9 operation

could be explicitly configured.

Conclusion:

Pi Camera 3 works well under direct Python/libcamera control, but PhotonVision currently limits access to the full-FOV behavior.

---

# Camera Architecture Decision

## Final Preferred Architecture

### Preferred

Single Raspberry Pi 4B running:

* main robot support code
* camera 1
* camera 2
* direct Python/OpenCV/libcamera control

### Rejected

Pi 3B running PhotonVision for dual cameras.

Reason:

* lower CPU capability
* additional networking complexity
* less camera control
* PhotonVision cropping limitations
* additional failure points

---

# PhotonVision Tuning Notes

## Processing Mode

Using:

* 2D mode

rather than:

* 3D pose estimation

Reason:

Primary goal is target selection and targeting assistance rather than full localization.

---

# AprilTag Pipeline Settings

## Final Working Settings

| Setting    | Value   |
| ---------- | ------- |
| Decimate   | 2       |
| Blur       | 0       |
| Brightness | ~60     |
| Exposure   | ~20,000 |
| Gain       | 22      |
| FPS        | ~50     |

---

# Decimate Observations

## Decimate = 1

Results:

* ~29 FPS
* highest CPU load
* maximum detection quality

## Decimate = 2

Results:

* ~50 FPS
* substantial performance increase
* acceptable tag detection out to ~3 meters

Conclusion:

Decimate = 2 is the practical operating point.

The bottleneck was primarily AprilTag processing load, not camera exposure settings.

---

# Exposure / Brightness Findings

Important discovery:

Increasing brightness reduced required exposure until approximately:

Brightness 55–60

Beyond this point:

* exposure improvements plateaued
* clipping risk increased
* no meaningful FPS improvement occurred

Practical conclusion:

Brightness around 60 appears near the optimal operating region.

---

# Gain Findings

Reducing gain below 22 caused loss of close-range tag detection.

Conclusion:

Gain 22 is required for stable close-tag operation under current lighting conditions.

---

# FPS Findings

Attempting 40 FPS operation with lower brightness/exposure resulted in:

* loss of close tags
* loss of far tags
* only mid-range tags remained stable

Conclusion:

Stable detection quality matters more than raw FPS.

Decimate 2 provided the real FPS improvement.

---

# Close Tag Detection Findings

The far tag (~3 meters) was consistently reliable.

The close tag failed first.

Possible causes:

* close-range saturation/clipping
* tag occupying too much of frame
* rolling shutter distortion
* autofocus/focus behavior
* partial frame clipping

The issue does NOT appear primarily related to decimate.

---

# PhotonVision Driver Mode

Driver Mode disables vision processing and optimizes the stream for human viewing.

Not useful during AprilTag tuning.

All testing was performed with Driver Mode OFF.

---

# White Balance / Low Latency

Recommended configuration:

| Setting            | Recommendation   |
| ------------------ | ---------------- |
| Auto White Balance | OFF after tuning |
| White Balance Temp | Fixed/manual     |
| Low Latency Mode   | ON               |

Reason:

Stable image characteristics are more important than visually pleasing output.

---

# Calibration Notes

PhotonVision calibration:

* calibrates camera intrinsics
* does NOT use AprilTag decimate settings

Critical calibration variables:

* resolution
* crop/FOV mode
* focus state
* camera pipeline mode

Most important lesson:

Pi Camera 3 cropping/FOV behavior affects calibration more than decimate settings.

---

# Major System Conclusion

The most important discovery from this work:

## Pi Camera 3 + PhotonVision

is limited by:

* automatic sensor mode selection
* cropping behavior
* lack of exposed full-FOV scaling controls

while:

## Pi Camera 3 + Python/libcamera

allows:

* native 16:9 operation
* explicit 640x360 full-FOV scaling
* greater control over sensor behavior

Therefore:

Direct Python/OpenCV/libcamera control on the Pi 4B is currently preferred over PhotonVision for this camera setup.

---

# Future Considerations

Possible future upgrades:

* OV9281 global shutter camera
* Orange Pi 5
* Pi 5
* dedicated vision coprocessor

Reasons:

* more predictable camera geometry
* global shutter
* lower latency
* better high-speed motion handling
* improved PhotonVision compatibility

---

# Final Summary

Current best-known stable operating configuration:

| Parameter       | Value                        |
| --------------- | ---------------------------- |
| Platform        | Raspberry Pi 4B              |
| Cameras         | Dual cameras                 |
| Vision Mode     | 2D                           |
| Decimate        | 2                            |
| Blur            | 0                            |
| Brightness      | ~60                          |
| Exposure        | ~20,000                      |
| Gain            | 22                           |
| FPS             | ~50                          |
| Target Range    | ~3 meters                    |
| Primary Use     | target selection / targeting |
| Preferred Stack | Python/OpenCV/libcamera      |
