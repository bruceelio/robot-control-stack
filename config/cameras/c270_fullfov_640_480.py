# config/cameras/c270_fullfov_640_480.py

"""
Logitech C270 USB camera - full FoV, 640x480 processing profile.

This file contains three types of settings:

1. PROPAGATED SETTINGS
   These are consumed by the camera/vision software.

2. OPTIONAL CAMERA OVERRIDES
   Reserved for camera controls applied through the
   OpenCV/V4L2 backend.

3. CROSS-CHECK / TUNING REFERENCE
   Documentation only. Compare these against the actual
   C270 V4L2 values and use them to guide tuning.

Initial exposure/gain settings are provisional.
They should be tuned on the physical camera.
"""


BACKEND = "opencv_usb"


# ==================================================
# 1. PROPAGATED SETTINGS
# ==================================================

# --------------------------------------------------
# USB capture
# --------------------------------------------------

# Start at the same resolution as the available
# C270-specific calibration.
#
# C270 / USB ID 046d:0825 is reported to support
# 640x480 MJPG at 30 fps.

CAPTURE_WIDTH = 640
CAPTURE_HEIGHT = 480

FPS = 30
PIXEL_FORMAT = "MJPG"


# --------------------------------------------------
# Vision processing resolution
# --------------------------------------------------

WIDTH = 640
HEIGHT = 480


# --------------------------------------------------
# AprilTag detector
# --------------------------------------------------

FAMILIES = "tag36h11"

MIN_DECISION_MARGIN = 15

# Start conservatively.
# The C270 has a substantially narrower FoV than the
# Arducam wide-angle camera, so tags should occupy
# more pixels at a given distance.
QUAD_DECIMATE = 1.5

NTHREADS = 2
QUAD_SIGMA = 0.0
REFINE_EDGES = 1
DECODE_SHARPENING = 0.25
APRILTAG_DEBUG = 0


# --------------------------------------------------
# Calibration selection
# --------------------------------------------------

CALIBRATION_PROFILE = "c270_fullfov_640_480"


# ==================================================
# 2. OPTIONAL CAMERA OVERRIDES
# ==================================================
#
# None = leave current camera/driver setting unchanged.
# A value = explicitly apply that V4L2 control.
#
# Initial values below are starting points only.
# Confirm actual supported controls using:
#
#     v4l2-ctl --list-ctrls-menus
#
# ==================================================


# C270 uses the normal UVC/V4L2 exposure convention:
#
#     1 = Manual
#     3 = Aperture Priority / automatic exposure
#
AUTO_EXPOSURE = 1


# V4L2 exposure_absolute units are 100 us.
#
# 50 = 5.0 ms
# 75 = 7.5 ms
# 100 = 10.0 ms
#
# Start at 5 ms because AprilTag detection benefits
# from limiting motion blur.
EXPOSURE_TIME_ABSOLUTE = 50


# Start with minimum gain.
# Increase gain before substantially increasing exposure
# if the image is too dark during robot motion.
GAIN = 0


# UK mains is 50 Hz:
#
#     0 = Disabled
#     1 = 50 Hz
#     2 = 60 Hz
#
# Leave unchanged initially and deliberately test later.
POWER_LINE_FREQUENCY = None


BRIGHTNESS = None
CONTRAST = None
GAMMA = None
SHARPNESS = None
BACKLIGHT_COMPENSATION = None


# ==================================================
# 3. CROSS-CHECK / TUNING REFERENCE
# ==================================================
#
# THESE VALUES ARE DOCUMENTATION ONLY.
#
# Actual values may vary between C270 hardware
# revisions, firmware and Linux/UVC driver versions.
#
# Compare against:
#
#     v4l2-ctl --list-ctrls-menus
#
# ==================================================


# --------------------------------------------------
# Reported C270 V4L2 control ranges
# --------------------------------------------------
#
# A Linux C270 / 046d:0825 example reports:
#
# brightness:
#     range:   0 to 255
#     default: 128
#
# contrast:
#     range:   0 to 255
#     default: 32
#
# saturation:
#     range:   0 to 255
#     default: 32
#
# gain:
#     range:   0 to 255
#     default: 0
#
# sharpness:
#     range:   0 to 255
#     default: 24
#
# backlight_compensation:
#     range:   0 to 1
#     default: 1
#
# power_line_frequency:
#     0 = Disabled
#     1 = 50 Hz
#     2 = 60 Hz
#
# auto_exposure:
#     1 = Manual
#     3 = Aperture Priority / automatic
#
# exposure_time_absolute:
#     reported range: 1 to 10000
#     default:        166
#
# 166 corresponds nominally to 16.6 ms.
# That is much longer than desirable for a moving robot.


# --------------------------------------------------
# Initial robotics exposure tests
# --------------------------------------------------
#
# Suggested first tests:
#
#     exposure = 50   gain = 0
#     exposure = 75   gain = 0
#     exposure = 50   gain = 20
#     exposure = 75   gain = 20
#
# Preference:
#
#     use the shortest exposure which still gives
#     reliable AprilTag detection.
#
# A visually dark image is acceptable if AprilTag
# detection remains reliable.


# --------------------------------------------------
# Field tuning order
# --------------------------------------------------
#
# 1. Confirm 640x480 MJPG @ 30 fps
# 2. Confirm the physical C270 V4L2 controls
# 3. Check image sharpness / fixed focus
# 4. Test exposure
# 5. Test gain
# 6. Test 50 Hz anti-flicker
# 7. Check AprilTag range and decision margin
# 8. Adjust QUAD_DECIMATE only if required
# 9. Perform official camera calibration
#
# Do not change several variables simultaneously.