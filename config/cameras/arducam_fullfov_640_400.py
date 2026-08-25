# config/cameras/arducam_fullfov_640_400.py

"""
Arducam OV9281 USB camera - full FoV, 640x400 processing profile.

This file contains three types of settings:

1. PROPAGATED SETTINGS
   These are consumed by the camera/vision software.

2. OPTIONAL CAMERA OVERRIDES
   Reserved for camera controls that may later be propagated
   into the OpenCV/V4L2 backend.

3. CROSS-CHECK / TUNING REFERENCE
   Documentation only. Compare these against actual V4L2
   camera values and use them to guide tuning.
"""


BACKEND = "opencv_usb"


# ==================================================
# 1. PROPAGATED SETTINGS
# ==================================================

# --------------------------------------------------
# USB capture
# --------------------------------------------------

CAPTURE_WIDTH = 1280
CAPTURE_HEIGHT = 800

FPS = 30
PIXEL_FORMAT = "MJPG"


# --------------------------------------------------
# Vision processing resolution
# --------------------------------------------------

WIDTH = 640
HEIGHT = 400


# --------------------------------------------------
# AprilTag detector
# --------------------------------------------------

FAMILIES = "tag36h11"

MIN_DECISION_MARGIN = 15

QUAD_DECIMATE = 1.5
NTHREADS = 2
QUAD_SIGMA = 0.0
REFINE_EDGES = 1
DECODE_SHARPENING = 0.25
APRILTAG_DEBUG = 0


# --------------------------------------------------
# Calibration selection
# --------------------------------------------------

CALIBRATION_PROFILE = "arducam_fullfov_640_400"


# ==================================================
# 2. OPTIONAL CAMERA OVERRIDES
# ==================================================
#
# None = leave current camera/driver setting unchanged.
# A value = explicitly apply that V4L2 control.
# ==================================================

AUTO_EXPOSURE = 1                # 1=Manual, 3=Aperture Priority
EXPOSURE_TIME_ABSOLUTE = 150      # range: 1 to 5000; 45 = 4.5 ms
GAIN = 0                         # range: 0 to 100

POWER_LINE_FREQUENCY = None      # 0=Disabled, 1=50 Hz, 2=60 Hz

BRIGHTNESS = None                # range: -64 to 64
CONTRAST = None                  # range: 0 to 64
GAMMA = None                     # range: 72 to 500
SHARPNESS = None                 # range: 0 to 6
BACKLIGHT_COMPENSATION = None    # range: 0 to 2


# ==================================================
# 3. CROSS-CHECK / TUNING REFERENCE
# ==================================================
#
# THESE VALUES ARE DOCUMENTATION ONLY.
#
# They are NOT currently propagated into OpenCVUSBCamera.
#
# Compare these with:
#
#     v4l2-ctl --list-ctrls-menus
#
# or individual --get-ctrl queries.
# ==================================================


# --------------------------------------------------
# Current observed camera values
# --------------------------------------------------
#
# From OV9281 V4L2 controls:
#
# brightness:                  0    # range: -64 to 64
# contrast:                   32    # range: 0 to 64
# saturation:                 64    # range: 0 to 128
# hue:                         0    # range: -40 to 40
# white_balance_automatic:     1    # range: 0 to 1
# gamma:                     100    # range: 72 to 500
# gain:                        0    # range: 0 to 100
# power_line_frequency:        2    # range: 0 to 2
#                                  # 0=Disabled, 1=50 Hz, 2=60 Hz
# sharpness:                   3    # range: 0 to 6
# backlight_compensation:      1    # range: 0 to 2
#
# auto_exposure:               3    # values: 1=Manual, 3=Aperture Priority
# exposure_time_absolute:    157    # range: 1 to 5000
#                                  # inactive while auto exposure is enabled
# exposure_dynamic_framerate:  0    # range: 0 to 1


# --------------------------------------------------
# UK mains-frequency cross-check
# --------------------------------------------------
#
# UK mains is 50 Hz.
#
# The camera was observed with:
#
# power_line_frequency = 2   # 60 Hz
#
# Candidate setting to test later:
#
# power_line_frequency = 1   # 50 Hz
#
# Do not force this until we deliberately test its effect.


# --------------------------------------------------
# Field tuning order
# --------------------------------------------------
#
# 1. Confirm actual capture mode is 1280x800 MJPG @ 30 fps
# 2. Check image sharpness / focus
# 3. Check actual exposure behaviour
# 4. Check gain
# 5. Compare detection at 640x400 vs 1280x800
# 6. Adjust MIN_DECISION_MARGIN if required
# 7. Adjust QUAD_DECIMATE only if required
#
# Do not change several variables simultaneously.


# --------------------------------------------------
# General tuning notes
# --------------------------------------------------
#
# Exposure:
#   Lower exposure reduces motion blur but darkens the image.
#   Higher exposure brightens the image but increases motion blur.
#
# Gain:
#   Higher gain brightens the image but increases noise.
#
# Decision margin:
#   Higher = stronger detections required.
#   Lower  = weaker / more distant detections accepted.
#
# Quad decimate:
#   1.0 = maximum detector image detail
#   1.5 = useful speed/detail compromise
#   2.0+ = faster but increasingly poor for small/distant tags