# config/cameras/pi3_fullfov_640_360.py

"""
Pi Camera Module 3 - full FoV, 640x360 processing profile.

This file contains three types of settings:

1. PROPAGATED SETTINGS
   These are consumed by the camera/vision software.

2. OPTIONAL CAMERA OVERRIDES
   These are passed to the Pi camera backend, but None means:
   do not override the camera/libcamera behaviour.

3. CROSS-CHECK / TUNING REFERENCE
   Documentation only. These values are used to compare against
   actual camera metadata and guide field tuning.
"""


BACKEND = "pi_libcamera_april"


# ==================================================
# 1. PROPAGATED SETTINGS
# ==================================================

# --------------------------------------------------
# Capture / processing resolution
# --------------------------------------------------

WIDTH = 640
HEIGHT = 360
FPS = 30


# --------------------------------------------------
# Sensor / full-FoV configuration
# --------------------------------------------------

SENSOR_OUTPUT_SIZE = (2304, 1296)
SENSOR_BIT_DEPTH = 10

SENSOR_WIDTH = 4608
SENSOR_HEIGHT = 2592

FORCE_FULL_SENSOR_SCALER_CROP = True


# --------------------------------------------------
# AprilTag detector
# --------------------------------------------------

FAMILIES = "tag36h11"

MIN_DECISION_MARGIN = 20

QUAD_DECIMATE = 1.5
NTHREADS = 2
QUAD_SIGMA = 0.0
REFINE_EDGES = 1
DECODE_SHARPENING = 0.25
APRILTAG_DEBUG = 0


# --------------------------------------------------
# Calibration selection
# --------------------------------------------------

CALIBRATION_PROFILE = "pi3_fullfov_640_360"


# ==================================================
# 2. OPTIONAL CAMERA OVERRIDES
# ==================================================
#
# These values are passed to the Pi camera backend.
#
# None means:
#     do not force this control;
#     allow libcamera / camera defaults to manage it.
#
# A real value means:
#     explicitly apply that value to the camera.
# ==================================================


# --------------------------------------------------
# Focus
# --------------------------------------------------

AF_MODE = None

# Currently explicitly applied because it is not None.
LENS_POSITION = 1.2


# --------------------------------------------------
# Exposure / gain
# --------------------------------------------------

AE_ENABLE = None
EXPOSURE_TIME_US = None
ANALOGUE_GAIN = None


# --------------------------------------------------
# White balance
# --------------------------------------------------

AWB_ENABLE = None
COLOUR_GAINS = None


# ==================================================
# 3. CROSS-CHECK / TUNING REFERENCE
# ==================================================
#
# THESE VALUES ARE DOCUMENTATION ONLY.
#
# They are NOT propagated into the camera software.
#
# Use them when examining actual libcamera metadata and when
# deciding whether an OPTIONAL CAMERA OVERRIDE should be set.
# ==================================================


# --------------------------------------------------
# Suggested starting points
# --------------------------------------------------

# General / base:
#
# Exposure time:       4500 us
# Analogue gain:       2.0
# Decision margin:     18
# Quad decimate:       1.2
#
# Use QUAD_DECIMATE = 1.5 when processing speed is preferred.


# Bright room:
#
# Exposure time:       2500 us
# Analogue gain:       1.2
# Decision margin:     20
# Quad decimate:       1.5


# Dark room:
#
# Exposure time:       7000 us
# Analogue gain:       2.8
# Decision margin:     16
# Quad decimate:       1.0


# --------------------------------------------------
# Field tuning order
# --------------------------------------------------
#
# 1. Check / adjust lens position for sharp tag edges
# 2. Check actual exposure time
# 3. Check actual analogue gain
# 4. Adjust MIN_DECISION_MARGIN if required
# 5. Adjust QUAD_DECIMATE only if required
#
# Do not change several variables simultaneously.


# --------------------------------------------------
# General tuning notes
# --------------------------------------------------
#
# Focus:
#   Lens position roughly 0.8 - 1.5 is a useful tuning region.
#
# Exposure:
#   Lower exposure reduces dead_reckoning blur but produces a darker image.
#   Higher exposure increases brightness but increases dead_reckoning blur.
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


# ==================================================
# 4. FUTURE / NOT CURRENTLY CONSUMED
# ==================================================
#
# Keep proposed settings here until code actually consumes them.
# This prevents them from looking like active configuration.
# ==================================================

# VISION_SETTLE_AFTER_ROTATE_S = 0.35
# VISION_SETTLE_AFTER_DRIVE_S = 0.80
# VISION_SETTLE_AFTER_ALIGN_S = 0.50
# VISION_SETTLE_AFTER_LIFT_S = 0.80
# VISION_FRESH_OBS_MAX_AGE_S = 0.12