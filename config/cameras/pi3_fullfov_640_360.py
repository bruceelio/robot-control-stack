# config/cameras/pi3_fullfov_640_360.py

"""
TUNING ORDER (DO THIS IN FIELD):

1. Adjust LENS_POSITION (sharpest edges)
2. Adjust EXPOSURE_TIME_US (reduce blur)
3. Adjust ANALOGUE_GAIN (brightness vs noise)
4. Adjust MIN_DECISION_MARGIN (stability vs range)
5. Adjust QUAD_DECIMATE (only if needed)

Do NOT change everything at once.
"""


"""
Full-FoV runtime camera profile (IMX708 / Pi Camera Module 3).

This profile prioritises:
- wide field-of-view (more tags visible)
- robust multi-tag localisation
- acceptable performance on Pi 4

Tuning strategy:
- Geometry (FoV) first → already maximised here
- Then optimise: focus → exposure → detector tuning

Use this profile for:
- localisation-heavy tasks
- large arenas
- when seeing ≥2 tags reliably is critical
"""
"""
Base Settings:
EXPOSURE_TIME_US = 4500
ANALOGUE_GAIN = 2
MIN_DECISION_MARGIN = 18
QUAD_DECIMATE = 1.2 (1.5 if want more speed)

Bright Room:
EXPOSURE_TIME_US = 2500
ANALOGUE_GAIN = 1.2
MIN_DECISION_MARGIN = 20.0
QUAD_DECIMATE = 1.5

Night Room
EXPOSURE_TIME_US = 7000
ANALOGUE_GAIN = 2.8
MIN_DECISION_MARGIN = 16.0
QUAD_DECIMATE = 1.0
"""

BACKEND = "pi_libcamera_april"


# --------------------------------------------------
# Capture settings
# --------------------------------------------------

# Output resolution (what your code sees)
# Lower = faster, less detail
# Higher = more detail, slower
# 640x360 is a good compromise for full-FoV mode
WIDTH = 640
HEIGHT = 360

# Frames per second
# Higher = smoother + better motion detection
# Lower = more exposure time available (better in low light)
FPS = 30


# --------------------------------------------------
# Sensor mode (VERY IMPORTANT for FoV)
# --------------------------------------------------

# Forces a wide sensor mode (IMX708 specific)
# 2304x1296 enables full horizontal FoV behaviour
# Lower values → cropped FoV
# Higher values → more detail but slower
SENSOR_OUTPUT_SIZE = (2304, 1296)

# Usually 10-bit for this sensor
# Higher bit depth = better dynamic range, slightly more processing cost
SENSOR_BIT_DEPTH = 10

# Physical sensor size (do not change unless hardware changes)
SENSOR_WIDTH = 4608
SENSOR_HEIGHT = 2592

# Forces use of full sensor area
# True  → maximum FoV (recommended for localisation)
# False → narrower FoV but sometimes cleaner image
FORCE_FULL_SENSOR_SCALER_CROP = True


# --------------------------------------------------
# AprilTag detector tuning
# --------------------------------------------------

FAMILIES = "tag36h11"

# Detection confidence threshold
# Higher (20–30):
#   - fewer false positives
#   - may miss distant or low-contrast tags
# Lower (10–18):
#   - detects weaker/further tags
#   - more noise / flicker
MIN_DECISION_MARGIN = 20

# Image decimation (speed vs detection quality)
# 1.0 → best quality (recommended for full-FoV)
# 1.5 → faster, slightly worse small-tag detection
# 2.0+ → fast but poor for distance
QUAD_DECIMATE = 1.5

# Number of CPU threads
# Increase on Pi 4 for performance
# Too high → diminishing returns
NTHREADS = 2

# Gaussian blur before detection
# 0.0 → sharpest edges (usually best)
# >0   → helps noisy images but reduces detail
QUAD_SIGMA = 0.0

# Edge refinement
# 1 → improves detection accuracy (recommended)
# 0 → faster but less accurate
REFINE_EDGES = 1

# Sharpens decoded image
# 0.25–0.5 typical
# Higher may help low contrast but can introduce artifacts
DECODE_SHARPENING = 0.25

# Debug output (0 = off)
APRILTAG_DEBUG = 0


# --------------------------------------------------
# Calibration
# --------------------------------------------------

# Must match THIS exact resolution + sensor mode
# Incorrect calibration → wrong distances / headings
CALIBRATION_PROFILE = "pi3_fullfov_640_360"


# --------------------------------------------------
# Focus (CRITICAL for detection quality)
# --------------------------------------------------

# manual → stable, best for robotics
# auto   → can drift unpredictably
# continuous → rarely useful here
AF_MODE = None   # "manual"

# Lens position (tune this carefully)
# Typical range: ~0.8 → 1.5
# Lower → focus closer
# Higher → focus further
#
# Symptoms:
#   too low  → distant tags blurry
#   too high → nearby tags blurry
#
# This is often the SINGLE most important setting
LENS_POSITION = 1.2


# --------------------------------------------------
# Exposure / gain (image brightness + motion blur)
# --------------------------------------------------

# Disable auto exposure for consistency
AE_ENABLE = None   # False

# Exposure time (microseconds)
# Lower (2000–4000): bright room: 2500-3500
#   - sharper image (less motion blur)
#   - darker image
# Higher (5000–10000): dark room (5000-8000)
#   - brighter image
#   - more motion blur (bad for moving robot)
EXPOSURE_TIME_US = None

# Analogue gain (brightness amplification)
# Lower (1.0–1.5): bright room (1.0-1.5)
#   - cleaner image
#   - darker
# Higher (2.0–4.0): dark room (2.0-3.5)
#   - brighter
#   - more noise (can hurt detection)
ANALOGUE_GAIN = None


# --------------------------------------------------
# White balance (less critical for AprilTags)
# --------------------------------------------------

# Disable for consistent grayscale conversion
AWB_ENABLE = None           # False

# Colour gains (R, B)
# Only matters slightly for grayscale contrast
# Leave stable once chosen
COLOUR_GAINS = None         # (1.8, 1.5)

# Need to be tested (and not in code yet)
VISION_SETTLE_AFTER_ROTATE_S = 0.35
VISION_SETTLE_AFTER_DRIVE_S = 0.80
VISION_SETTLE_AFTER_ALIGN_S = 0.50
VISION_SETTLE_AFTER_LIFT_S = 0.80
VISION_FRESH_OBS_MAX_AGE_S = 0.12