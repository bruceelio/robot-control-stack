# config/cameras/pi3_legacy_640_480.py

"""
Legacy / baseline camera configuration for Raspberry Pi Camera Module 3.

This represents the original working setup before full-FoV experiments.

Use this as:
- a known-good fallback
- calibration baseline
- comparison reference for new modes
"""

BACKEND = "pi_libcamera_april"

# Capture settings (original stable mode)
WIDTH = 640
HEIGHT = 480
FPS = 30

# Sensor mode (let libcamera choose default for this resolution)
SENSOR_OUTPUT_SIZE = None
SENSOR_BIT_DEPTH = None

# Physical sensor size (IMX708)
SENSOR_WIDTH = 4608
SENSOR_HEIGHT = 2592

# Runtime camera behaviour
FORCE_FULL_SENSOR_SCALER_CROP = False

# AprilTag detector settings
FAMILIES = "tag36h11"
MIN_DECISION_MARGIN = 20.0
QUAD_DECIMATE = 1.5
NTHREADS = 2
QUAD_SIGMA = 0.0
REFINE_EDGES = 1
DECODE_SHARPENING = 0.25
APRILTAG_DEBUG = 0

# Calibration profile (matches this exact mode)
CALIBRATION_PROFILE = "pi3_640_480"

# -----------------------------
# Camera control (from setup)
# -----------------------------

# Focus
AF_MODE = None    # "None" "manual"
LENS_POSITION = 1.2    # 1.2; replace with your measured best value

# Exposure / gain
AE_ENABLE = None       # False
EXPOSURE_TIME_US = None   # 3075; replace with your measured value
ANALOGUE_GAIN = None       # 1.1228;  replace with your measured value

# White balance
AWB_ENABLE = None  # False
COLOUR_GAINS = None   # (2.0241, 1.9154); replace with your measured values

# Need to be tested (and not in code yet)
VISION_SETTLE_AFTER_ROTATE_S = 0.35
VISION_SETTLE_AFTER_DRIVE_S = 0.80
VISION_SETTLE_AFTER_ALIGN_S = 0.50
VISION_SETTLE_AFTER_LIFT_S = 0.80
VISION_FRESH_OBS_MAX_AGE_S = 0.12