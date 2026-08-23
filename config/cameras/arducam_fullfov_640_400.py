# config/cameras/arducam_fullfov_640_400.py

"""
Full-FoV runtime camera profile for Arducam OV9281 USB camera.

Physical USB capture:
- 1280x800
- MJPG
- 30 fps

Vision processing:
- resized to 640x400
- AprilTag detection
"""

BACKEND = "opencv_usb_april"


# --------------------------------------------------
# USB capture settings
# --------------------------------------------------

CAPTURE_WIDTH = 1280
CAPTURE_HEIGHT = 800

WIDTH = 640
HEIGHT = 400

FPS = 30
PIXEL_FORMAT = "MJPG"


# --------------------------------------------------
# AprilTag detector tuning
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
# Calibration
# --------------------------------------------------

CALIBRATION_PROFILE = "arducam_fullfov_640_400"