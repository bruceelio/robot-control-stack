# config/cameras/pi3.py

"""
Runtime camera configuration for a RaspberryPi camera setup.

This file answers:
- which backend to use
- runtime/tuning settings
- which calibration profile to load

It does NOT contain measured intrinsics itself.
Those live in calibration/cameras/.
It also does NOT define tag size; that belongs to arena/game config.
"""

BACKEND = "pi_libcamera_april"

# Capture settings
WIDTH = 640
HEIGHT = 480
FPS = 30

# AprilTag detector settings
FAMILIES = "tag36h11"
MIN_DECISION_MARGIN = 20.0

# Which measured/assumed calibration to use
CALIBRATION_PROFILE = "pi3_640_480"