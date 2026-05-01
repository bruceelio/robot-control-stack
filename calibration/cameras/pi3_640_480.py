# calibration/cameras/pi3_640_480.py

"""
Approximate / unofficial intrinsics for a RaspberryPi camera setup.

These values are placeholders to enable pose estimation during early
development. They are NOT the result of a proper calibration procedure.

Replace with real calibrated values when available.
"""

# Format:
# CAMERA_PARAMS = (fx, fy, cx, cy)
# default values (950, 950, 320, 240)   (typical webcam 600 but Pi3 around 950)

CAMERA_PARAMS = (950.0, 950.0, 320.0, 240.0)    #default values

DIST_COEFFS = None

NOTES = (
    "Unofficial placeholder values for 640x480 mode. "
    "Replace with real calibration when available."
)
