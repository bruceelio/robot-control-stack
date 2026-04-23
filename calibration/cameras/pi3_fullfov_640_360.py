# calibration/cameras/pi3_fullfov_640_360.py

from .pi3_640_480 import *  # noqa

# Override only what must change immediately

# Adjust principal point for 640x360
CAMERA_PARAMS = (460.0, 460.0, 320.0, 240.0)    #default values

NOTES = (
    "Temporary calibration derived from pi3_640_480. "
    "Only cy adjusted for 640x360. "
    "Full calibration required for accuracy."
)