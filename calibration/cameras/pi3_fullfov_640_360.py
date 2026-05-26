# calibration/cameras/pi3_fullfov_640_360.py

# can track april tags at 15 sec/rotation (0.11 speed)

from .pi3_640_480 import *  # noqa

# --------------------------------------------------
# Legacy 2D / bearing-distance calibration
# --------------------------------------------------
# Used by cam1_markers2 / triangulation-style localisation.
CAMERA_PARAMS = (477.0, 477.0, 320.0, 240.0)
DISTORTION_COEFFICIENTS = (0.0, 0.0, 0.0, 0.0, 0.0)

# --------------------------------------------------
# PnP calibration
# --------------------------------------------------
# Used only by AprilTagPnPPoseProvider.
# Temporary runtime calibration for 640x360 stream.
# PNP_CAMERA_PARAMS = (390.0, 390.0, 320.0, 180.0)
# PNP_CAMERA_PARAMS = (493.0, 370.0, 320.0, 180.0) (this isn't the right path)
PNP_CAMERA_PARAMS = (493.0, 493.0, 320.0, 180.0)
PNP_DISTORTION_COEFFICIENTS = (0.0, 0.0, 0.0, 0.0, 0.0)


NOTES = (
    "Temporary calibration derived from pi3_640_480. "
    "CAMERA_PARAMS preserves legacy 2D localisation behaviour. "
    "PNP_CAMERA_PARAMS is used only for AprilTag PnP. "
    "Full calibration required for accuracy."
)