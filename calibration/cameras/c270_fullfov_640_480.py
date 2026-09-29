# calibration/cameras/c270_fullfov_640_480.py

"""
Temporary Logitech C270 640x480 calibration.

The active calibration below is taken from the
FIRST Tech Challenge C270 calibration published in
teamwebcamcalibrations.xml.

Camera:
    Logitech HD Webcam C270

Resolution:
    640x480

Calibration method:
    3DF Zephyr

This is a bootstrap calibration only.

It should be replaced by calibration of the actual
physical camera before relying on high-accuracy PnP.
"""


# --------------------------------------------------
# Internet calibration references
# --------------------------------------------------

# PRIMARY / ACTIVE
#
# FIRST Tech Challenge C270 calibration:
#
# focalLength:
#     822.317, 822.317
#
# principalPoint:
#     319.495, 242.502
#
# distortionCoefficients:
#     -0.0449369, 1.17277, 0, 0, -3.63244, 0, 0, 0
#
# Our calibration format currently uses the standard
# five OpenCV coefficients:
#
#     k1, k2, p1, p2, k3
#
# The remaining published coefficients are zero.


# --------------------------------------------------
# Alternative internet cross-check
# --------------------------------------------------
#
# Another independently published C270 ROS calibration:
#
# CAMERA_PARAMS:
#     fx = 956.467512
#     fy = 952.034613
#     cx = 322.227734
#     cy = 211.367552
#
# DISTORTION:
#     0.111399
#    -0.110521
#    -0.008289
#     0.015730
#     0.000000
#
# IMPORTANT:
# The source does not clearly state the capture
# resolution, so these values are NOT used.
#
# They are retained only as evidence that calibration
# varies appreciably between C270 setups.


# --------------------------------------------------
# Legacy 2D / bearing-distance calibration
# --------------------------------------------------
#
# Used by cam1_markers2 / triangulation-style localisation.

CAMERA_PARAMS = (
    822.317,
    822.317,
    319.495,
    242.502,
)

DISTORTION_COEFFICIENTS = (
    -0.0449369,
    1.17277,
    0.0,
    0.0,
    -3.63244,
)


# --------------------------------------------------
# PnP calibration
# --------------------------------------------------
#
# Used only by AprilTagPnPPoseProvider.
#
# Until we calibrate this physical C270, use the same
# published C270 calibration for PnP.

PNP_CAMERA_PARAMS = (
    822.317,
    822.317,
    319.495,
    242.502,
)

PNP_DISTORTION_COEFFICIENTS = (
    -0.0449369,
    1.17277,
    0.0,
    0.0,
    -3.63244,
)


# --------------------------------------------------
# Metadata
# --------------------------------------------------

RESOLUTION = (640, 480)

# Horizontal FoV derived from fx=822.317 at 640 px:
#
#     HFOV ~= 42.5 degrees
#
# Logitech specifies approximately 55 degrees diagonal FoV.
#
# This matches the convention used by the Arducam
# calibration, where FOV_DEG represents horizontal FoV.
FOV_DEG = 42.5

DESCRIPTION = (
    "Logitech C270 full-FoV 640x480 bootstrap calibration "
    "using published FTC C270 intrinsics"
)

NOTES = (
    "Temporary calibration derived from a published Logitech C270 "
    "640x480 calibration. Replace with calibration of this physical "
    "camera before treating PnP pose estimates as authoritative."
)