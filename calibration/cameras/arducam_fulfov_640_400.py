# calibration/cameras/arducam_fullfov_640_400.py

# CAMERA_PARAMS = (342.4, 342.9, 341.5, 199.1)                          # first pass; 1280 x 800 conversion
# CAMERA_PARAMS = (349.2, 348.5, 335.0, 212.2)                          # latest test; 1280 x 800 conversion

# DISTORTION_COEFFICIENTS = (-0.347, 0.148, 0.002, -0.001, -0.032)      # first pass; 1280 x 800 conversion
# DISTORTION_COEFFICIENTS = (-0.332, 0.119, 0.0, 0.0, -0.021)           # latest test; 1280 x 800 conversion


# --------------------------------------------------
# Legacy 2D / bearing-distance calibration
# --------------------------------------------------
# Used by cam1_markers2 / triangulation-style localisation.

CAMERA_PARAMS = (349.2, 348.5, 335.0, 212.2)
DISTORTION_COEFFICIENTS = (-0.332, 0.119, 0.0, 0.0, -0.021)

# --------------------------------------------------
# PnP calibration
# --------------------------------------------------
# Used only by AprilTagPnPPoseProvider.

PNP_CAMERA_PARAMS = (349.2, 348.5, 335.0, 212.2)
PNP_DISTORTION_COEFFICIENTS = (-0.332, 0.119, 0.0, 0.0, -0.021)

