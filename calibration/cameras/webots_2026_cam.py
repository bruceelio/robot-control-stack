# calibration/cameras/webots_2026_cam.py

"""
Camera / vision calibration for the Student Robotics 2026 Webots simulator.

Webots does not use a physical camera calibration in the same way as
Pi/USB cameras.

The SR API supplies marker observations through:

    robot.camera.see()

The values in this file are therefore applied to the observations
after they have been returned by the simulator.

Physical camera mounting geometry does not belong here.
"""


# --------------------------------------------------
# Optical / perception correction
# --------------------------------------------------

# Webots reported marker distance correction.
DISTANCE_SCALE = 1.0

# Bearing direction correction.
BEARING_SIGN = 1.0

# Constant bearing correction.
BEARING_OFFSET_DEG = 0.0


# --------------------------------------------------
# Metadata
# --------------------------------------------------

RESOLUTION = (640, 480)

FOV_DEG = 60.0

DESCRIPTION = "Student Robotics 2026 Webots simulated camera"