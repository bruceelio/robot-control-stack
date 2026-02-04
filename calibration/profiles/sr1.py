# calibration/profiles/sr1.py

"""
Calibration profile for SR1 robot.

This file defines *physical truth* for the SR1 platform.

Rules:
- No logic
- No behavior
- No conditionals
- Only measured or declared constants

All values here are interpreted by higher layers and must
remain stable during runtime.
"""

# ==================================================
# Motion calibration (TimedMotionBackend)
# ==================================================

# Distance at which drive profile switches from short to long
DRIVE_SWITCH_MM = 800

# Motor power levels
DRIVE_POWER_SHORT = 0.65
DRIVE_POWER_LONG  = 0.85

# Linear time model (t = m * d + b)
DRIVE_M_SHORT = 0.00133
DRIVE_B_SHORT = 0.06

DRIVE_M_LONG  = 0.00112
DRIVE_B_LONG  = 0.09

# Rotation calibration (in-place)
ROTATE_POWER = 0.55
ROTATE_M     = 0.0056
ROTATE_B     = 0.18


# ==================================================
# Camera calibration
# ==================================================

"""
Camera calibration defines how camera observations map into
the robot coordinate frame.

Conventions (CANONICAL):
- +X forward
- +Y left
- +bearing = counter-clockwise (left)
- Angles in degrees
"""

CAMERAS = {
    "front": {
        # ----------------------------------------------
        # Physical mounting (robot frame)
        # ----------------------------------------------

        # Camera orientation relative to robot forward
        "yaw_offset_deg": 0.0,

        # Camera position relative to robot center
        "x_offset_mm": 0.0,
        "y_offset_mm": 0.0,

        # ----------------------------------------------
        # Optical / perception calibration
        # ----------------------------------------------

        # Distance scaling factor (camera → real world)
        "distance_scale": 1.0,

        # Bearing convention correction
        # Raw camera bearing:
        #   +ve = left (CCW), -ve = right (CW)
        "bearing_sign": 1.0,

        # Static angular misalignment correction
        "bearing_offset_deg": 0.0,

        # ----------------------------------------------
        # Metadata (non-functional, informational)
        # ----------------------------------------------

        "resolution": (640, 480),
        "fov_deg": 60.0,
        "description": "Front-facing primary camera",
    }
}

# End of SR1 calibration profile

