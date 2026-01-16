# calibration/profiles/simulation.py

"""
Calibration profile for simulation robot.

This file defines physical truth:
- Motor timing calibration
- Camera mounting and optical corrections

Values here are NOT policy and MUST NOT be changed at runtime.
"""

# --------------------------------------------------
# Timed motion calibration
# --------------------------------------------------

DRIVE_SWITCH_MM = 1000

# Power levels (open-loop)
DRIVE_POWER_SHORT = 0.60
DRIVE_POWER_LONG  = 0.85

# Distance → time calibration
DRIVE_M_SHORT = 0.0011
DRIVE_B_SHORT = 0.06

DRIVE_M_LONG  = 0.00078
DRIVE_B_LONG  = 0.05

# Rotation calibration
ROTATE_POWER = 0.50
ROTATE_M = 0.0051
ROTATE_B = 0.15


# --------------------------------------------------
# Camera calibration
# --------------------------------------------------

CAMERAS = {
    "front": {
        # ------------------------------------------
        # Physical mounting (robot frame)
        # ------------------------------------------
        "mount": {
            "yaw_offset_deg": 0.0,   # camera faces forward
            "x_offset_mm": 0.0,      # centered (sim)
            "y_offset_mm": 0.0,
        },

        # ------------------------------------------
        # Optical / perception correction
        # ------------------------------------------
        "optical": {
            "distance_scale": 1.327,     # sim camera overestimates distance
            "bearing_sign": -1.0,        # image X axis inverted vs robot yaw
            "bearing_offset_deg": 0.0,   # no constant bias
        },

        # ------------------------------------------
        # Metadata (non-functional)
        # ------------------------------------------
        "meta": {
            "resolution": (640, 480),
            "fov_deg": 60.0,
            "description": "Forward-facing AprilTag camera (simulation)",
        },
    },

    # Example future camera
    # "rear": {
    #     "mount": {...},
    #     "optical": {...},
    #     "meta": {...},
    # }
}

