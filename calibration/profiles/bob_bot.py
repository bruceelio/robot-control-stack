# calibration/profiles/bob_bot.py

"""
Calibration profile for simulation robot.

This file defines physical truth:
- Motor timing calibration
- Camera mounting and optical corrections

Values here are NOT policy and MUST NOT be changed at runtime.
"""

# --------------------------------------------------
# Timed Drive calibration
# --------------------------------------------------

DRIVE_SWITCH_MM = 800

# Power levels (open-loop)
DRIVE_POWER_SHORT = 0.20
DRIVE_POWER_LONG  = 0.35

# Distance → time calibration
DRIVE_M_SHORT = 0.00421
DRIVE_B_SHORT = -0.0063

DRIVE_M_LONG  = 0.00208
DRIVE_B_LONG  = -0.030

# =========================
# Timed Rotation Calibration
# =========================

ROTATE_SWITCH_DEG = 10  # 30 deg is ideal?

# Small-angle rotations (precision)
ROTATE_POWER_SMALL = 0.10
ROTATE_M_SMALL = 0.025
ROTATE_B_SMALL = 0.0

# Large-angle rotations (momentum) (if under roting need to increase)
ROTATE_POWER_LARGE = 0.20
ROTATE_M_LARGE = 0.015   # sec/deg (greater number is more time turning per deg)
ROTATE_B_LARGE = -0.0413


# Rotation calibration
# ROTATE_POWER = 0.50
# ROTATE_M = 0.0051
# ROTATE_B = 0.15


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
            "distance_scale": 1.0,     # sim camera overestimates distance
            "bearing_sign": 1.0,        # image X axis inverted vs robot yaw
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

