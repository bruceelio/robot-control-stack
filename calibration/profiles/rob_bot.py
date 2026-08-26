# calibration/profiles/rob_bot.py

"""
Calibration profile for rob_bot

This file defines physical truth:
- Motor timing calibration
- Camera mounting and optical corrections

Values here are NOT policy and MUST NOT be changed at runtime.
"""

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

