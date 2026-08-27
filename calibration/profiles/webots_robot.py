# calibration/profiles/webots_robot.py
"""
Calibration profile for simulation robot.

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
            "yaw_offset_deg": 0.0,
            "x_offset_mm": 0.0,
            "y_offset_mm": 0.0,
        },

        # ------------------------------------------
        # Optical / perception correction
        # ------------------------------------------
        "optical": {
            "distance_scale": 0.50,
            "bearing_sign": 1.0,
            "bearing_offset_deg": 0.0,
        },

        # ------------------------------------------
        # Metadata (non-functional)
        # ------------------------------------------
        "meta": {
            "resolution": (640, 480),
            "fov_deg": 60.0,
            "description": "Forward-facing AprilTag camera (Webots)",
        },
    },
}
