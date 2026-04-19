# config/profiles/bob_bot.py

from .simulation import *  # noqa

ROBOT_ID = "bob_bot"
HARDWARE_PROFILE = "bob_bot"
ENVIRONMENT = "real"
SURFACE = "tile"

CAMERAS = {
    "front": "pi3",
}

SURFACE_MULTIPLIERS = {
    "simulation": {"rotate": 1.00, "drive": 1.00},
    "tile": {"rotate": 1.00, "drive": 1.00},
}

ENCODERS = {
    "deadwheel_parallel": "gobilda_4bar_odometry_pod_32mm",
    "deadwheel_perpendicular": "gobilda_swingarm_odometry_pod_48mm",
#    "shooter": "gobilda_yellowjacket_312rpm",
}

# Lower commanded power
MAX_MOTOR_POWER = 0.5

# Increase timed duration scaling
BASE_ROTATE_FACTOR = 1.0
BASE_DRIVE_FACTOR = 1.0

# InitEscape (400, 40)
INIT_ESCAPE_DRIVE_MM = 0
INIT_ESCAPE_ROTATE_DEG = 0.0

CAMERA_SETTLE_TIME = 0.8
CAMERA_FRESH_OBS_MAX_AGE_S = 0.12

# These are actual radians (so AI doesn't implode on itself)
MARKER_PITCH_HIGH_DEG = -0.05               # 0.0523598776 is 3 degrees
MARKER_PITCH_LOW_DEG = 0.05
HEIGHT_DECISION_DEADLINE_MM = 1500            # cannot be low are will never commit
MARKER_HEIGHT_MAX_DISTANCE_MM = 6000

FINAL_APPROACH_MARKER_PUSH = 220

"""
# All poses are relative to base_link:
# base_link = midpoint between the two drive wheels
# +x forward, +y left, +z up

CAMERA_MOUNTS = {
    "front": {
        "x_mm": 85.0,
        "y_mm": -110.0,
        "z_mm": 210.0,
        "roll_deg": 0.0,
        "pitch_deg": -18.0,
        "yaw_deg": 8.0,
    }
}

GRIPPER_MOUNT = {
    "x_mm": 160.0,
    "y_mm": 0.0,
    "z_mm": 35.0,
    "roll_deg": 0.0,
    "pitch_deg": 0.0,
    "yaw_deg": 0.0,
}
"""