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
BASE_ROTATE_FACTOR = 2.5
BASE_DRIVE_FACTOR = 2.5

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