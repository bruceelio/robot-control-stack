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