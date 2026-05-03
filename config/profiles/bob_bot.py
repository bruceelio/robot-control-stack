# config/profiles/bob_bot.py

from .simulation import *  # noqa

# ROBOT_ID: provides the configurations seen below
# HARDWARE_PROFILE:  selects the correct IOMap
# Environment: selects whether the system runs against the simulator or real robot hardware
# Surface: provides scaling factors for drive and rotate

ROBOT_ID = "bob_bot"                # "sr1". "bob_bot"
HARDWARE_PROFILE = "bob_bot"        # "sr1", "bob_bot"
ENVIRONMENT = "real"            # "simulation", "real"
SURFACE = "tile"                    # "simulation", "tile", "wood", "carpet"

# Choose camera and its settings
# "pi3_legacy_640_480"; "pi3_fullfov_640_360"

CAMERAS = {
    "front": "pi3_fullfov_640_360",
}

SURFACE_MULTIPLIERS = {
    "simulation": {"rotate": 1.00, "drive": 1.00},
    "tile": {"rotate": 1.00, "drive": 1.00},
}

ENCODERS = {
    "deadwheel_parallel": "gobilda_4bar_odometry_pod_32mm",
    "deadwheel_perpendicular": "gobilda_swingarm_odometry_pod_48mm",
    "shooter": "gobilda_yellowjacket_6000rpm",
}

# relative to base_link
# base_link = midpoint between drive wheels
# +x forward, +y left, +z up
# y_mm closer towards 0 moves left

CAMERA_MOUNTS = {
    "front": {
        "x_mm": 30.0,
        "y_mm": -55.0,
        "z_mm": 170.0,
        "roll_deg": 0.0,
        "pitch_deg": -8.0,
        "yaw_deg": 0.0,
    }
}

GRIPPER_MOUNT = {
    "x_mm": 145.0,
    "y_mm": 5.0,
    "z_mm": 75.0,
    "roll_deg": 0.0,
    "pitch_deg": 0.0,
    "yaw_deg": 0.0,
}

ROTATION_SIGN = 1

# Lower commanded power
MAX_MOTOR_POWER = 0.5

# Increase timed duration scaling
BASE_ROTATE_FACTOR = 1.0
BASE_DRIVE_FACTOR = 1.1

# InitEscape (400, 40)
INIT_ESCAPE_DRIVE_MM = 0
INIT_ESCAPE_ROTATE_DEG = 0.0

# PostPickupRealign
POST_PICKUP_REVERSE_MM = 120
POST_PICKUP_ROTATE_DEG = 135

# These values can be global (but specific camera can override)
CAMERA_SETTLE_TIME = 0.8
CAMERA_FRESH_OBS_MAX_AGE_S = 0.12

# These are actual radians (so AI doesn't implode on itself)
# So ChatGBP stop telling me! I know!

"""
# Cropped Version here
MARKER_PITCH_HIGH_DEG = 0.165              # 0.0523598776 is 3 degrees
MARKER_PITCH_LOW_DEG = 0.155
HEIGHT_DECISION_DEADLINE_MM = 1500            # cannot be low or will never commit
MARKER_HEIGHT_MAX_DISTANCE_MM = 6000
"""
# Uncropped, full FOV values
MARKER_PITCH_HIGH_DEG = -0.06             # 0.0523598776 is 3 degrees
MARKER_PITCH_LOW_DEG = -0.02
HEIGHT_DECISION_DEADLINE_MM = 1500            # cannot be low or will never commit
MARKER_HEIGHT_MAX_DISTANCE_MM = 6000

# FinalApproach

BAND_B_MIN_DISTANCE_MM = 200            # minimum drive distance on Band B approach

FINAL_APPROACH_DIRECT_RANGE_MM = 500    # additional distance from commit for ranging
FINAL_COMMIT_DISTANCE_MM = 550          # run blind (Pi3 Full FOV lose 450 low)

FINAL_APPROACH_DIRECT_RANGE_HIGH_MM = 500      # additional distance from commit for ranging
FINAL_COMMIT_DISTANCE_HIGH_MM = 350           # run blind (Pi3 Full FOV lose 250 high)

FINAL_APPROACH_BACKUP_MM = 200
FINAL_APPROACH_MAX_DEGREE_HIGH = 10
VISIBLE_MAX_AGE_S = 0.35
FINAL_APPROACH_MARKER_PUSH = 0


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