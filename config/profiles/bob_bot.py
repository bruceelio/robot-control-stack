# config/profiles/bob_bot.py

# ==================================================
# 1. ROBOT HARDWARE / CAPABILITIES
# ==================================================

# ROBOT_ID:
# Identifies this particular robot configuration.
#
# HARDWARE_PROFILE:
# Selects the IOMap implementation appropriate to the robot hardware.
#
# ENVIRONMENT:
# Selects whether the robot is running against real or simulated hardware.

ROBOT_ID = "bob_bot"
HARDWARE_PROFILE = "bob_bot"
ENVIRONMENT = "real"


# -------------------------
# Cameras / Vision Hardware
# -------------------------

CAMERAS = {
    "front": "pi3_fullfov_640_360",
}

VISION_SOURCES = {
    "vision1": {
        "camera": "front",
        "provider": "apriltag_pnp",
        "enabled": True,
    }
}

ASYNC_VISION_ENABLED = True


# -------------------------
# Encoders
# -------------------------

ENCODERS = {
#    "deadwheel_parallel": "gobilda_4bar_odometry_pod_32mm",
#    "deadwheel_perpendicular": "gobilda_swingarm_odometry_pod_48mm",
#    "shooter": "gobilda_yellowjacket_6000rpm",
}


# -------------------------
# Voltage / Battery
# -------------------------

VOLTAGE_SENSORS = {
    "battery": "stemedu_voltage_sensor_0_25v",
}

BATTERY_VOLTAGE_NOMINAL = 14.17


# -------------------------
# Physical Geometry
# -------------------------

# All poses are relative to base_link:
# base_link = midpoint between drive wheels
# +x forward, +y left, +z up

CAMERA_MOUNTS = {
    "front": {
        "x_mm": 40.0,
        "y_mm": -90.0,
        "z_mm": 215.0,
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


"""
# Previous camera / gripper geometry

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


# -------------------------
# Motion Hardware Limits
# -------------------------

DRIVE_LAYOUT = "2WD"
WHEEL_TYPE = "standard"

MOTOR_POLARITY = [1, 1]

MOTION_BACKEND = "timed"

ROTATION_SIGN = 1
MAX_MOTOR_POWER = 1.0




# Generic motion command limits

MIN_ROTATE_DEG = 2.0
MAX_ROTATE_DEG = 180.0

MIN_DRIVE_MM = 5.0
MAX_DRIVE_MM = 2500.0



# ==================================================
# 2. DEVICE / CALIBRATION SELECTION & SCALING
# ==================================================

# Surface provides scaling factors for drive and rotate.

SURFACE = "tile"

SURFACE_MULTIPLIERS = {
    "simulation": {"rotate": 1.00, "drive": 1.00},
    "tile": {"rotate": 1.00, "drive": 1.00},
}

BASE_ROTATE_FACTOR = 1.0
BASE_DRIVE_FACTOR = 1.0



# ==================================================
# 3. AUTONOMOUS / PERCEPTION TUNING
# ==================================================

# -------------------------
# Initial Escape
# -------------------------

# InitEscape (400, 40)
INIT_ESCAPE_DRIVE_MM = 0
INIT_ESCAPE_ROTATE_DEG = 0.0


# -------------------------
# Post-Pickup Realignment
# -------------------------

POST_PICKUP_REVERSE_MM = 120
POST_PICKUP_ROTATE_DEG = 135

# -------------------------
# Post-Dropoff Realignment
# -------------------------

POST_DROPOFF_REVERSE_MM = 120
POST_DROPOFF_ROTATE_DEG = 90


# -------------------------
# Localisation Recovery
# -------------------------

RECOVER_STEP_DEG = 15.0
RECOVER_MAX_SWEEP_DEG = 180.0
RECOVER_SETTLE_TIME = 0.5



# -------------------------
# Vision Timing
# -------------------------

# These values can be global
# (specific cameras can override them)

CAMERA_SETTLE_TIME = 0.8
CAMERA_FRESH_OBS_MAX_AGE_S = 0.12

VISION_LOSS_TIMEOUT_S = 0.5
VISION_GRACE_PERIOD_S = 0.3

REACQUIRE_TARGET_VISION_LOSS = 20


# -------------------------
# Marker Height Detection
# -------------------------

# These are actual radians.

"""
# Cropped Version
MARKER_PITCH_HIGH_DEG = 0.165
MARKER_PITCH_LOW_DEG = 0.155
HEIGHT_DECISION_DEADLINE_MM = 1500
MARKER_HEIGHT_MAX_DISTANCE_MM = 6000
"""

# Uncropped, full FOV values
MARKER_PITCH_HIGH_DEG = -0.06
MARKER_PITCH_LOW_DEG = -0.02

HEIGHT_DECISION_DEADLINE_MM = 1500
MARKER_HEIGHT_MAX_DISTANCE_MM = 6000


# -------------------------
# Final Approach
# -------------------------

BAND_B_MIN_DISTANCE_MM = 200

FINAL_APPROACH_DIRECT_RANGE_MM = 500
FINAL_COMMIT_DISTANCE_MM = 550

FINAL_APPROACH_DIRECT_RANGE_HIGH_MM = 500
FINAL_COMMIT_DISTANCE_HIGH_MM = 350

FINAL_APPROACH_BACKUP_MM = 200
FINAL_APPROACH_MAX_DEGREE_HIGH = 10

VISIBLE_MAX_AGE_S = 0.35
FINAL_APPROACH_MARKER_PUSH = 0

# -------------------------
# Backoff Scan
# -------------------------

BACKOFF_SCAN_MM = 200.0
BACKOFF_SCAN_CAP_DEG = 60.0
BACKOFF_SCAN_STEP_DEG = 20.0
BACKOFF_SCAN_TIMEOUT_S = 3.0


# -------------------------
# Wall / Ultrasonic Navigation
# -------------------------

# Which wall-angle backend to use:
# "one_ultrasonic_scan"
# "two_ultrasonics"

WALL_ANGLE_BACKEND = "one_ultrasonic_scan"


# Two-ultrasonic configuration

WALL_TWO_ULTRASONIC_KEYS = ("left", "right")
WALL_TWO_ULTRASONIC_BASELINE_MM = 160.0


# One-ultrasonic scan configuration

WALL_ONE_ULTRASONIC_KEY = "front"

WALL_SCAN_ANGLE_1_DEG = -8.0
WALL_SCAN_ANGLE_2_DEG = 8.0
WALL_SCAN_SAMPLES_PER_ANGLE = 3
WALL_SCAN_SETTLE_TIME_S = 0.10


# Ultrasonic sanity limits

WALL_ULTRASONIC_MIN_MM = 50.0
WALL_ULTRASONIC_MAX_MM = 2500.0


# Wall-angle filtering / stability

WALL_ANGLE_STABLE_SAMPLES = 2
WALL_ANGLE_MAX_AGE_S = 0.25


# Parallel-to-wall control

WALL_PARALLEL_TOLERANCE_DEG = 3.0
WALL_PARALLEL_TRIGGER_DEG = 10.0

WALL_PARALLEL_MAX_ROTATE_DEG = 15.0
WALL_PARALLEL_STEP_DEG = 5.0
WALL_PARALLEL_TIMEOUT_S = 4.0

GRAB_DISTANCE_MM = 0.0 # not currently in code?


