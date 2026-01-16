# config/simulation.py

ROBOT_ID = "sim"
ENVIRONMENT = "simulation"
SURFACE = "simulation"
DRIVE_LAYOUT = "2WD"
WHEEL_TYPE = "standard"

# -------------------------
# Strategy
# -------------------------
MOTION_BACKEND = "timed"

# -------------------------
# Calibration bases
# -------------------------
BASE_ROTATE_FACTOR = 1.0
BASE_DRIVE_FACTOR = 1.0
BASE_DISTANCE_SCALE = 1.0

# -------------------------
# Behavior Constants
# -------------------------
INIT_ESCAPE_DRIVE_MM = 150
INIT_ESCAPE_ROTATE_DEG = 10

POST_PICKUP_REVERSE_MM = 200
POST_PICKUP_ROTATE_DEG = 90

RECOVER_STEP_DEG = 30
RECOVER_MAX_SWEEP_DEG = 360
RECOVER_SETTLE_TIME = 0.30

# Robot hardware
MOTOR_POLARITY = [1, 1, 1, 1]
GRAB_DISTANCE_MM = 120

# -------------------------
# Motion safety limits
# -------------------------
MIN_ROTATE_DEG = 3.0
MIN_DRIVE_MM = 10.0

MAX_DRIVE_MM = 1500.0
MAX_ROTATE_DEG = 45.0

CAMERA_SETTLE_TIME = 0.3

MARKER_HEIGHT_MAX_DISTANCE_MM = 2000.0
MARKER_PITCH_HIGH_DEG = 10.0
MARKER_PITCH_LOW_DEG = 3.0

FINAL_COMMIT_DISTANCE_MM = 300.0

VISION_LOSS_TIMEOUT_S = 0.5
