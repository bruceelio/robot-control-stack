# config/profiles/sr1.py

ROBOT_ID = "sr1"

# Environment
ENVIRONMENT = "simulation"
SURFACE = "simulation"

# Drive
DRIVE_LAYOUT = "tank"
WHEEL_TYPE = "rubber"
MOTION_BACKEND = "timed"

# Strategy
DEFAULT_TARGET_KIND = "basic"

# Robot physical facts
MOTOR_POLARITY = [1, 1, 1, 1]
GRAB_DISTANCE_MM = 120

# Calibration
BASE_ROTATE_FACTOR = 1.0
BASE_DRIVE_FACTOR = 1.0
BASE_DISTANCE_SCALE = 1.0

# InitEscape
INIT_ESCAPE_DRIVE_MM = 150
INIT_ESCAPE_ROTATE_DEG = 10

# PostPickupRealign
POST_PICKUP_REVERSE_MM = 200
POST_PICKUP_ROTATE_DEG = 90

# RecoverLocalisation
RECOVER_STEP_DEG = 30
RECOVER_MAX_SWEEP_DEG = 360
RECOVER_SETTLE_TIME = 0.30

