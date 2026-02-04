# config/profiles/simulation.py

# Identity
ROBOT_ID = "sim"
HARDWARE_PROFILE = "sr1"
ENVIRONMENT = "simulation"
SURFACE = "simulation"

# Drive hardware
DRIVE_LAYOUT = "2WD"
WHEEL_TYPE = "standard"
MOTOR_POLARITY = [1, 1]
MOTION_BACKEND = "timed"

# Base calibration
BASE_ROTATE_FACTOR = 1.0
BASE_DRIVE_FACTOR = 1.0

SURFACE_MULTIPLIERS = {
    "simulation": {"rotate": 1.00, "drive": 1.00},
}

# Grabbing
GRAB_DISTANCE_MM = 0.0   # not currently in code?

# InitEscape
INIT_ESCAPE_DRIVE_MM = 280
INIT_ESCAPE_ROTATE_DEG = 27.0

# PostPickupRealign
POST_PICKUP_REVERSE_MM = 120
POST_PICKUP_ROTATE_DEG = 150.0

# PostDropoffRealign
POST_DROPOFF_REVERSE_MM = 120
POST_DROPOFF_ROTATE_DEG = 90

# RecoverLocalisation
RECOVER_STEP_DEG = 15.0
RECOVER_MAX_SWEEP_DEG = 180.0
RECOVER_SETTLE_TIME = 0.5

# Motion limits
MIN_ROTATE_DEG = 2.0
MAX_ROTATE_DEG = 90.0
MIN_DRIVE_MM = 5.0          # only for seek_and_collect
MAX_DRIVE_MM = 2500.0
MAX_MOTOR_POWER = 0.8

# Vision / perception
CAMERA_SETTLE_TIME = 0.5
MARKER_HEIGHT_MAX_DISTANCE_MM = 2000

# These are actual radians (so AI doesn't implode on itself)
MARKER_PITCH_HIGH_DEG = 0.052                # 0.0523598776 is 3 degrees
MARKER_PITCH_LOW_DEG = 0.02
HEIGHT_DECISION_DEADLINE_MM = 1500            # cannot be low are will never commit
MARKER_HEIGHT_MAX_DISTANCE_MM = 6000


VISION_LOSS_TIMEOUT_S = 0.5
VISION_GRACE_PERIOD_S = 0.3             # for policy/vision_grace_period.py

# FinalApproach
FINAL_APPROACH_DIRECT_RANGE_MM = 500    # additional distance from commit for ranging
FINAL_COMMIT_DISTANCE_MM = 650          # from here we go blind
FINAL_APPROACH_BACKUP_MM = 200

FINAL_APPROACH_DIRECT_RANGE_HIGH_MM = 500      # additional distance from commit for ranging
FINAL_COMMIT_DISTANCE_HIGH_MM = 1100            # from here we go blind
FINAL_APPROACH_MAX_DEGREE_HIGH = 10
VISIBLE_MAX_AGE_S = 0.35
FINAL_APPROACH_MARKER_PUSH = 13

# ReacquireTarget
REACQUIRE_TARGET_VISION_LOSS = 20

# BackoffScan
BACKOFF_SCAN_MM = 200.0
BACKOFF_SCAN_CAP_DEG = 60.0
BACKOFF_SCAN_STEP_DEG = 20.0
BACKOFF_SCAN_TIMEOUT_S = 3.0

# --------------------------------------------------
# Wall / ultrasonic geometry (navigation)
# --------------------------------------------------

# Which wall-angle backend to use
#   "one_ultrasonic_scan"
#   "two_ultrasonics"
WALL_ANGLE_BACKEND = "one_ultrasonic_scan"


# ---- Two-ultrasonic configuration ----
# Keys must exist in io.ultrasonics()
WALL_TWO_ULTRASONIC_KEYS = ("left", "right")

# Physical separation between the two ultrasonic sensors (mm)
WALL_TWO_ULTRASONIC_BASELINE_MM = 160.0


# ---- One-ultrasonic scan configuration ----
# Key must exist in io.ultrasonics()
WALL_ONE_ULTRASONIC_KEY = "front"

# Relative scan angles (robot frame, degrees)
WALL_SCAN_ANGLE_1_DEG = -8.0
WALL_SCAN_ANGLE_2_DEG = 8.0

# Number of samples taken at each scan angle (per side)
WALL_SCAN_SAMPLES_PER_ANGLE = 3

# Time to wait after rotation before sampling (sensor + motion settle)
WALL_SCAN_SETTLE_TIME_S = 0.10


# ---- Ultrasonic sanity limits ----
WALL_ULTRASONIC_MIN_MM = 50.0
WALL_ULTRASONIC_MAX_MM = 2500.0


# ---- Wall angle filtering / stability ----
# Require N consecutive valid angle estimates before we "trust" it.
WALL_ANGLE_STABLE_SAMPLES = 2

# If we haven’t refreshed an estimate in this long, treat it stale.
WALL_ANGLE_MAX_AGE_S = 0.25


# ---- Parallel-to-wall control ----
# How close (in degrees) we need to be to parallel before success
WALL_PARALLEL_TOLERANCE_DEG = 3.0

# Start parallel-to-wall action when abs(error) exceeds this
WALL_PARALLEL_TRIGGER_DEG = 10.0    # typically same as FINAL_APPROACH_MAX_DEGREE_HIGH

# Safety caps on how much we rotate while trying to parallel
WALL_PARALLEL_MAX_ROTATE_DEG = 15.0
WALL_PARALLEL_STEP_DEG = 5.0
WALL_PARALLEL_TIMEOUT_S = 4.0



