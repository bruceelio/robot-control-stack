# config/base.py

from enum import Enum, auto

class RunMode(Enum):
    NORMAL = auto()
    TESTS = auto()
    DIAGNOSTICS = auto()

# --------------------------------------------------
# SELECT EXACTLY ONE
# --------------------------------------------------

RUN_MODE = RunMode.NORMAL
# RUN_MODE = RunMode.TESTS
# RUN_MODE = RunMode.DIAGNOSTICS


ARENA_SIZE = 6000  # mm

DEFAULT_TARGET_KIND   = "basic"   # or "basic"

# Marker height decision distances
MARKER_HEIGHT_MAX_DISTANCE_MM = 1000   # beyond this, pitch unreliable
FINAL_COMMIT_DISTANCE_MM      = 500    # blind execution threshold

# Pitch thresholds (simulation tuned; robot overrides later)
MARKER_PITCH_HIGH_DEG = 0.35   # platform markers cluster ~0.5
MARKER_PITCH_LOW_DEG  = 0.10   # floor markers cluster ~0.0

CAMERA_SETTLE_TIME = 0.15  # seconds (tune later)

MIN_ROTATE_DEG = 2
MAX_ROTATE_DEG     = 90

MIN_DRIVE_MM   = 5.0
MAX_DRIVE_MM   = 550

ALIGN_THRESHOLD_DEG = 6.0
GRAB_DISTANCE_MM   = 120


# Supported options (validation only)
VALID_SURFACES = ("simulation", "tile", "wood", "carpet")
VALID_ENVIRONMENTS = ("simulation", "real")
VALID_DRIVE_LAYOUTS = ("2WD", "3WD", "4WD")
VALID_WHEEL_TYPES = ("standard", "mecanum", "omni", "tracked")
VALID_ROBOT_IDS = ("sim", "sr1", "sr2")

# Base calibration factors
BASE_ROTATE_FACTOR = 1.0
BASE_DRIVE_FACTOR = 1.0
BASE_DISTANCE_SCALE = 1.0

# Surface multipliers
SURFACE_MULTIPLIERS = {
    "simulation": {"rotate": 1.00, "drive": 1.00},
    "tile":       {"rotate": 0.92, "drive": 0.95},
    "wood":       {"rotate": 0.88, "drive": 0.90},
    "carpet":     {"rotate": 1.15, "drive": 1.10},
}

# Environment distance scales
DISTANCE_SCALES = {
    "simulation": 1.327,
    "real":       1.00,
}

# Robot hardware configs
ROBOT_CONFIGS = {
    "sim": {
        "motor_polarity": {
            "2WD": [1, 1],
            "3WD": [1, 1, 1],
            "4WD": [1, 1, 1, 1],
        },
    },
    "sr1": {
        "motor_polarity": {
            "2WD": [1, -1],
            "3WD": [1, -1, 1],
            "4WD": [1, -1, 1, -1],
        },
    },
    "sr2": {
        "motor_polarity": {
            "2WD": [-1, 1],
            "3WD": [-1, 1, -1],
            "4WD": [-1, 1, -1, 1],
        },
    },
}
# Motion backend selection
VALID_MOTION_BACKENDS = ("timed", "encoder")
MOTION_BACKEND = "timed"   # "timed" | "encoder" | "trajectory"
