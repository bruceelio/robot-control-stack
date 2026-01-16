from enum import Enum, auto


class RunMode(Enum):
    NORMAL = auto()
    TESTS = auto()
    DIAGNOSTICS = auto()

RUN_MODE = RunMode.NORMAL
# RUN_MODE = RunMode.TESTS
# RUN_MODE = RunMode.DIAGNOSTICS

DEFAULT_TARGET_KIND = "basic"  # "acidic" or "basic"

# Vision & perception
MARKER_HEIGHT_MAX_DISTANCE_MM = 1000
FINAL_COMMIT_DISTANCE_MM = 500

MARKER_PITCH_HIGH_DEG = 0.35
MARKER_PITCH_LOW_DEG  = 0.10

CAMERA_SETTLE_TIME = 0.15

# Motion constraints (planner-level)
MIN_ROTATE_DEG = 2
MAX_ROTATE_DEG = 90

MIN_DRIVE_MM = 5.0
MAX_DRIVE_MM = 550

ALIGN_THRESHOLD_DEG = 6.0

VALID_MOTION_BACKENDS = ("timed", "encoder")
MOTION_BACKEND = "timed"