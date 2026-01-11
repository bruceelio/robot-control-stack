# config/__init__.py

from .base import *
from .simulation import *   # swap for competition/testing later

# -------------------------
# Safety checks
# -------------------------
if ENVIRONMENT not in VALID_ENVIRONMENTS:
    raise ValueError(f"Unknown ENVIRONMENT '{ENVIRONMENT}'")

if SURFACE not in VALID_SURFACES:
    raise ValueError(f"Unknown SURFACE '{SURFACE}'")

if DRIVE_LAYOUT not in VALID_DRIVE_LAYOUTS:
    raise ValueError(f"Unknown DRIVE_LAYOUT '{DRIVE_LAYOUT}'")

if WHEEL_TYPE not in VALID_WHEEL_TYPES:
    raise ValueError(f"Unknown WHEEL_TYPE '{WHEEL_TYPE}'")

if ROBOT_ID not in VALID_ROBOT_IDS:
    raise ValueError(f"Unknown ROBOT_ID '{ROBOT_ID}'")


# -------------------------
# Apply calibration
# -------------------------
rotate_factor = (
    BASE_ROTATE_FACTOR *
    SURFACE_MULTIPLIERS[SURFACE]["rotate"]
)

drive_factor = (
    BASE_DRIVE_FACTOR *
    SURFACE_MULTIPLIERS[SURFACE]["drive"]
)

distance_scale = (
    BASE_DISTANCE_SCALE *
    DISTANCE_SCALES[ENVIRONMENT]
)

motor_polarity = ROBOT_CONFIGS[ROBOT_ID]["motor_polarity"][DRIVE_LAYOUT]

# Wheel-type adjustments
if WHEEL_TYPE in ("mecanum", "omni"):
    rotate_factor *= 1.05
    drive_factor  *= 0.95

if WHEEL_TYPE == "tracked":
    rotate_factor *= 1.15
    drive_factor  *= 0.85

# -------------------------
# Log configuration
# -------------------------
print(
    "\n================ CONFIGURATION ================\n"
    f"Robot ID      : {ROBOT_ID}\n"
    f"Environment   : {ENVIRONMENT}\n"
    f"Surface       : {SURFACE}\n"
    f"Drive Layout  : {DRIVE_LAYOUT}\n"
    f"Wheel Type    : {WHEEL_TYPE}\n"
    f"Rotate factor : {rotate_factor:.3f}\n"
    f"Drive factor  : {drive_factor:.3f}\n"
    f"Distance scale: {distance_scale:.3f}\n"
    f"Motor polarity: {motor_polarity}\n"
    "==============================================\n"
)
