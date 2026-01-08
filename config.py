# config.py

# =========================
# SELECTIONS
# =========================
SURFACE = "simulation"   # simulation, tile, wood, carpet
ENVIRONMENT = "simulation"   # simulation, real
DRIVE_LAYOUT  = "2WD"           # 2WD, 3WD, 4WD
WHEEL_TYPE    = "standard"      # standard, mecanum, omni, tracked
ROBOT_ID = "sim"   # sim, sr1, sr2, practice

# =========================
# Base calibration factors
# =========================
default_power = 0.5

# Delays / times (seconds)
drive_delay = 0.1        # delay time until acceleration at 1.0 power
drive_time_1000 = 0.6        # time in seconds to drive 1000 mm at 1.0 power
rotate_delay = 0.06       # delay time until acceleration starts at 0.51 power
rotate_time_90 = 0.44        # time in seconds to rotate 90 deg at 0.51 power

# Base factors
BASE_ROTATE_FACTOR = 1.0
BASE_DRIVE_FACTOR = 1.0
BASE_DISTANCE_SCALE = 1.0

# =========================
# Surface Multipliers
# =========================
SURFACE_MULTIPLIERS = {
    "simulation": {"rotate": 1.00, "drive": 1.00},
    "tile":       {"rotate": 0.92, "drive": 0.95},
    "wood":       {"rotate": 0.88, "drive": 0.90},
    "carpet":     {"rotate": 1.15, "drive": 1.10},
}

# =========================
# Environment Distance Scales
# =========================
DISTANCE_SCALES = {
    "simulation": 1.327,   # known correct for Webots
    "real":       1.00,    # starting point, tune later
}

# =========================
# MOTOR POLARITY
# +1 = normal direction
# -1 = reversed motor
# =========================

ROBOT_CONFIGS = {
    "sim": {
        "motor_polarity": {
            "2WD": [ 1,  1],
            "3WD": [ 1,  1,  1],
            "4WD": [ 1,  1,  1,  1],
        },
    },
    "sr1": {
        "motor_polarity": {
            "2WD": [ 1, -1],
            "3WD": [ 1, -1,  1],
            "4WD": [ 1, -1,  1, -1],
        },
    },
    "sr2": {
        "motor_polarity": {
            "2WD": [-1,  1],
            "3WD": [-1,  1, -1],
            "4WD": [-1,  1, -1,  1],
        },
    },
}



# =========================
# Safety Checks
# =========================
if ENVIRONMENT not in DISTANCE_SCALES:
    raise ValueError(f"Unknown ENVIRONMENT '{ENVIRONMENT}'")

if SURFACE not in SURFACE_MULTIPLIERS:
    raise ValueError(f"Unknown SURFACE '{SURFACE}'")

if ROBOT_ID not in ROBOT_CONFIGS:
    raise ValueError(f"Unknown ROBOT_ID '{ROBOT_ID}'")

if DRIVE_LAYOUT not in ROBOT_CONFIGS[ROBOT_ID]["motor_polarity"]:
    raise ValueError(
        f"No motor polarity for {ROBOT_ID} + {DRIVE_LAYOUT}"
    )



# =========================
# Apply calibration
# =========================
rotate_factor = BASE_ROTATE_FACTOR * SURFACE_MULTIPLIERS[SURFACE]["rotate"]
drive_factor  = BASE_DRIVE_FACTOR  * SURFACE_MULTIPLIERS[SURFACE]["drive"]
distance_scale = BASE_DISTANCE_SCALE * DISTANCE_SCALES[ENVIRONMENT]
motor_polarity = ROBOT_CONFIGS[ROBOT_ID]["motor_polarity"][DRIVE_LAYOUT]


# Optional: adjust drive/rotate factors depending on DRIVE_LAYOUT / WHEEL_TYPE
# Example: mecanum wheels often need slightly higher rotate factor due to slippage
if WHEEL_TYPE in ("mecanum", "omni"):
    rotate_factor *= 1.05   # tweak as needed
    drive_factor  *= 0.95

if WHEEL_TYPE == "tracked":
    rotate_factor *= 1.15
    drive_factor *= 0.85
# =========================
# Log loaded configuration
# =========================
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



