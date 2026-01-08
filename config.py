# config.py

# Selections

SURFACE = "wood"   # simulation, tile, wood, carpet
ENVIRONMENT = "simulation"   # simulation, real

# Calibration factors
drive_delay = 0.1        # delay time until acceleration at 1.0 power
drive_time_1000 = 0.6        # time in seconds to drive 1000 mm at 1.0 power

rotate_delay = 0.06       # delay time until acceleration starts at 0.51 power
rotate_time_90 = 0.44        # time in seconds to rotate 90 deg at 0.51 power

default_power = 0.5

# =========================
# Base Calibration (do NOT touch per surface)
# =========================
BASE_ROTATE_FACTOR = 1.0
BASE_DRIVE_FACTOR = 1.0

# =========================
# Surface Multipliers
# (rotate first, then drive)
# =========================
SURFACE_MULTIPLIERS = {
    "simulation": {
        "rotate": 1.00,
        "drive": 1.00,
    },
    "tile": {
        "rotate": 0.92,
        "drive": 0.95,
    },
    "wood": {
        "rotate": 0.88,
        "drive": 0.90,
    },
    "carpet": {
        "rotate": 1.15,
        "drive": 1.10,
    },
}

# =========================
# Safety Check
# =========================
if SURFACE not in SURFACE_MULTIPLIERS:
    raise ValueError(
        f"Unknown surface '{SURFACE}'. "
        f"Choose from: {list(SURFACE_MULTIPLIERS.keys())}"
    )

# =========================
# Apply Surface Calibration
# =========================
rotate_factor = BASE_ROTATE_FACTOR * SURFACE_MULTIPLIERS[SURFACE]["rotate"]
drive_factor  = BASE_DRIVE_FACTOR  * SURFACE_MULTIPLIERS[SURFACE]["drive"]

# =========================
# Base Distance Calibration
# =========================
BASE_DISTANCE_SCALE = 1.0


# =========================
# Environment Distance Scales
# =========================
DISTANCE_SCALES = {
    "simulation": 1.327,   # known correct for Webots
    "real":       1.00,    # starting point, tune later
}


# =========================
# Safety Check
# =========================
if ENVIRONMENT not in DISTANCE_SCALES:
    raise ValueError(
        f"Unknown environment '{ENVIRONMENT}'. "
        f"Choose from: {list(DISTANCE_SCALES.keys())}"
    )


# =========================
# Apply Distance Calibration
# =========================
distance_scale = BASE_DISTANCE_SCALE * DISTANCE_SCALES[ENVIRONMENT]


# =========================
# Log Confirmations
# =========================
print(
    "\n================ CONFIGURATION ================\n"
    f"Surface       : {SURFACE}\n"
    f"Environment   : {ENVIRONMENT}\n"
    f"Rotate factor : {rotate_factor:.3f}\n"
    f"Drive factor  : {drive_factor:.3f}\n"
    f"Distance scale: {distance_scale:.3f}\n"
    "==============================================\n"
)


