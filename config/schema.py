# config/schema.py

from dataclasses import dataclass, asdict
from pprint import pprint

# --------------------------------------------------
# Validation tables
# --------------------------------------------------

VALID_ENVIRONMENTS = ("simulation", "real")
VALID_SURFACES = ("simulation", "tile", "wood", "carpet")

SURFACE_MULTIPLIERS = {
    "simulation": {"rotate": 1.00, "drive": 1.00},
    "tile":       {"rotate": 0.92, "drive": 0.95},
    "wood":       {"rotate": 0.88, "drive": 0.90},
    "carpet":     {"rotate": 1.15, "drive": 1.10},
}

DISTANCE_SCALES = {
    "simulation": 1.327,
    "real":       1.00,
}

# --------------------------------------------------
# Resolved config object (single source of truth)
# --------------------------------------------------

@dataclass(frozen=True)
class Config:
    # Identity / mode
    robot_id: str
    environment: str
    surface: str
    drive_layout: str
    wheel_type: str

    # Arena
    arena_size: int

    # Strategy
    default_target_kind: str

    # Motion / robot
    motion_backend: str
    grab_distance_mm: float
    motor_polarity: list[int]

    # Calibration
    rotate_factor: float
    drive_factor: float
    distance_scale: float

    # InitEscape
    init_escape_drive_mm: int
    init_escape_rotate_deg: float

    # PostPickupRealign
    post_pickup_reverse_mm: int
    post_pickup_rotate_deg: float

    # RecoverLocalisation
    recover_step_deg: float
    recover_max_sweep_deg: float
    recover_settle_time: float

    # Motion limits
    min_rotate_deg: float
    min_drive_mm: float
    max_drive_mm: float
    max_rotate_deg: float

    # Vision / Seek & Collect
    camera_settle_time: float
    marker_height_max_distance_mm: float
    marker_pitch_high_deg: float
    marker_pitch_low_deg: float

    final_commit_distance_mm: float
    vision_loss_timeout_s: float

    def dump(self):
        print("\n=== RESOLVED CONFIGURATION ===")
        pprint(asdict(self), sort_dicts=False)
        print("=== END CONFIGURATION ===\n")

# --------------------------------------------------
# Resolver
# --------------------------------------------------

def resolve(*, arena, profile, strategy) -> Config:
    """
    Explicit configuration resolver.

    arena    : fixed physical constants (arena.py)
    profile  : robot + environment + calibration (profiles/*.py)
    strategy : run intent & competition choices (profiles/strategy.py)
    """

    # -------------------------
    # Arena validation
    # -------------------------
    if not hasattr(arena, "ARENA_SIZE"):
        raise ValueError("Arena missing ARENA_SIZE")

    # -------------------------
    # Profile validation
    # -------------------------
    REQUIRED_PROFILE_FIELDS = (
        "ROBOT_ID",
        "ENVIRONMENT",
        "SURFACE",
        "DRIVE_LAYOUT",
        "WHEEL_TYPE",
        "MOTION_BACKEND",
        "MOTOR_POLARITY",
        "GRAB_DISTANCE_MM",
        "BASE_ROTATE_FACTOR",
        "BASE_DRIVE_FACTOR",
        "BASE_DISTANCE_SCALE",
        "INIT_ESCAPE_DRIVE_MM",
        "INIT_ESCAPE_ROTATE_DEG",
        "POST_PICKUP_REVERSE_MM",
        "POST_PICKUP_ROTATE_DEG",
        "RECOVER_STEP_DEG",
        "RECOVER_MAX_SWEEP_DEG",
        "RECOVER_SETTLE_TIME",
        "MIN_ROTATE_DEG",
        "MIN_DRIVE_MM",
        "MAX_DRIVE_MM",
        "MAX_ROTATE_DEG",
        "CAMERA_SETTLE_TIME",
        "MARKER_HEIGHT_MAX_DISTANCE_MM",
        "MARKER_PITCH_HIGH_DEG",
        "MARKER_PITCH_LOW_DEG",
        "FINAL_COMMIT_DISTANCE_MM",
        "VISION_LOSS_TIMEOUT_S",
    )

    for name in REQUIRED_PROFILE_FIELDS:
        if not hasattr(profile, name):
            raise ValueError(f"Profile missing required field: {name}")

    if profile.ENVIRONMENT not in VALID_ENVIRONMENTS:
        raise ValueError(f"Unknown ENVIRONMENT '{profile.ENVIRONMENT}'")

    if profile.SURFACE not in VALID_SURFACES:
        raise ValueError(f"Unknown SURFACE '{profile.SURFACE}'")

    # -------------------------
    # Strategy validation
    # -------------------------
    if not hasattr(strategy, "DEFAULT_TARGET_KIND"):
        raise ValueError("Strategy missing DEFAULT_TARGET_KIND")

    if strategy.DEFAULT_TARGET_KIND not in ("acidic", "basic"):
        raise ValueError(
            f"DEFAULT_TARGET_KIND must be 'acidic' or 'basic', "
            f"got '{strategy.DEFAULT_TARGET_KIND}'"
        )

    # -------------------------
    # Calibration resolution
    # -------------------------
    rotate_factor = (
        profile.BASE_ROTATE_FACTOR
        * SURFACE_MULTIPLIERS[profile.SURFACE]["rotate"]
    )

    drive_factor = (
        profile.BASE_DRIVE_FACTOR
        * SURFACE_MULTIPLIERS[profile.SURFACE]["drive"]
    )

    distance_scale = (
        profile.BASE_DISTANCE_SCALE
        * DISTANCE_SCALES[profile.ENVIRONMENT]
    )

    # Wheel-type adjustments
    if profile.WHEEL_TYPE in ("mecanum", "omni"):
        rotate_factor *= 1.05
        drive_factor  *= 0.95

    if profile.WHEEL_TYPE == "tracked":
        rotate_factor *= 1.15
        drive_factor  *= 0.85

    # -------------------------
    # Freeze resolved config
    # -------------------------
    return Config(
        # Identity
        robot_id=profile.ROBOT_ID,
        environment=profile.ENVIRONMENT,
        surface=profile.SURFACE,
        drive_layout=profile.DRIVE_LAYOUT,
        wheel_type=profile.WHEEL_TYPE,

        # Arena
        arena_size=arena.ARENA_SIZE,

        # Strategy
        default_target_kind=strategy.DEFAULT_TARGET_KIND,

        # Motion
        motion_backend=profile.MOTION_BACKEND,
        grab_distance_mm=profile.GRAB_DISTANCE_MM,
        motor_polarity=profile.MOTOR_POLARITY,

        # Calibration
        rotate_factor=rotate_factor,
        drive_factor=drive_factor,
        distance_scale=distance_scale,

        # InitEscape
        init_escape_drive_mm=profile.INIT_ESCAPE_DRIVE_MM,
        init_escape_rotate_deg=profile.INIT_ESCAPE_ROTATE_DEG,

        # PostPickupRealign
        post_pickup_reverse_mm=profile.POST_PICKUP_REVERSE_MM,
        post_pickup_rotate_deg=profile.POST_PICKUP_ROTATE_DEG,

        # RecoverLocalisation
        recover_step_deg=profile.RECOVER_STEP_DEG,
        recover_max_sweep_deg=profile.RECOVER_MAX_SWEEP_DEG,
        recover_settle_time=profile.RECOVER_SETTLE_TIME,

        # Motion limits
        min_rotate_deg=profile.MIN_ROTATE_DEG,
        min_drive_mm=profile.MIN_DRIVE_MM,
        max_drive_mm=profile.MAX_DRIVE_MM,
        max_rotate_deg=profile.MAX_ROTATE_DEG,

        # Vision / Seek & Collect
        camera_settle_time=profile.CAMERA_SETTLE_TIME,
        marker_height_max_distance_mm=profile.MARKER_HEIGHT_MAX_DISTANCE_MM,
        marker_pitch_high_deg=profile.MARKER_PITCH_HIGH_DEG,
        marker_pitch_low_deg=profile.MARKER_PITCH_LOW_DEG,

        final_commit_distance_mm=profile.FINAL_COMMIT_DISTANCE_MM,
        vision_loss_timeout_s=profile.VISION_LOSS_TIMEOUT_S,
    )
