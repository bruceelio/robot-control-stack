# config/schema.py

from dataclasses import dataclass, asdict
from pprint import pprint

# --------------------------------------------------
# Validation tables (schema-level only)
# --------------------------------------------------

VALID_ENVIRONMENTS = ("simulation", "real")
VALID_SURFACES = ("simulation", "tile", "wood", "carpet")

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
    max_motor_power: float

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
# Declarative resolve map
# --------------------------------------------------

RESOLVE_MAP = {
    # Identity
    "robot_id": ("profile", "ROBOT_ID"),
    "environment": ("profile", "ENVIRONMENT"),
    "surface": ("profile", "SURFACE"),
    "drive_layout": ("profile", "DRIVE_LAYOUT"),
    "wheel_type": ("profile", "WHEEL_TYPE"),

    # Arena
    "arena_size": ("arena", "ARENA_SIZE"),

    # Strategy
    "default_target_kind": ("strategy", "DEFAULT_TARGET_KIND"),

    # Motion / robot
    "motion_backend": ("profile", "MOTION_BACKEND"),
    "grab_distance_mm": ("profile", "GRAB_DISTANCE_MM"),
    "motor_polarity": ("profile", "MOTOR_POLARITY"),

    # Calibration (computed)
    "rotate_factor": ("computed", "rotate_factor"),
    "drive_factor": ("computed", "drive_factor"),

    # Motion limits
    "min_rotate_deg": ("profile", "MIN_ROTATE_DEG"),
    "min_drive_mm": ("profile", "MIN_DRIVE_MM"),
    "max_rotate_deg": ("profile", "MAX_ROTATE_DEG"),
    "max_drive_mm": ("profile", "MAX_DRIVE_MM"),
    "max_motor_power": ("profile", "MAX_MOTOR_POWER"),

    # InitEscape
    "init_escape_drive_mm": ("profile", "INIT_ESCAPE_DRIVE_MM"),
    "init_escape_rotate_deg": ("profile", "INIT_ESCAPE_ROTATE_DEG"),

    # PostPickupRealign
    "post_pickup_reverse_mm": ("profile", "POST_PICKUP_REVERSE_MM"),
    "post_pickup_rotate_deg": ("profile", "POST_PICKUP_ROTATE_DEG"),

    # RecoverLocalisation
    "recover_step_deg": ("profile", "RECOVER_STEP_DEG"),
    "recover_max_sweep_deg": ("profile", "RECOVER_MAX_SWEEP_DEG"),
    "recover_settle_time": ("profile", "RECOVER_SETTLE_TIME"),

    # Vision
    "camera_settle_time": ("profile", "CAMERA_SETTLE_TIME"),
    "marker_height_max_distance_mm": ("profile", "MARKER_HEIGHT_MAX_DISTANCE_MM"),
    "marker_pitch_high_deg": ("profile", "MARKER_PITCH_HIGH_DEG"),
    "marker_pitch_low_deg": ("profile", "MARKER_PITCH_LOW_DEG"),
    "final_commit_distance_mm": ("profile", "FINAL_COMMIT_DISTANCE_MM"),
    "vision_loss_timeout_s": ("profile", "VISION_LOSS_TIMEOUT_S"),
}

# --------------------------------------------------
# Resolver
# --------------------------------------------------

def resolve(*, arena, profile, strategy) -> Config:
    # --- validation ---
    if profile.ENVIRONMENT not in VALID_ENVIRONMENTS:
        raise ValueError(f"Invalid ENVIRONMENT: {profile.ENVIRONMENT}")

    if profile.SURFACE not in VALID_SURFACES:
        raise ValueError(f"Invalid SURFACE: {profile.SURFACE}")

    # --- derived calibration ---
    rotate_factor = (
        profile.BASE_ROTATE_FACTOR
        * profile.SURFACE_MULTIPLIERS[profile.SURFACE]["rotate"]
    )

    drive_factor = (
        profile.BASE_DRIVE_FACTOR
        * profile.SURFACE_MULTIPLIERS[profile.SURFACE]["drive"]
    )


    computed = {
        "rotate_factor": rotate_factor,
        "drive_factor": drive_factor,
            }

    values = {}

    for field, (source, name) in RESOLVE_MAP.items():
        try:
            if source == "profile":
                values[field] = getattr(profile, name)
            elif source == "arena":
                values[field] = getattr(arena, name)
            elif source == "strategy":
                values[field] = getattr(strategy, name)
            elif source == "computed":
                values[field] = computed[name]
            else:
                raise RuntimeError(f"Unknown source '{source}'")
        except AttributeError as e:
            raise RuntimeError(
                f"Config resolve failed: missing {source}.{name} "
                f"(needed for field '{field}')"
            ) from e

    return Config(**values)
