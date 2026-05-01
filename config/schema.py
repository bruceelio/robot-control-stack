# config/schema.py

from dataclasses import dataclass, asdict
from pprint import pprint

# --------------------------------------------------
# Validation tables (schema-level only)
# --------------------------------------------------

VALID_ENVIRONMENTS = ("simulation", "real")
VALID_SURFACES = ("simulation", "tile", "wood", "carpet")
VALID_WALL_ANGLE_BACKENDS = ("one_ultrasonic_scan", "two_ultrasonics")


# --------------------------------------------------
# Resolved config object (single source of truth)
# --------------------------------------------------

@dataclass(frozen=True)
class Config:
    # Identity / mode
    robot_id: str
    hardware_profile: str
    environment: str
    surface: str
    drive_layout: str
    wheel_type: str
    cameras: dict
    encoders: dict
    camera_mounts: dict
    gripper_mount: dict
    gripper_from_camera: dict

    # Arena
    arena_size: int

    # Strategy
    default_target_kind: str

    # Motion / robot
    motion_backend: str
    grab_distance_mm: float
    motor_polarity: list[int]
    rotation_sign: int

    # Calibration
    rotate_factor: float
    drive_factor: float

    # InitEscape
    init_escape_drive_mm: int
    init_escape_rotate_deg: float

    # PostPickupRealign
    post_pickup_reverse_mm: int
    post_pickup_rotate_deg: float

    # PostDropOffRealign
    post_dropoff_reverse_mm: int
    post_dropoff_rotate_deg: float

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
    camera_fresh_obs_max_age_s: float
    marker_height_max_distance_mm: float
    marker_pitch_high_deg: float
    marker_pitch_low_deg: float

    vision_loss_timeout_s: float
    vision_grace_period_s: float

    band_b_min_distance_mm: float
    final_commit_distance_mm: float
    final_approach_direct_range_mm: float
    final_approach_backup_mm: float
    height_decision_deadline_mm: float

    final_commit_distance_high_mm: float
    final_approach_direct_range_high_mm: float
    final_approach_max_degree_high: float
    visible_max_age_s: float
    final_approach_marker_push: float

    reacquire_target_vision_loss: float

    # BackoffScan
    backoff_scan_mm: float
    backoff_scan_cap_deg: float
    backoff_scan_step_deg: float
    backoff_scan_timeout_s: float


    # --------------------------------------------------
    # Wall / ultrasonic geometry (navigation)
    # --------------------------------------------------

    # Which wall-angle backend to use:
    #   "one_ultrasonic_scan"
    #   "two_ultrasonics"
    wall_angle_backend: str

    # ---- Two-ultrasonic configuration ----
    wall_two_ultrasonic_keys: tuple[str, str]
    wall_two_ultrasonic_baseline_mm: float

    # ---- One-ultrasonic scan configuration ----
    wall_one_ultrasonic_key: str
    wall_scan_angle_1_deg: float
    wall_scan_angle_2_deg: float
    wall_scan_samples_per_angle: int
    wall_scan_settle_time_s: float

    # ---- Ultrasonic sanity limits ----
    wall_ultrasonic_min_mm: float
    wall_ultrasonic_max_mm: float

    # ---- Wall angle filtering / stability ----
    wall_angle_stable_samples: int
    wall_angle_max_age_s: float

    # ---- Parallel-to-wall control ----
    wall_parallel_tolerance_deg: float
    wall_parallel_trigger_deg: float
    wall_parallel_max_rotate_deg: float
    wall_parallel_step_deg: float
    wall_parallel_timeout_s: float


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
    "hardware_profile": ("profile", "HARDWARE_PROFILE"),
    "environment": ("profile", "ENVIRONMENT"),
    "surface": ("profile", "SURFACE"),
    "drive_layout": ("profile", "DRIVE_LAYOUT"),
    "wheel_type": ("profile", "WHEEL_TYPE"),
    "cameras": ("profile", "CAMERAS"),
    "encoders": ("computed", "encoders"),
    "camera_mounts": ("profile", "CAMERA_MOUNTS"),
    "gripper_mount": ("profile", "GRIPPER_MOUNT"),
    "gripper_from_camera": ("computed", "gripper_from_camera"),


    # Arena
    "arena_size": ("arena", "ARENA_SIZE"),

    # Strategy
    "default_target_kind": ("strategy", "DEFAULT_TARGET_KIND"),

    # Motion / robot
    "motion_backend": ("profile", "MOTION_BACKEND"),
    "grab_distance_mm": ("profile", "GRAB_DISTANCE_MM"),
    "motor_polarity": ("profile", "MOTOR_POLARITY"),
    "rotation_sign": ("profile", "ROTATION_SIGN"),

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

    # PostDropoffRealign
    "post_dropoff_reverse_mm": ("profile", "POST_DROPOFF_REVERSE_MM"),
    "post_dropoff_rotate_deg": ("profile", "POST_DROPOFF_ROTATE_DEG"),


    # RecoverLocalisation
    "recover_step_deg": ("profile", "RECOVER_STEP_DEG"),
    "recover_max_sweep_deg": ("profile", "RECOVER_MAX_SWEEP_DEG"),
    "recover_settle_time": ("profile", "RECOVER_SETTLE_TIME"),

    # Vision / Seek & Collect
    "camera_settle_time": ("profile", "CAMERA_SETTLE_TIME"),
    "camera_fresh_obs_max_age_s": ("profile", "CAMERA_FRESH_OBS_MAX_AGE_S"),
    "marker_height_max_distance_mm": ("profile", "MARKER_HEIGHT_MAX_DISTANCE_MM"),
    "marker_pitch_high_deg": ("profile", "MARKER_PITCH_HIGH_DEG"),
    "marker_pitch_low_deg": ("profile", "MARKER_PITCH_LOW_DEG"),
    "height_decision_deadline_mm": ("profile", "HEIGHT_DECISION_DEADLINE_MM"),

    "vision_loss_timeout_s": ("profile", "VISION_LOSS_TIMEOUT_S"),
    "vision_grace_period_s": ("profile", "VISION_GRACE_PERIOD_S"),

    "band_b_min_distance_mm": ("profile", "BAND_B_MIN_DISTANCE_MM"),
    "final_commit_distance_mm": ("profile", "FINAL_COMMIT_DISTANCE_MM"),
    "final_approach_direct_range_mm": ("profile", "FINAL_APPROACH_DIRECT_RANGE_MM"),
    "final_approach_backup_mm": ("profile", "FINAL_APPROACH_BACKUP_MM"),

    "final_commit_distance_high_mm": ("profile", "FINAL_COMMIT_DISTANCE_HIGH_MM"),
    "final_approach_direct_range_high_mm": ("profile", "FINAL_APPROACH_DIRECT_RANGE_HIGH_MM"),
    "final_approach_max_degree_high": ("profile", "FINAL_APPROACH_MAX_DEGREE_HIGH"),
    "visible_max_age_s": ("profile", "VISIBLE_MAX_AGE_S"),
    "final_approach_marker_push": ("profile", "FINAL_APPROACH_MARKER_PUSH"),


    "reacquire_target_vision_loss": ("profile", "REACQUIRE_TARGET_VISION_LOSS"),

    # BackoffScan
    "backoff_scan_mm": ("profile", "BACKOFF_SCAN_MM"),
    "backoff_scan_cap_deg": ("profile", "BACKOFF_SCAN_CAP_DEG"),
    "backoff_scan_step_deg": ("profile", "BACKOFF_SCAN_STEP_DEG"),
    "backoff_scan_timeout_s": ("profile", "BACKOFF_SCAN_TIMEOUT_S"),


    # --------------------------------------------------
    # Wall / ultrasonic geometry (navigation)
    # --------------------------------------------------
    "wall_angle_backend": ("profile", "WALL_ANGLE_BACKEND"),

    # Two ultrasonics
    "wall_two_ultrasonic_keys": ("profile", "WALL_TWO_ULTRASONIC_KEYS"),
    "wall_two_ultrasonic_baseline_mm": ("profile", "WALL_TWO_ULTRASONIC_BASELINE_MM"),

    # One ultrasonic scan
    "wall_one_ultrasonic_key": ("profile", "WALL_ONE_ULTRASONIC_KEY"),
    "wall_scan_angle_1_deg": ("profile", "WALL_SCAN_ANGLE_1_DEG"),
    "wall_scan_angle_2_deg": ("profile", "WALL_SCAN_ANGLE_2_DEG"),
    "wall_scan_samples_per_angle": ("profile", "WALL_SCAN_SAMPLES_PER_ANGLE"),
    "wall_scan_settle_time_s": ("profile", "WALL_SCAN_SETTLE_TIME_S"),

    # Ultrasonic sanity
    "wall_ultrasonic_min_mm": ("profile", "WALL_ULTRASONIC_MIN_MM"),
    "wall_ultrasonic_max_mm": ("profile", "WALL_ULTRASONIC_MAX_MM"),

    # Filtering / stability
    "wall_angle_stable_samples": ("profile", "WALL_ANGLE_STABLE_SAMPLES"),
    "wall_angle_max_age_s": ("profile", "WALL_ANGLE_MAX_AGE_S"),

    # Parallel-to-wall control
    "wall_parallel_tolerance_deg": ("profile", "WALL_PARALLEL_TOLERANCE_DEG"),
    "wall_parallel_trigger_deg": ("profile", "WALL_PARALLEL_TRIGGER_DEG"),
    "wall_parallel_max_rotate_deg": ("profile", "WALL_PARALLEL_MAX_ROTATE_DEG"),
    "wall_parallel_step_deg": ("profile", "WALL_PARALLEL_STEP_DEG"),
    "wall_parallel_timeout_s": ("profile", "WALL_PARALLEL_TIMEOUT_S"),
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

    if profile.WALL_ANGLE_BACKEND not in VALID_WALL_ANGLE_BACKENDS:
        raise ValueError(f"Invalid WALL_ANGLE_BACKEND: {profile.WALL_ANGLE_BACKEND}")


    # --- derived calibration ---
    rotate_factor = (
        profile.BASE_ROTATE_FACTOR
        * profile.SURFACE_MULTIPLIERS[profile.SURFACE]["rotate"]
    )

    drive_factor = (
        profile.BASE_DRIVE_FACTOR
        * profile.SURFACE_MULTIPLIERS[profile.SURFACE]["drive"]
    )

    camera_mounts = getattr(profile, "CAMERA_MOUNTS")
    gripper_mount = getattr(profile, "GRIPPER_MOUNT")

    front_cam = camera_mounts["front"]

    computed = {
        "rotate_factor": rotate_factor,
        "drive_factor": drive_factor,
        "encoders": getattr(profile, "ENCODERS", {}),
        "gripper_from_camera": {
            "x_mm": gripper_mount["x_mm"] - front_cam["x_mm"],
            "y_mm": gripper_mount["y_mm"] - front_cam["y_mm"],
        },
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
