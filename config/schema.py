# config/schema.py

from dataclasses import dataclass, asdict
import math
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
    drive_motor_profile: str
    surface: str
    drive_layout: str
    wheel_type: str
    drive_track_width_mm: float
    gripper_mount: dict
    gripper_from_camera: dict
    gripper_open_position: float
    gripper_grab_position: float
    lift_carry_position: float

    # Pickup lift positions
    lift_low_pickup_position: float
    lift_low_retreat_position: float
    lift_high_pickup_position: float
    lift_high_retreat_position: float

    # Grip verification
    pickup_lift_settle_s: float
    pickup_grip_verify_enabled: bool
    pickup_grip_verify_min_mm: float | None
    pickup_grip_verify_max_mm: float | None
    pickup_grip_verify_samples: int
    pickup_grip_verify_min_valid_samples: int
    pickup_grip_verify_max_spread_mm: float
    pickup_grip_verify_sample_delay_s: float

    io: dict[str, str | None]

    cameras: dict
    async_vision_enabled: bool
    vision_sources: dict
    encoders: dict
    encoder_sign: dict
    encoder_wheel_diameter_mm: dict
    camera_mounts: dict
    range_sensor_mounts: dict


    # Servoing
    servoing_enabled: bool
    servoing_linear_max_mm_s: float
    servoing_angular_max_rad_s: float

    # Arena
    arena_size: int

    # Strategy
    default_target_kind: str | None
    default_target_elevation: str | None
    match_zone_source: str
    match_zone_fixed: int
    usb_match_zone_file: str

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

    # Approach Target servo
    approach_target_servo_linear_kp: float
    approach_target_servo_linear_ki: float
    approach_target_servo_linear_kd: float

    approach_target_servo_angular_kp: float
    approach_target_servo_angular_ki: float
    approach_target_servo_angular_kd: float

    approach_target_servo_derivative_mode: str


    approach_target_servo_drive_cutoff_deg: float
    approach_target_servo_stop_tolerance_mm: float

    approach_range_linear_max_mps: float
    approach_range_angular_max_rad_s: float

    approach_pose_pbvs_linear_max_mps: float
    approach_pose_smooth_linear_max_mps: float
    approach_pose_angular_max_rad_s: float

    # PostPickupRealign
    post_pickup_reverse_mm: int
    post_pickup_rotate_deg: float

    # Return To Base servo
    return_to_base_servo_linear_kp: float
    return_to_base_servo_linear_ki: float
    return_to_base_servo_linear_kd: float

    return_to_base_servo_angular_kp: float
    return_to_base_servo_angular_ki: float
    return_to_base_servo_angular_kd: float

    return_to_base_servo_derivative_mode: str

    return_to_base_servo_linear_max_mm_s: float
    return_to_base_servo_angular_max_rad_s: float
    return_to_base_servo_drive_slowdown_start_deg: float
    return_to_base_servo_drive_cutoff_deg: float
    return_to_base_servo_stop_tolerance_mm: float


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
    motor_power_max: float
    motor_power_min: float
    battery_voltage_nominal: float

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


    def has_io(self, category: str, name: str) -> bool:
        key = f"{category}.{name}"

        if key not in self.io:
            raise KeyError(f"Unknown IO capability: {key}")

        return self.io[key] is not None

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
    "drive_motor_profile": ("profile", "DRIVE_MOTOR_PROFILE"),
    "surface": ("profile", "SURFACE"),
    "drive_layout": ("profile", "DRIVE_LAYOUT"),
    "wheel_type": ("profile", "WHEEL_TYPE"),
    "drive_track_width_mm": ("profile", "DRIVE_TRACK_WIDTH_MM"),

    "io": ("profile", "IO"),

    "cameras": ("profile", "CAMERAS"),
    "vision_sources": ("profile", "VISION_SOURCES"),
    "async_vision_enabled": ("profile", "ASYNC_VISION_ENABLED"),
    "encoders": ("computed", "encoders"),
    "encoder_sign": ("profile", "ENCODER_SIGN"),
    "encoder_wheel_diameter_mm": ("profile", "ENCODER_WHEEL_DIAMETER_MM"),
    "camera_mounts": ("computed", "camera_mounts"),
    "range_sensor_mounts": ("computed", "range_sensor_mounts"),
    "gripper_mount": ("computed", "gripper_mount"),
    "gripper_from_camera": ("computed", "gripper_from_camera"),
    "gripper_open_position": ("profile", "GRIPPER_OPEN_POSITION"),
    "gripper_grab_position": ("profile", "GRIPPER_GRAB_POSITION"),
    "lift_carry_position": ("profile", "LIFT_CARRY_POSITION"),

    # Pickup lift positions
    "lift_low_pickup_position": ("profile", "LIFT_LOW_PICKUP_POSITION"),
    "lift_low_retreat_position": ("profile", "LIFT_LOW_RETREAT_POSITION"),
    "lift_high_pickup_position": ("profile", "LIFT_HIGH_PICKUP_POSITION"),
    "lift_high_retreat_position": ("profile", "LIFT_HIGH_RETREAT_POSITION"),

    # Grip verification
    "pickup_lift_settle_s": ("profile", "PICKUP_LIFT_SETTLE_S"),
    "pickup_grip_verify_enabled": ("profile", "PICKUP_GRIP_VERIFY_ENABLED"),
    "pickup_grip_verify_min_mm": ("profile", "PICKUP_GRIP_VERIFY_MIN_MM"),
    "pickup_grip_verify_max_mm": ("profile", "PICKUP_GRIP_VERIFY_MAX_MM"),
    "pickup_grip_verify_samples": ("profile", "PICKUP_GRIP_VERIFY_SAMPLES"),
    "pickup_grip_verify_min_valid_samples": (
        "profile", "PICKUP_GRIP_VERIFY_MIN_VALID_SAMPLES"
    ),
    "pickup_grip_verify_max_spread_mm": (
        "profile", "PICKUP_GRIP_VERIFY_MAX_SPREAD_MM"
    ),
    "pickup_grip_verify_sample_delay_s": (
        "profile", "PICKUP_GRIP_VERIFY_SAMPLE_DELAY_S"
    ),

    # Servoing
    "servoing_enabled": ("profile", "SERVOING_ENABLED"),
    "servoing_linear_max_mm_s": ("profile", "SERVOING_LINEAR_MAX_MM_S"),
    "servoing_angular_max_rad_s": ("profile", "SERVOING_ANGULAR_MAX_RAD_S"),

    # Arena
    "arena_size": ("arena", "ARENA_SIZE"),

    # Strategy
    "default_target_kind": ("strategy", "DEFAULT_TARGET_KIND"),
    "default_target_elevation": ("strategy", "DEFAULT_TARGET_ELEVATION"),
    "match_zone_source": ("strategy", "MATCH_ZONE_SOURCE"),
    "match_zone_fixed": ("strategy", "MATCH_ZONE_FIXED"),
    "usb_match_zone_file": ("strategy", "USB_MATCH_ZONE_FILE"),

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
    "motor_power_max": ("profile", "MOTOR_POWER_MAX"),
    "motor_power_min": ("profile", "MOTOR_POWER_MIN"),
    "battery_voltage_nominal": ("profile", "BATTERY_VOLTAGE_NOMINAL"),

    # InitEscape
    "init_escape_drive_mm": ("profile", "INIT_ESCAPE_DRIVE_MM"),
    "init_escape_rotate_deg": ("profile", "INIT_ESCAPE_ROTATE_DEG"),

    # Approach Target servo
    "approach_target_servo_linear_kp":
        ("profile", "APPROACH_TARGET_SERVO_LINEAR_KP"),
    "approach_target_servo_linear_ki":
        ("profile", "APPROACH_TARGET_SERVO_LINEAR_KI"),
    "approach_target_servo_linear_kd":
        ("profile", "APPROACH_TARGET_SERVO_LINEAR_KD"),

    "approach_target_servo_angular_kp":
        ("profile", "APPROACH_TARGET_SERVO_ANGULAR_KP"),
    "approach_target_servo_angular_ki":
        ("profile", "APPROACH_TARGET_SERVO_ANGULAR_KI"),
    "approach_target_servo_angular_kd":
        ("profile", "APPROACH_TARGET_SERVO_ANGULAR_KD"),

    "approach_target_servo_derivative_mode":
        ("profile", "APPROACH_TARGET_SERVO_DERIVATIVE_MODE"),


    "approach_target_servo_drive_cutoff_deg":
        ("profile", "APPROACH_TARGET_SERVO_DRIVE_CUTOFF_DEG"),
    "approach_target_servo_stop_tolerance_mm":
        ("profile", "APPROACH_TARGET_SERVO_STOP_TOLERANCE_MM"),

    "approach_range_linear_max_mps":
        ("profile", "APPROACH_RANGE_LINEAR_MAX_MPS"),
    "approach_range_angular_max_rad_s":
     ("profile", "APPROACH_RANGE_ANGULAR_MAX_RAD_S"),

    "approach_pose_pbvs_linear_max_mps":
        ("profile", "APPROACH_POSE_PBVS_LINEAR_MAX_MPS"),
    "approach_pose_smooth_linear_max_mps":
        ("profile", "APPROACH_POSE_SMOOTH_LINEAR_MAX_MPS"),
    "approach_pose_angular_max_rad_s":
        ("profile", "APPROACH_POSE_ANGULAR_MAX_RAD_S"),

    # PostPickupRealign
    "post_pickup_reverse_mm": ("profile", "POST_PICKUP_REVERSE_MM"),
    "post_pickup_rotate_deg": ("profile", "POST_PICKUP_ROTATE_DEG"),

# Return To Base servo
"return_to_base_servo_linear_kp":
    ("profile", "RETURN_TO_BASE_SERVO_LINEAR_KP"),
"return_to_base_servo_linear_ki":
    ("profile", "RETURN_TO_BASE_SERVO_LINEAR_KI"),
"return_to_base_servo_linear_kd":
    ("profile", "RETURN_TO_BASE_SERVO_LINEAR_KD"),

"return_to_base_servo_angular_kp":
    ("profile", "RETURN_TO_BASE_SERVO_ANGULAR_KP"),
"return_to_base_servo_angular_ki":
    ("profile", "RETURN_TO_BASE_SERVO_ANGULAR_KI"),
"return_to_base_servo_angular_kd":
    ("profile", "RETURN_TO_BASE_SERVO_ANGULAR_KD"),

"return_to_base_servo_derivative_mode":
    ("profile", "RETURN_TO_BASE_SERVO_DERIVATIVE_MODE"),

"return_to_base_servo_linear_max_mm_s":
    ("profile", "RETURN_TO_BASE_SERVO_LINEAR_MAX_MM_S"),
"return_to_base_servo_angular_max_rad_s":
    ("profile", "RETURN_TO_BASE_SERVO_ANGULAR_MAX_RAD_S"),
"return_to_base_servo_drive_slowdown_start_deg":
    (
        "profile",
        "RETURN_TO_BASE_SERVO_DRIVE_SLOWDOWN_START_DEG",
    ),
"return_to_base_servo_drive_cutoff_deg":
    ("profile", "RETURN_TO_BASE_SERVO_DRIVE_CUTOFF_DEG"),
"return_to_base_servo_stop_tolerance_mm":
    ("profile", "RETURN_TO_BASE_SERVO_STOP_TOLERANCE_MM"),


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
}

def _mount_to_si(
    mount: dict,
) -> dict:
    """
    Normalize one source-config mount into canonical runtime units.

    Source configuration:
        position = millimetres
        angles   = degrees

    Runtime configuration:
        position = metres
        angles   = radians

    Axis conventions are not changed here.
    """

    return {
        "x_m": (
            float(mount["x_mm"])
            / 1000.0
        ),
        "y_m": (
            float(mount["y_mm"])
            / 1000.0
        ),
        "z_m": (
            float(
                mount.get(
                    "z_mm",
                    0.0,
                )
            )
            / 1000.0
        ),
        "roll_rad": math.radians(
            float(
                mount.get(
                    "roll_deg",
                    0.0,
                )
            )
        ),
        "pitch_rad": math.radians(
            float(
                mount.get(
                    "pitch_deg",
                    0.0,
                )
            )
        ),
        "yaw_rad": math.radians(
            float(
                mount.get(
                    "yaw_deg",
                    0.0,
                )
            )
        ),
    }


def _mounts_to_si(
    mounts: dict,
) -> dict:
    """
    Normalize a named collection of source-config mounts.
    """

    return {
        str(name): _mount_to_si(mount)
        for name, mount in mounts.items()
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

    source_camera_mounts = getattr(
        profile,
        "CAMERA_MOUNTS",
    )

    source_gripper_mount = getattr(
        profile,
        "GRIPPER_MOUNT",
    )

    camera_mounts = _mounts_to_si(
        source_camera_mounts
    )

    gripper_mount = _mount_to_si(
        source_gripper_mount
    )

    # Retained temporarily for the legacy
    # gripper_from_camera mm interface.
    front_cam = source_camera_mounts[
        "front"
    ]

    computed = {
        "rotate_factor": rotate_factor,
        "drive_factor": drive_factor,
        "encoders": getattr(profile, "ENCODERS", {}),
        "camera_mounts": camera_mounts,
        "range_sensor_mounts": getattr(
            profile,
            "RANGE_SENSOR_MOUNTS",
            {},
        ),
        "gripper_mount": gripper_mount,
        "gripper_from_camera": {
            "x_mm": (
                source_gripper_mount["x_mm"]
                - front_cam["x_mm"]
            ),
            "y_mm": (
                source_gripper_mount["y_mm"]
                - front_cam["y_mm"]
            ),
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
