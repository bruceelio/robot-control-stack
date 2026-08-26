# calibration/resolve.py

from importlib import import_module
from typing import Dict

from calibration.schema import (
    Calibration,
    CameraCalibration,
    CameraMount,
    CameraOptical,
    CameraMeta,
)

# --------------------------------------------------
# Robot → calibration profile mapping
# --------------------------------------------------

PROFILE_MAP = {
    "sim": "simulation",
    "simulation": "simulation",
    "sr1": "sr1",
    "bob_bot": "bob_bot",
    "rob_bot": "rob_bot",
}


# --------------------------------------------------
# Resolver
# --------------------------------------------------

def resolve(*, config) -> Calibration:
    """
    Resolve the physical calibration profile for the selected robot.

    This loads immutable, real-world calibration data:
    - Drive timing
    - Rotation timing
    - Camera mounting & optical corrections
    """

    try:
        profile_name = PROFILE_MAP[config.robot_id]
    except KeyError:
        raise RuntimeError(
            f"No calibration profile for robot_id='{config.robot_id}'. "
            f"Known robots: {sorted(PROFILE_MAP.keys())}"
        )

    module_path = f"calibration.profiles.{profile_name}"
    profile = import_module(module_path)

    motor_module_path = f"calibration.motors.{config.drive_motor_profile}"
    motor = import_module(motor_module_path)

    # --------------------------------------------------
    # Resolve cameras (structured, optional)
    # --------------------------------------------------

    cameras: Dict[str, CameraCalibration] = {}

    if hasattr(profile, "CAMERAS"):
        for name, cam in profile.CAMERAS.items():
            try:
                mount = cam["mount"]
                optical = cam["optical"]
                meta = cam.get("meta", {})

                cameras[name] = CameraCalibration(
                    mount=CameraMount(
                        yaw_offset_deg=mount["yaw_offset_deg"],
                        x_offset_mm=mount["x_offset_mm"],
                        y_offset_mm=mount["y_offset_mm"],
                    ),
                    optical=CameraOptical(
                        distance_scale=optical["distance_scale"],
                        bearing_sign=optical["bearing_sign"],
                        bearing_offset_deg=optical["bearing_offset_deg"],
                    ),
                    meta=CameraMeta(
                        resolution=tuple(meta.get("resolution", (0, 0))),
                        fov_deg=meta.get("fov_deg", 0.0),
                        description=meta.get("description", ""),
                    ),
                )

            except KeyError as e:
                raise RuntimeError(
                    f"Camera '{name}' in calibration profile '{profile_name}' "
                    f"is missing required field: {e}"
                ) from e

    # --------------------------------------------------
    # Build final immutable calibration object
    # --------------------------------------------------

    return Calibration(
        # Drive
        drive_switch_mm=motor.DRIVE_SWITCH_MM,
        drive_power_short=motor.DRIVE_POWER_SHORT,
        drive_power_long=motor.DRIVE_POWER_LONG,
        drive_m_short=motor.DRIVE_M_SHORT,
        drive_b_short=motor.DRIVE_B_SHORT,
        drive_m_long=motor.DRIVE_M_LONG,
        drive_b_long=motor.DRIVE_B_LONG,

        # Rotate
        rotate_switch_deg=motor.ROTATE_SWITCH_DEG,

        rotate_power_small=motor.ROTATE_POWER_SMALL,
        rotate_m_small=motor.ROTATE_M_SMALL,
        rotate_b_small=motor.ROTATE_B_SMALL,

        rotate_power_large=motor.ROTATE_POWER_LARGE,
        rotate_m_large=motor.ROTATE_M_LARGE,
        rotate_b_large=motor.ROTATE_B_LARGE,

        # Voltage compensation
        voltage_reference=motor.VOLTAGE_REFERENCE,

        voltage_low_model=motor.VOLTAGE_LOW_MODEL,
        voltage_low_a=motor.VOLTAGE_LOW_A,
        voltage_low_b=getattr(motor, "VOLTAGE_LOW_B", None),

        voltage_high_model=motor.VOLTAGE_HIGH_MODEL,
        voltage_high_a=motor.VOLTAGE_HIGH_A,
        voltage_high_b=getattr(motor, "VOLTAGE_HIGH_B", None),

        # Cameras
        cameras=cameras,
    )
