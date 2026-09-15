# calibration/resolve.py

from importlib import import_module
from typing import Dict

from calibration.cameras.resolve import resolve_camera_calibration

from calibration.schema import (
    Calibration,
    CameraCalibration,
    CameraOptical,
    CameraMeta,
)


# --------------------------------------------------
# Helpers
# --------------------------------------------------

def _camera_calibration_name(camera_name: str, camera_config) -> str:
    """
    Return the calibration profile name for one configured camera.

    Normal physical cameras usually use the same name for the camera
    backend profile and calibration profile.

    Example:
        {
            "profile": "arducam_fullfov_640_400",
            "device": "/dev/video0",
        }

    Simulation / SR cameras may use a different backend and calibration:

        {
            "profile": "sr",
            "calibration": "webots_2026_cam",
            "device": None,
        }
    """

    # Older/simple format:
    #
    # CAMERAS = {
    #     "front": "pi3_fullfov_640_360",
    # }
    if isinstance(camera_config, str):
        return camera_config

    if isinstance(camera_config, dict):
        calibration_name = camera_config.get("calibration")

        if calibration_name:
            return calibration_name

        profile_name = camera_config.get("profile")

        if profile_name:
            return profile_name

    raise RuntimeError(
        f"Camera '{camera_name}' does not specify a usable "
        f"camera calibration profile."
    )


# --------------------------------------------------
# Resolver
# --------------------------------------------------

def resolve(*, config) -> Calibration:
    """
    Resolve calibration for the selected robot configuration.

    Motor calibration is selected by:
        config.drive_motor_profile

    Camera calibration is selected by:
        config.cameras

    Robot-level calibration profiles are no longer used.
    """

    # --------------------------------------------------
    # Resolve motor calibration
    # --------------------------------------------------

    motor_module_path = (
        f"calibration.motors.{config.drive_motor_profile}"
    )

    motor = import_module(motor_module_path)

    # --------------------------------------------------
    # Resolve cameras
    # --------------------------------------------------

    cameras: Dict[str, CameraCalibration] = {}

    for camera_name, camera_config in config.cameras.items():

        calibration_name = _camera_calibration_name(
            camera_name,
            camera_config,
        )

        camera = resolve_camera_calibration(
            calibration_name
        )

        # Camera mount geometry is owned by the robot profile
        # and resolved through CONFIG.camera_mounts..


        optical = CameraOptical(
            distance_scale=getattr(
                camera,
                "DISTANCE_SCALE",
                1.0,
            ),
            bearing_sign=getattr(
                camera,
                "BEARING_SIGN",
                1.0,
            ),
            bearing_offset_deg=getattr(
                camera,
                "BEARING_OFFSET_DEG",
                0.0,
            ),
        )

        meta = CameraMeta(
            resolution=tuple(
                getattr(
                    camera,
                    "RESOLUTION",
                    (0, 0),
                )
            ),
            fov_deg=getattr(
                camera,
                "FOV_DEG",
                0.0,
            ),
            description=getattr(
                camera,
                "DESCRIPTION",
                "",
            ),
        )

        cameras[camera_name] = CameraCalibration(
            optical=optical,
            meta=meta,
        )

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

        drive_velocity_curve=tuple(
            (float(power), float(velocity_mm_s))
            for power, velocity_mm_s in motor.DRIVE_VELOCITY_CURVE
        ),

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
        voltage_low_b=getattr(
            motor,
            "VOLTAGE_LOW_B",
            None,
        ),

        voltage_high_model=motor.VOLTAGE_HIGH_MODEL,
        voltage_high_a=motor.VOLTAGE_HIGH_A,
        voltage_high_b=getattr(
            motor,
            "VOLTAGE_HIGH_B",
            None,
        ),

        # Cameras
        cameras=cameras,
    )