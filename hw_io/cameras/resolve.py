# hw_io/cameras/resolve.py

from __future__ import annotations

from config.cameras.resolve import resolve_camera_config
from calibration.cameras.resolve import resolve_camera_calibration
from config.arena_tags import resolve_tag_size_m

from hw_io.cameras.sr_april import SRAprilCamera
from hw_io.cameras.pi_libcamera_april import PiLibcameraAprilCamera


def resolve_camera(*, camera_name: str, robot):
    """
    Build one camera backend from a named camera config.

    Parameters
    ----------
    camera_name:
        Name from CONFIG.cameras, for example:
            "sr"
            "pi3"

    robot:
        The SR Robot() object if available, else None.
        Required for SR-backed cameras.
    """
    cam_cfg = resolve_camera_config(camera_name)
    backend = cam_cfg.BACKEND

    if backend == "sr":
        if robot is None:
            raise RuntimeError(
                f"Camera config {camera_name!r} requested SR backend, "
                f"but no SR robot object was provided"
            )

        # Current assumption:
        # SR exposes a single camera as robot.camera.
        return SRAprilCamera(robot.camera)

    if backend == "pi_libcamera_april":
        calibration_profile = getattr(cam_cfg, "CALIBRATION_PROFILE", None)
        calibration = None

        if calibration_profile is not None:
            calibration = resolve_camera_calibration(calibration_profile)

        camera_params = None
        if calibration is not None:
            camera_params = getattr(calibration, "CAMERA_PARAMS", None)

        return PiLibcameraAprilCamera(
            width=cam_cfg.WIDTH,
            height=cam_cfg.HEIGHT,
            fps=cam_cfg.FPS,
            families=cam_cfg.FAMILIES,
            camera_params=camera_params,
            min_decision_margin=cam_cfg.MIN_DECISION_MARGIN,
            tag_size_for_id=resolve_tag_size_m,
        )

    raise RuntimeError(
        f"Unknown camera backend {backend!r} for camera config {camera_name!r}"
    )