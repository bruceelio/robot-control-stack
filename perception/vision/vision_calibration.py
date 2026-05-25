# perception/vision/vision_calibration.py

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module
from typing import Any

from calibration import CALIBRATION
from config import CONFIG


@dataclass(frozen=True)
class VisionPnPCalibration:
    source_id: str
    camera_name: str
    camera_matrix: tuple[tuple[float, float, float], ...]
    distortion_coefficients: tuple[float, float, float, float, float]
    camera_to_robot_transform: Any


def get_vision_pnp_calibration(
    *,
    source_id: str,
    perception_camera_name: str,
) -> VisionPnPCalibration:
    vision_cfg = CONFIG.vision_sources[source_id]
    camera_name = vision_cfg["camera"]

    if camera_name != perception_camera_name:
        raise RuntimeError(
            f"Vision source mismatch: {source_id} configured for "
            f"camera={camera_name!r}, but perception is using "
            f"camera={perception_camera_name!r}"
        )

    camera_profile_name = CONFIG.cameras[camera_name]
    camera_profile = import_module(f"config.cameras.{camera_profile_name}")

    calibration_profile_name = camera_profile.CALIBRATION_PROFILE
    camera_cal_profile = import_module(
        f"calibration.cameras.{calibration_profile_name}"
    )

    fx, fy, cx, cy = getattr(
        camera_cal_profile,
        "PNP_CAMERA_PARAMS",
        camera_cal_profile.CAMERA_PARAMS,
    )

    camera_matrix = (
        (float(fx), 0.0, float(cx)),
        (0.0, float(fy), float(cy)),
        (0.0, 0.0, 1.0),
    )

    distortion_coefficients = tuple(
        float(v)
        for v in getattr(
            camera_cal_profile,
            "PNP_DISTORTION_COEFFICIENTS",
            getattr(
                camera_cal_profile,
                "DISTORTION_COEFFICIENTS",
                (0.0, 0.0, 0.0, 0.0, 0.0),
            ),
        )
    )

    robot_camera_cal = CALIBRATION.cameras[camera_name]

    return VisionPnPCalibration(
        source_id=source_id,
        camera_name=camera_name,
        camera_matrix=camera_matrix,
        distortion_coefficients=distortion_coefficients,
        camera_to_robot_transform=robot_camera_cal.mount,
    )