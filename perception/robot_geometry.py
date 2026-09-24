# perception/robot_geometry.py

from __future__ import annotations

import math


"""
Target geometry transforms.

Coordinate conventions
----------------------

Robot geometry:
    +x = forward
    +y = left
    +yaw = counter-clockwise / left

Control bearing:
    +bearing = right
    -bearing = left

Vision observations are expected to be CAMERA-RELATIVE and to contain
the logical source camera name.

Supported observation shapes:

Arena detection:
    {
        "distance_mm": ...,
        "bearing_deg": ...,
        "camera": "front",
    }

Perception object:
    {
        "distance": ...,
        "bearing": ...,
        "camera": "front",
    }

Public functions return:

    (distance_mm, bearing_deg)

relative to the requested frame.
"""


# ==================================================
# Observation extraction
# ==================================================

def _camera_name(observation: dict) -> str:
    """
    Return the logical camera which produced the observation.

    Camera identity is mandatory because camera mounting geometry
    may differ between cameras.
    """

    camera_name = observation.get("camera")

    if not camera_name:
        raise ValueError(
            "geometry observation has no source camera"
        )

    return str(camera_name)


def _camera_measurement(
    observation: dict,
) -> tuple[float, float]:
    """
    Extract camera-relative distance and bearing from either
    supported perception observation format.
    """

    if "distance_mm" in observation:
        distance_mm = float(
            observation["distance_mm"]
        )

    elif "distance" in observation:
        distance_mm = float(
            observation["distance"]
        )

    else:
        raise ValueError(
            "geometry observation has no distance"
        )

    if "bearing_deg" in observation:
        bearing_deg = float(
            observation["bearing_deg"]
        )

    elif "bearing" in observation:
        bearing_deg = float(
            observation["bearing"]
        )

    else:
        raise ValueError(
            "geometry observation has no bearing"
        )

    return distance_mm, bearing_deg


# ==================================================
# Polar / Cartesian conversion
# ==================================================

def _polar_to_xy(
    distance_mm: float,
    bearing_deg: float,
) -> tuple[float, float]:
    """
    Convert control bearing into robot-style XY geometry.

    Control:
        +bearing = right

    Geometry:
        +x = forward
        +y = left
    """

    bearing_rad = math.radians(
        float(bearing_deg)
    )

    x_mm = (
        float(distance_mm)
        * math.cos(bearing_rad)
    )

    y_mm = (
        -float(distance_mm)
        * math.sin(bearing_rad)
    )

    return x_mm, y_mm


def _xy_to_polar(
    x_mm: float,
    y_mm: float,
) -> tuple[float, float]:
    """
    Convert robot-style XY geometry back into control
    distance/bearing.
    """

    distance_mm = math.hypot(
        x_mm,
        y_mm,
    )

    bearing_deg = -math.degrees(
        math.atan2(
            y_mm,
            x_mm,
        )
    )

    return distance_mm, bearing_deg


# ==================================================
# Camera -> base_link
# ==================================================

def _target_xy_base_link(
    *,
    observation: dict,
    config,
) -> tuple[float, float]:
    """
    Convert a camera-relative observation into target XY
    relative to base_link.
    """

    camera_name = _camera_name(
        observation
    )

    distance_mm, bearing_deg = (
        _camera_measurement(
            observation
        )
    )

    target_x_cam, target_y_cam = (
        _polar_to_xy(
            distance_mm,
            bearing_deg,
        )
    )

    try:
        mount = config.camera_mounts[
            camera_name
        ]

    except KeyError as exc:
        raise ValueError(
            f"no camera mount configured "
            f"for {camera_name!r}"
        ) from exc

    mount_x_mm = float(
        mount["x_mm"]
    )

    mount_y_mm = float(
        mount["y_mm"]
    )

    mount_yaw_rad = math.radians(
        float(
            mount.get(
                "yaw_deg",
                0.0,
            )
        )
    )

    cos_yaw = math.cos(
        mount_yaw_rad
    )

    sin_yaw = math.sin(
        mount_yaw_rad
    )

    # Camera coordinates -> base_link orientation.
    rotated_x_mm = (
        cos_yaw * target_x_cam
        - sin_yaw * target_y_cam
    )

    rotated_y_mm = (
        sin_yaw * target_x_cam
        + cos_yaw * target_y_cam
    )

    # Camera origin -> base_link origin.
    target_x_base_mm = (
        mount_x_mm
        + rotated_x_mm
    )

    target_y_base_mm = (
        mount_y_mm
        + rotated_y_mm
    )

    return (
        target_x_base_mm,
        target_y_base_mm,
    )


# ==================================================
# Public geometry API
# ==================================================

def target_from_camera(
    *,
    observation: dict,
) -> tuple[float, float]:
    """
    Return target distance/bearing relative to the camera
    which produced the observation.

    No robot transform is applied.
    """

    return _camera_measurement(
        observation
    )


def target_from_base_link(
    *,
    observation: dict,
    config,
) -> tuple[float, float]:
    """
    Return target distance/bearing relative to robot base_link.

    base_link is the canonical robot reference frame.
    """

    x_mm, y_mm = _target_xy_base_link(
        observation=observation,
        config=config,
    )

    return _xy_to_polar(
        x_mm,
        y_mm,
    )


def target_from_gripper(
    *,
    observation: dict,
    config,
) -> tuple[float, float]:
    """
    Return target distance/bearing relative to the gripper.

    Transformation path:

        source camera
            ->
        base_link
            ->
        gripper
    """

    target_x_base_mm, target_y_base_mm = (
        _target_xy_base_link(
            observation=observation,
            config=config,
        )
    )

    grip = config.gripper_mount

    grip_x_mm = float(
        grip["x_mm"]
    )

    grip_y_mm = float(
        grip["y_mm"]
    )

    grip_yaw_rad = math.radians(
        float(
            grip.get(
                "yaw_deg",
                0.0,
            )
        )
    )

    # Move origin from base_link to gripper.
    delta_x_mm = (
        target_x_base_mm
        - grip_x_mm
    )

    delta_y_mm = (
        target_y_base_mm
        - grip_y_mm
    )

    # Rotate base_link axes into gripper axes.
    cos_yaw = math.cos(
        grip_yaw_rad
    )

    sin_yaw = math.sin(
        grip_yaw_rad
    )

    target_x_grip_mm = (
        cos_yaw * delta_x_mm
        + sin_yaw * delta_y_mm
    )

    target_y_grip_mm = (
        -sin_yaw * delta_x_mm
        + cos_yaw * delta_y_mm
    )

    return _xy_to_polar(
        target_x_grip_mm,
        target_y_grip_mm,
    )


__all__ = [
    "target_from_camera",
    "target_from_base_link",
    "target_from_gripper",
]