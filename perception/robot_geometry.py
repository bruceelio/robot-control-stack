# perception/robot_geometry.py

from __future__ import annotations

import math

from navigation.geometry.relative_target import (
    RelativeTarget2D,
    relative_target_from_polar,
    relative_target_from_xy,
)

from navigation.geometry.transforms_2d import (
    inverse_transform_point_2d,
    transform_point_2d,
)


"""
Target geometry transforms.

Coordinate conventions
----------------------

Robot geometry:
    +x = forward
    +y = left
    +yaw = counter-clockwise / left



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

Canonical geometry functions return RelativeTarget2D:

    position = metres
    distance = metres
    bearing  = radians

    +x       = forward
    +y       = left
    +bearing = counter-clockwise / left


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


def _camera_target(
    observation: dict,
) -> RelativeTarget2D:
    """
    Return canonical camera-relative target geometry.

    Preferred perception contract:

        distance_m
        bearing_rad

    where:

        distance = metres
        +bearing = left / counter-clockwise
        bearing  = radians

    Legacy camera observations remain supported temporarily:

        distance / distance_mm
        bearing / bearing_deg

    where:

        distance = millimetres
        +bearing = image-right
        bearing  = degrees
    """

    if (
        "distance_m" in observation
        and "bearing_rad" in observation
    ):
        distance_m = float(
            observation[
                "distance_m"
            ]
        )

        bearing_rad = float(
            observation[
                "bearing_rad"
            ]
        )

        return relative_target_from_polar(
            distance_m=distance_m,
            bearing_rad=bearing_rad,
        )

    distance_mm, bearing_deg = (
        _camera_measurement(
            observation
        )
    )

    return relative_target_from_polar(
        distance_m=(
            float(distance_mm)
            / 1000.0
        ),

        # Legacy perception bearing is
        # positive image-right.
        bearing_rad=-math.radians(
            float(bearing_deg)
        ),
    )


# ==================================================
# Camera -> base_link
# ==================================================



def _target_base_link(
    *,
    observation: dict,
    config,
) -> RelativeTarget2D:
    """
    Convert a raw camera-relative observation into canonical
    target geometry relative to base_link.

    Canonical geometry:
        +x       = forward
        +y       = left
        +bearing = counter-clockwise / left

    Units:
        position = metres
        distance = metres
        bearing  = radians
    """

    camera_name = _camera_name(
        observation
    )

    camera_target = _camera_target(
        observation
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

    mount_x_m = float(
        mount["x_m"]
    )

    mount_y_m = float(
        mount["y_m"]
    )

    mount_yaw_rad = float(
        mount.get(
            "yaw_rad",
            0.0,
        )
    )

    (
        target_x_base_m,
        target_y_base_m,
    ) = transform_point_2d(
        point_x=camera_target.x_m,
        point_y=camera_target.y_m,
        frame_x=mount_x_m,
        frame_y=mount_y_m,
        frame_yaw_rad=mount_yaw_rad,
    )

    return relative_target_from_xy(
        x_m=target_x_base_m,
        y_m=target_y_base_m,
    )

def _target_gripper(
    *,
    observation: dict,
    config,
) -> RelativeTarget2D:
    """
    Convert a raw camera-relative observation into canonical
    target geometry relative to the gripper frame.

    Transformation path:

        source camera
            ->
        base_link
            ->
        gripper

    Canonical geometry:
        +x       = forward
        +y       = left
        +bearing = counter-clockwise / left

    Units:
        position = metres
        distance = metres
        bearing  = radians
    """

    base_target = _target_base_link(
        observation=observation,
        config=config,
    )

    grip = config.gripper_mount

    grip_x_m = float(
        grip["x_m"]
    )

    grip_y_m = float(
        grip["y_m"]
    )

    grip_yaw_rad = float(
        grip.get(
            "yaw_rad",
            0.0,
        )
    )

    (
        target_x_grip_m,
        target_y_grip_m,
    ) = inverse_transform_point_2d(
        point_x=base_target.x_m,
        point_y=base_target.y_m,
        frame_x=grip_x_m,
        frame_y=grip_y_m,
        frame_yaw_rad=grip_yaw_rad,
    )

    return relative_target_from_xy(
        x_m=target_x_grip_m,
        y_m=target_y_grip_m,
    )



# ==================================================
# Public geometry API
# ==================================================

def relative_target_from_camera(
    *,
    observation: dict,
) -> RelativeTarget2D:
    """
    Return canonical target geometry relative to the source
    camera frame.

    Units:
        position = metres
        distance = metres
        bearing  = radians

    Convention:
        +x       = forward
        +y       = left
        +bearing = counter-clockwise / left
    """

    return _camera_target(
        observation
    )


def relative_target_from_base_link(
    *,
    observation: dict,
    config,
) -> RelativeTarget2D:
    """
    Return canonical target geometry relative to base_link.
    """

    return _target_base_link(
        observation=observation,
        config=config,
    )


def relative_target_from_gripper(
    *,
    observation: dict,
    config,
) -> RelativeTarget2D:
    """
    Return canonical target geometry relative to the gripper.
    """

    return _target_gripper(
        observation=observation,
        config=config,
    )



__all__ = [
    "relative_target_from_base_link",
    "relative_target_from_camera",
    "relative_target_from_gripper",
]