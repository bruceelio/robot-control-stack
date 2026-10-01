# navigation/geometry/transforms_2d.py

from __future__ import annotations

import math


def rotate_vector_2d(
    *,
    x: float,
    y: float,
    angle_rad: float,
) -> tuple[float, float]:
    """
    Rotate a 2-D vector counter-clockwise.

    Coordinate convention:

        +x     = forward
        +y     = left
        +angle = counter-clockwise / left

    x and y may use any consistent linear unit.
    """

    x = _finite_float(
        x,
        name="x",
    )

    y = _finite_float(
        y,
        name="y",
    )

    angle_rad = _finite_float(
        angle_rad,
        name="angle_rad",
    )

    cos_angle = math.cos(
        angle_rad
    )

    sin_angle = math.sin(
        angle_rad
    )

    return (
        cos_angle * x
        - sin_angle * y,

        sin_angle * x
        + cos_angle * y,
    )


def transform_point_2d(
    *,
    point_x: float,
    point_y: float,
    frame_x: float,
    frame_y: float,
    frame_yaw_rad: float,
) -> tuple[float, float]:
    """
    Transform a point from a local frame into its parent frame.

    frame_x / frame_y:
        Position of the local-frame origin in the parent frame.

    frame_yaw_rad:
        Orientation of the local frame relative to the parent.

    Equivalent to:

        p_parent = translation + R(yaw) * p_local
    """

    point_x = _finite_float(
        point_x,
        name="point_x",
    )

    point_y = _finite_float(
        point_y,
        name="point_y",
    )

    frame_x = _finite_float(
        frame_x,
        name="frame_x",
    )

    frame_y = _finite_float(
        frame_y,
        name="frame_y",
    )

    frame_yaw_rad = _finite_float(
        frame_yaw_rad,
        name="frame_yaw_rad",
    )

    rotated_x, rotated_y = rotate_vector_2d(
        x=point_x,
        y=point_y,
        angle_rad=frame_yaw_rad,
    )

    return (
        frame_x + rotated_x,
        frame_y + rotated_y,
    )


def inverse_transform_point_2d(
    *,
    point_x: float,
    point_y: float,
    frame_x: float,
    frame_y: float,
    frame_yaw_rad: float,
) -> tuple[float, float]:
    """
    Transform a point from a parent frame into a local frame.

    This is the inverse of transform_point_2d().

    Equivalent to:

        p_local = R(-yaw) * (p_parent - translation)
    """

    point_x = _finite_float(
        point_x,
        name="point_x",
    )

    point_y = _finite_float(
        point_y,
        name="point_y",
    )

    frame_x = _finite_float(
        frame_x,
        name="frame_x",
    )

    frame_y = _finite_float(
        frame_y,
        name="frame_y",
    )

    frame_yaw_rad = _finite_float(
        frame_yaw_rad,
        name="frame_yaw_rad",
    )

    delta_x = (
        point_x - frame_x
    )

    delta_y = (
        point_y - frame_y
    )

    return rotate_vector_2d(
        x=delta_x,
        y=delta_y,
        angle_rad=-frame_yaw_rad,
    )


def _finite_float(
    value: float,
    *,
    name: str,
) -> float:
    value = float(value)

    if not math.isfinite(value):
        raise ValueError(
            f"{name} must be finite"
        )

    return value


__all__ = [
    "inverse_transform_point_2d",
    "rotate_vector_2d",
    "transform_point_2d",
]