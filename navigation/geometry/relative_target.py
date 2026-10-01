# navigation/geometry/relative_target.py

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class RelativeTarget2D:
    """
    A target point expressed relative to a reference frame.

    Coordinate convention:

        +x       = forward
        +y       = left
        +bearing = counter-clockwise / left

    Units:

        position = metres
        distance = metres
        bearing  = radians

    bearing_rad is the standard planar geometric bearing:

        bearing_rad = atan2(y_m, x_m)
    """

    x_m: float
    y_m: float

    distance_m: float
    bearing_rad: float


def relative_target_from_polar(
    *,
    distance_m: float,
    bearing_rad: float,
) -> RelativeTarget2D:
    """
    Construct a relative target from polar geometry.

    Bearing convention:

        0            = forward
        +pi/2        = left
        -pi/2        = right
        +/-pi        = behind

    Positive bearing is counter-clockwise / left.
    """

    distance_m = float(distance_m)
    bearing_rad = float(bearing_rad)

    if not math.isfinite(distance_m):
        raise ValueError(
            "distance_m must be finite"
        )

    if not math.isfinite(bearing_rad):
        raise ValueError(
            "bearing_rad must be finite"
        )

    if distance_m < 0.0:
        raise ValueError(
            "distance_m must be >= 0"
        )

    bearing_rad = _wrap_angle_rad(
        bearing_rad
    )

    # Bearing is undefined at zero range.
    # Use zero as the deterministic canonical representation.
    if distance_m == 0.0:
        return RelativeTarget2D(
            x_m=0.0,
            y_m=0.0,
            distance_m=0.0,
            bearing_rad=0.0,
        )

    x_m = (
        distance_m
        * math.cos(bearing_rad)
    )

    y_m = (
        distance_m
        * math.sin(bearing_rad)
    )

    return RelativeTarget2D(
        x_m=x_m,
        y_m=y_m,
        distance_m=distance_m,
        bearing_rad=bearing_rad,
    )


def relative_target_from_xy(
    *,
    x_m: float,
    y_m: float,
) -> RelativeTarget2D:
    """
    Construct a relative target from Cartesian geometry.

    The resulting bearing is:

        atan2(y_m, x_m)

    and therefore follows the canonical positive-left /
    counter-clockwise convention.
    """

    x_m = float(x_m)
    y_m = float(y_m)

    if not math.isfinite(x_m):
        raise ValueError(
            "x_m must be finite"
        )

    if not math.isfinite(y_m):
        raise ValueError(
            "y_m must be finite"
        )

    distance_m = math.hypot(
        x_m,
        y_m,
    )

    # Bearing is undefined at the origin.
    if distance_m == 0.0:
        bearing_rad = 0.0
    else:
        bearing_rad = math.atan2(
            y_m,
            x_m,
        )

    return RelativeTarget2D(
        x_m=x_m,
        y_m=y_m,
        distance_m=distance_m,
        bearing_rad=bearing_rad,
    )


def _wrap_angle_rad(
    angle_rad: float,
) -> float:
    """
    Wrap an angle to [-pi, pi].
    """

    return math.atan2(
        math.sin(angle_rad),
        math.cos(angle_rad),
    )


__all__ = [
    "RelativeTarget2D",
    "relative_target_from_polar",
    "relative_target_from_xy",
]