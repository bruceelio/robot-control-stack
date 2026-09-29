from __future__ import annotations

import math

from navigation.wall_geometry import (
    RangeObservation2D,
    WallGeometry,
)


def _hit_point(
    observation: RangeObservation2D,
) -> tuple[float, float]:

    if not math.isfinite(observation.distance_mm):
        raise ValueError(
            "range distance must be finite"
        )

    if observation.distance_mm <= 0.0:
        raise ValueError(
            "range distance must be positive"
        )

    x_mm = (
        observation.sensor_x_mm
        + observation.distance_mm
        * math.cos(observation.sensor_yaw_rad)
    )

    y_mm = (
        observation.sensor_y_mm
        + observation.distance_mm
        * math.sin(observation.sensor_yaw_rad)
    )

    return x_mm, y_mm


def estimate_wall_geometry_two_ranges(
    observation_a: RangeObservation2D,
    observation_b: RangeObservation2D,
) -> WallGeometry:
    """
    Estimate wall heading and perpendicular distance from two
    independent range observations.

    The observations may come from any ranging technology.

    Each measurement ray is converted into a wall hit point.
    The line through those points defines the wall.
    """

    ax, ay = _hit_point(observation_a)
    bx, by = _hit_point(observation_b)

    wall_dx = bx - ax
    wall_dy = by - ay

    wall_span_mm = math.hypot(
        wall_dx,
        wall_dy,
    )

    if wall_span_mm <= 1e-6:
        raise ValueError(
            "range observations do not define a wall line"
        )

    # One of the two normals to the wall line.
    normal_x = wall_dy / wall_span_mm
    normal_y = -wall_dx / wall_span_mm

    midpoint_x = 0.5 * (ax + bx)
    midpoint_y = 0.5 * (ay + by)

    # Select the normal which points from base_link toward
    # the observed wall.
    if (
        normal_x * midpoint_x
        + normal_y * midpoint_y
    ) < 0.0:

        normal_x = -normal_x
        normal_y = -normal_y

    distance_mm = (
        normal_x * midpoint_x
        + normal_y * midpoint_y
    )

    heading_rad = math.atan2(
        normal_y,
        normal_x,
    )

    return WallGeometry(
        heading_rad=heading_rad,
        distance_mm=distance_mm,
    )