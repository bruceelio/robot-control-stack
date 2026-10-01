# navigation/wall_geometry/two_ray_plane.py

from __future__ import annotations

import math

from navigation.wall_geometry.models import (
    RangeRay2D,
    WallGeometry,
)


def _hit_point(
    ray: RangeRay2D,
) -> tuple[float, float]:

    if not math.isfinite(ray.distance_mm):
        raise ValueError(
            "range distance must be finite"
        )

    if ray.distance_mm <= 0.0:
        raise ValueError(
            "range distance must be positive"
        )

    x_mm = (
            ray.origin_x_mm
            + ray.distance_mm
            * math.cos(ray.ray_heading_rad)
    )

    y_mm = (
            ray.origin_y_mm
            + ray.distance_mm
            * math.sin(ray.ray_heading_rad)
    )

    return x_mm, y_mm


def estimate_wall_from_two_rays(
    ray_a: RangeRay2D,
    ray_b: RangeRay2D,
) -> WallGeometry:
    """
    Estimate a planar wall from two range rays.

    Both rays must be expressed in the same coordinate frame and
    must intersect the same physical wall plane.

    Each range ray is converted into a hit point. The line through
    those two hit points defines the wall.
    """

    ax, ay = _hit_point(ray_a)
    bx, by = _hit_point(ray_b)

    wall_dx = bx - ax
    wall_dy = by - ay

    wall_span_mm = math.hypot(
        wall_dx,
        wall_dy,
    )

    if wall_span_mm <= 1e-6:
        raise ValueError(
        "range rays do not define a wall line"
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