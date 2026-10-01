# navigation/geometry/ray_line_intersection.py

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RayLineIntersection2D:
    """
    Intersection of a 2D ray with an infinite 2D line.

    Coordinate convention:

        +x = forward
        +y = left

    Units:

        x / y             = metres
        ray_distance_m    = metres
        line_parameter_m  = metres

    line_parameter_m is signed distance along the supplied
    line direction from the supplied line point.
    """

    x_m: float
    y_m: float

    ray_distance_m: float
    line_parameter_m: float


def ray_line_intersection_2d(
    *,
    ray_origin_x_m: float,
    ray_origin_y_m: float,
    ray_heading_rad: float,
    line_point_x_m: float,
    line_point_y_m: float,
    line_direction_x: float,
    line_direction_y: float,
    parallel_epsilon: float = 1e-12,
) -> RayLineIntersection2D | None:
    """
    Intersect a 2D ray with an infinite 2D line.

    Ray:

        origin
            +
        t * ray_direction

        where t >= 0

    Infinite line:

        line_point
            +
        u * line_direction

        where u may be positive or negative.

    The line direction does not need to be normalized.

    Returns None when:

        - the ray and line are parallel;
        - the ray and line are collinear;
        - the mathematical intersection lies behind the ray origin.

    Raises ValueError for invalid or degenerate input.
    """

    values = (
        ray_origin_x_m,
        ray_origin_y_m,
        ray_heading_rad,
        line_point_x_m,
        line_point_y_m,
        line_direction_x,
        line_direction_y,
        parallel_epsilon,
    )

    if not all(
        math.isfinite(float(value))
        for value in values
    ):
        raise ValueError(
            "ray/line geometry must be finite"
        )

    parallel_epsilon = float(
        parallel_epsilon
    )

    if parallel_epsilon <= 0.0:
        raise ValueError(
            "parallel_epsilon must be > 0"
        )

    ray_origin_x_m = float(
        ray_origin_x_m
    )

    ray_origin_y_m = float(
        ray_origin_y_m
    )

    ray_heading_rad = float(
        ray_heading_rad
    )

    line_point_x_m = float(
        line_point_x_m
    )

    line_point_y_m = float(
        line_point_y_m
    )

    line_direction_x = float(
        line_direction_x
    )

    line_direction_y = float(
        line_direction_y
    )

    line_direction_norm = math.hypot(
        line_direction_x,
        line_direction_y,
    )

    if line_direction_norm <= parallel_epsilon:
        raise ValueError(
            "line direction must be non-zero"
        )

    line_direction_x /= (
        line_direction_norm
    )

    line_direction_y /= (
        line_direction_norm
    )

    ray_direction_x = math.cos(
        ray_heading_rad
    )

    ray_direction_y = math.sin(
        ray_heading_rad
    )

    delta_x = (
        line_point_x_m
        - ray_origin_x_m
    )

    delta_y = (
        line_point_y_m
        - ray_origin_y_m
    )

    denominator = _cross_2d(
        ray_direction_x,
        ray_direction_y,
        line_direction_x,
        line_direction_y,
    )

    if abs(denominator) <= parallel_epsilon:
        return None

    ray_distance_m = (
        _cross_2d(
            delta_x,
            delta_y,
            line_direction_x,
            line_direction_y,
        )
        / denominator
    )

    line_parameter_m = (
        _cross_2d(
            delta_x,
            delta_y,
            ray_direction_x,
            ray_direction_y,
        )
        / denominator
    )

    if ray_distance_m < -parallel_epsilon:
        return None

    # Numerical tolerance around the ray origin.
    if ray_distance_m < 0.0:
        ray_distance_m = 0.0

    intersection_x_m = (
        ray_origin_x_m
        + ray_distance_m
        * ray_direction_x
    )

    intersection_y_m = (
        ray_origin_y_m
        + ray_distance_m
        * ray_direction_y
    )

    return RayLineIntersection2D(
        x_m=intersection_x_m,
        y_m=intersection_y_m,
        ray_distance_m=ray_distance_m,
        line_parameter_m=line_parameter_m,
    )


def _cross_2d(
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float:
    """
    Return the scalar 2D cross product a x b.
    """

    return (
        ax * by
        - ay * bx
    )


__all__ = [
    "RayLineIntersection2D",
    "ray_line_intersection_2d",
]