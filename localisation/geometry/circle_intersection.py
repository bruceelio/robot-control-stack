# localisation/geometry/circle_intersection.py

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CircleIntersectionPoint2D:
    """
    One finite intersection point between two circles.

    All positions and radii are expressed in the same
    Cartesian coordinate frame.

    Units:
        position = metres
    """

    x_m: float
    y_m: float


def circle_intersections_2d(
    *,
    center_a_x_m: float,
    center_a_y_m: float,
    radius_a_m: float,
    center_b_x_m: float,
    center_b_y_m: float,
    radius_b_m: float,
    epsilon_m: float = 1e-9,
) -> tuple[CircleIntersectionPoint2D, ...]:
    """
    Return the finite intersection points of two circles.

    Circle A:

        centre = (
            center_a_x_m,
            center_a_y_m,
        )

        radius = radius_a_m

    Circle B:

        centre = (
            center_b_x_m,
            center_b_y_m,
        )

        radius = radius_b_m

    Returns:

        ()
            no finite intersection

        (point,)
            one intersection, including tangency

        (point_left, point_right)
            two intersections

    For the two-point case, point_left lies to the left of
    the directed line from centre A to centre B.

    Coincident non-zero circles have infinitely many
    intersections and therefore raise ValueError.

    This module performs geometry only. It does not know
    which physical features produced the circles, nor does
    it decide which candidate position is correct.
    """

    values = (
        center_a_x_m,
        center_a_y_m,
        radius_a_m,
        center_b_x_m,
        center_b_y_m,
        radius_b_m,
        epsilon_m,
    )

    if not all(
        math.isfinite(float(value))
        for value in values
    ):
        raise ValueError(
            "circle geometry must be finite"
        )

    center_a_x_m = float(center_a_x_m)
    center_a_y_m = float(center_a_y_m)
    radius_a_m = float(radius_a_m)

    center_b_x_m = float(center_b_x_m)
    center_b_y_m = float(center_b_y_m)
    radius_b_m = float(radius_b_m)

    epsilon_m = float(epsilon_m)

    if radius_a_m < 0.0:
        raise ValueError(
            "radius_a_m must be >= 0"
        )

    if radius_b_m < 0.0:
        raise ValueError(
            "radius_b_m must be >= 0"
        )

    if epsilon_m <= 0.0:
        raise ValueError(
            "epsilon_m must be > 0"
        )

    delta_x_m = (
        center_b_x_m
        - center_a_x_m
    )

    delta_y_m = (
        center_b_y_m
        - center_a_y_m
    )

    center_distance_m = math.hypot(
        delta_x_m,
        delta_y_m,
    )

    # --------------------------------------------------
    # Same centre
    # --------------------------------------------------

    if center_distance_m <= epsilon_m:

        # Two zero-radius circles at the same centre
        # represent one finite point.
        if (
            radius_a_m <= epsilon_m
            and radius_b_m <= epsilon_m
        ):
            return (
                CircleIntersectionPoint2D(
                    x_m=center_a_x_m,
                    y_m=center_a_y_m,
                ),
            )

        # Equal non-zero radii with a common centre
        # describe the same circle.
        if abs(
            radius_a_m
            - radius_b_m
        ) <= epsilon_m:
            raise ValueError(
                "coincident circles have infinitely many "
                "intersections"
            )

        # Concentric circles with different radii.
        return ()

    # --------------------------------------------------
    # Reject non-intersecting geometry
    # --------------------------------------------------

    radius_sum_m = (
        radius_a_m
        + radius_b_m
    )

    radius_difference_m = abs(
        radius_a_m
        - radius_b_m
    )

    # Too far apart.
    if (
        center_distance_m
        > radius_sum_m + epsilon_m
    ):
        return ()

    # One lies entirely inside the other.
    if (
        center_distance_m
        < radius_difference_m - epsilon_m
    ):
        return ()

    # --------------------------------------------------
    # Intersection chord
    # --------------------------------------------------
    #
    # Distance from centre A to the midpoint of the
    # common chord, measured along A -> B.
    #

    chord_axis_m = (
        (
            radius_a_m * radius_a_m
            - radius_b_m * radius_b_m
            + center_distance_m
            * center_distance_m
        )
        / (
            2.0
            * center_distance_m
        )
    )

    chord_half_height_sq_m2 = (
        radius_a_m * radius_a_m
        - chord_axis_m * chord_axis_m
    )

    # Floating-point arithmetic can produce a tiny
    # negative value for a geometrically tangent case.
    squared_tolerance_m2 = (
        epsilon_m
        * max(
            radius_a_m,
            radius_b_m,
            center_distance_m,
            epsilon_m,
        )
    )

    if (
        chord_half_height_sq_m2
        < -squared_tolerance_m2
    ):
        return ()

    chord_half_height_m = math.sqrt(
        max(
            0.0,
            chord_half_height_sq_m2,
        )
    )

    axis_unit_x = (
        delta_x_m
        / center_distance_m
    )

    axis_unit_y = (
        delta_y_m
        / center_distance_m
    )

    chord_mid_x_m = (
        center_a_x_m
        + chord_axis_m
        * axis_unit_x
    )

    chord_mid_y_m = (
        center_a_y_m
        + chord_axis_m
        * axis_unit_y
    )

    # --------------------------------------------------
    # Tangent
    # --------------------------------------------------

    if (
        chord_half_height_m
        <= epsilon_m
    ):
        return (
            CircleIntersectionPoint2D(
                x_m=chord_mid_x_m,
                y_m=chord_mid_y_m,
            ),
        )

    # --------------------------------------------------
    # Two intersections
    # --------------------------------------------------
    #
    # A left-pointing perpendicular to A -> B is:
    #
    #     (-axis_y, +axis_x)
    #

    left_normal_x = -axis_unit_y
    left_normal_y = axis_unit_x

    point_left = CircleIntersectionPoint2D(
        x_m=(
            chord_mid_x_m
            + chord_half_height_m
            * left_normal_x
        ),
        y_m=(
            chord_mid_y_m
            + chord_half_height_m
            * left_normal_y
        ),
    )

    point_right = CircleIntersectionPoint2D(
        x_m=(
            chord_mid_x_m
            - chord_half_height_m
            * left_normal_x
        ),
        y_m=(
            chord_mid_y_m
            - chord_half_height_m
            * left_normal_y
        ),
    )

    return (
        point_left,
        point_right,
    )


__all__ = [
    "CircleIntersectionPoint2D",
    "circle_intersections_2d",
]