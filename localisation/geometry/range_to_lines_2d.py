# localisation/geometry/range_to_lines_2d.py

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RangePositionLine2D:
    """
    One possible locus of observer positions produced by a
    perpendicular range to a known infinite line.

    The line is stored in unit-normal form:

        normal_x * x + normal_y * y = offset_m

    side_sign records which side of the mapped line produced this
    locus:

        +1 = positive side of the mapped line's left normal
        -1 = negative side
         0 = zero-range locus coincident with the mapped line

    Units:
        offset / position = metres
    """

    normal_x: float
    normal_y: float
    offset_m: float
    side_sign: int

    def residual_m(
        self,
        *,
        x_m: float,
        y_m: float,
    ) -> float:
        """
        Signed perpendicular residual from a point to this locus.
        """

        return (
            self.normal_x * float(x_m)
            + self.normal_y * float(y_m)
            - self.offset_m
        )


@dataclass(frozen=True)
class RangeLineIntersectionPoint2D:
    """
    Intersection point of two non-parallel position loci.

    Units:
        position = metres
    """

    x_m: float
    y_m: float


def range_to_lines_2d(
    *,
    mapped_line_point_x_m: float,
    mapped_line_point_y_m: float,
    mapped_line_direction_x: float,
    mapped_line_direction_y: float,
    range_m: float,
    side_sign: int | None = None,
    direction_epsilon: float = 1e-12,
) -> tuple[RangePositionLine2D, ...]:
    """
    Convert a perpendicular range to a known mapped line into the
    possible loci of observer positions.

    The mapped feature is an infinite 2D line:

        mapped_point + t * mapped_direction

    The measurement is the perpendicular distance from the unknown
    observer position to that line.

    With unsigned range and unknown side, two parallel observer-position
    lines are possible. If side_sign is supplied, only that side is
    returned.

    side_sign convention:

        +1 = left side of mapped_line_direction
        -1 = right side of mapped_line_direction

    A zero range produces the mapped line itself and therefore returns
    only one locus with side_sign=0.

    This function deliberately does not interpret an arbitrary sensor-ray
    distance as perpendicular wall distance. The caller must supply a
    measurement that legitimately represents perpendicular distance to
    the mapped line.
    """

    values = (
        mapped_line_point_x_m,
        mapped_line_point_y_m,
        mapped_line_direction_x,
        mapped_line_direction_y,
        range_m,
        direction_epsilon,
    )

    if not all(
        math.isfinite(float(value))
        for value in values
    ):
        raise ValueError(
            "range-to-line geometry must be finite"
        )

    mapped_line_point_x_m = float(
        mapped_line_point_x_m
    )

    mapped_line_point_y_m = float(
        mapped_line_point_y_m
    )

    mapped_line_direction_x = float(
        mapped_line_direction_x
    )

    mapped_line_direction_y = float(
        mapped_line_direction_y
    )

    range_m = float(
        range_m
    )

    direction_epsilon = float(
        direction_epsilon
    )

    if range_m < 0.0:
        raise ValueError(
            "range_m must be >= 0"
        )

    if direction_epsilon <= 0.0:
        raise ValueError(
            "direction_epsilon must be > 0"
        )

    if side_sign not in (
        None,
        -1,
        1,
    ):
        raise ValueError(
            "side_sign must be None, -1, or +1"
        )

    direction_norm = math.hypot(
        mapped_line_direction_x,
        mapped_line_direction_y,
    )

    if direction_norm <= direction_epsilon:
        raise ValueError(
            "mapped line direction must be non-zero"
        )

    direction_x = (
        mapped_line_direction_x
        / direction_norm
    )

    direction_y = (
        mapped_line_direction_y
        / direction_norm
    )

    # Left unit normal to the mapped-line direction.
    normal_x = -direction_y
    normal_y = direction_x

    mapped_offset_m = (
        normal_x * mapped_line_point_x_m
        + normal_y * mapped_line_point_y_m
    )

    if range_m <= direction_epsilon:
        return (
            RangePositionLine2D(
                normal_x=normal_x,
                normal_y=normal_y,
                offset_m=mapped_offset_m,
                side_sign=0,
            ),
        )

    if side_sign is not None:
        return (
            RangePositionLine2D(
                normal_x=normal_x,
                normal_y=normal_y,
                offset_m=(
                    mapped_offset_m
                    + side_sign * range_m
                ),
                side_sign=side_sign,
            ),
        )

    return (
        RangePositionLine2D(
            normal_x=normal_x,
            normal_y=normal_y,
            offset_m=(
                mapped_offset_m
                + range_m
            ),
            side_sign=1,
        ),
        RangePositionLine2D(
            normal_x=normal_x,
            normal_y=normal_y,
            offset_m=(
                mapped_offset_m
                - range_m
            ),
            side_sign=-1,
        ),
    )


def intersect_range_position_lines_2d(
    line_a: RangePositionLine2D,
    line_b: RangePositionLine2D,
    *,
    parallel_epsilon: float = 1e-12,
) -> RangeLineIntersectionPoint2D | None:
    """
    Intersect two observer-position loci.

    Returns None when the two loci are parallel or coincident.
    """

    parallel_epsilon = float(
        parallel_epsilon
    )

    if (
        not math.isfinite(parallel_epsilon)
        or parallel_epsilon <= 0.0
    ):
        raise ValueError(
            "parallel_epsilon must be finite and > 0"
        )

    values = (
        line_a.normal_x,
        line_a.normal_y,
        line_a.offset_m,
        line_b.normal_x,
        line_b.normal_y,
        line_b.offset_m,
    )

    if not all(
        math.isfinite(float(value))
        for value in values
    ):
        raise ValueError(
            "position-line geometry must be finite"
        )

    determinant = (
        line_a.normal_x
        * line_b.normal_y
        - line_a.normal_y
        * line_b.normal_x
    )

    if abs(determinant) <= parallel_epsilon:
        return None

    x_m = (
        line_a.offset_m
        * line_b.normal_y
        - line_a.normal_y
        * line_b.offset_m
    ) / determinant

    y_m = (
        line_a.normal_x
        * line_b.offset_m
        - line_a.offset_m
        * line_b.normal_x
    ) / determinant

    return RangeLineIntersectionPoint2D(
        x_m=x_m,
        y_m=y_m,
    )


__all__ = [
    "RangePositionLine2D",
    "RangeLineIntersectionPoint2D",
    "range_to_lines_2d",
    "intersect_range_position_lines_2d",
]
