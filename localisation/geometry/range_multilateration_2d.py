# localisation/geometry/range_multilateration_2d.py

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class RangeConstraint2D:
    """
    One range constraint to a known point.

    All values are expressed in one common Cartesian frame.

    Units:
        anchor position = metres
        range           = metres
    """

    anchor_x_m: float
    anchor_y_m: float
    range_m: float


@dataclass(frozen=True)
class RangeMultilaterationResult2D:
    """
    Best-fit point from three or more range constraints.

    residuals_m contains, for each constraint:

        calculated_range - measured_range
    """

    x_m: float
    y_m: float

    residuals_m: tuple[float, ...]
    rms_residual_m: float
    max_abs_residual_m: float

    iterations: int
    converged: bool


def range_multilateration_2d(
    constraints: Iterable[RangeConstraint2D],
    *,
    max_iterations: int = 20,
    convergence_tolerance_m: float = 1e-6,
    determinant_epsilon: float = 1e-12,
) -> RangeMultilaterationResult2D:
    """
    Estimate one 2D point from three or more known-point ranges.

    The input anchors, ranges and returned point all use metres and
    the same Cartesian coordinate frame.

    Method:

        1. Build an algebraic least-squares seed by subtracting
           one circle equation from the others.

        2. Refine that seed with Gauss-Newton least squares
           against the actual nonlinear range residuals.

    No assumption is made about what the anchors represent. They
    may be mapped landmarks, known objects, beacons, or any other
    fixed points.

    Raises ValueError when the input is invalid or the anchor
    geometry cannot constrain a unique 2D point.
    """

    constraints = tuple(
        _normalise_constraint(constraint)
        for constraint in constraints
    )

    if len(constraints) < 3:
        raise ValueError(
            "range multilateration requires at least "
            "three constraints"
        )

    max_iterations = int(max_iterations)

    if max_iterations <= 0:
        raise ValueError(
            "max_iterations must be > 0"
        )

    convergence_tolerance_m = float(
        convergence_tolerance_m
    )

    determinant_epsilon = float(
        determinant_epsilon
    )

    if (
        not math.isfinite(convergence_tolerance_m)
        or convergence_tolerance_m <= 0.0
    ):
        raise ValueError(
            "convergence_tolerance_m must be finite "
            "and > 0"
        )

    if (
        not math.isfinite(determinant_epsilon)
        or determinant_epsilon <= 0.0
    ):
        raise ValueError(
            "determinant_epsilon must be finite "
            "and > 0"
        )

    x_m, y_m = _initial_estimate(
        constraints,
        determinant_epsilon=determinant_epsilon,
    )

    converged = False
    iterations = 0

    for iteration in range(
        1,
        max_iterations + 1,
    ):
        iterations = iteration

        h_xx = 0.0
        h_xy = 0.0
        h_yy = 0.0

        gradient_x = 0.0
        gradient_y = 0.0

        usable_constraints = 0

        for constraint in constraints:
            delta_x_m = (
                x_m
                - constraint.anchor_x_m
            )

            delta_y_m = (
                y_m
                - constraint.anchor_y_m
            )

            calculated_range_m = math.hypot(
                delta_x_m,
                delta_y_m,
            )

            if calculated_range_m <= 1e-12:
                continue

            usable_constraints += 1

            residual_m = (
                calculated_range_m
                - constraint.range_m
            )

            jacobian_x = (
                delta_x_m
                / calculated_range_m
            )

            jacobian_y = (
                delta_y_m
                / calculated_range_m
            )

            h_xx += jacobian_x * jacobian_x
            h_xy += jacobian_x * jacobian_y
            h_yy += jacobian_y * jacobian_y

            gradient_x += jacobian_x * residual_m
            gradient_y += jacobian_y * residual_m

        if usable_constraints < 2:
            raise ValueError(
                "range geometry does not provide enough "
                "directional information"
            )

        step_x_m, step_y_m = _solve_2x2(
            a=h_xx,
            b=h_xy,
            c=h_xy,
            d=h_yy,
            rhs_x=-gradient_x,
            rhs_y=-gradient_y,
            determinant_epsilon=determinant_epsilon,
            error_message=(
                "range anchor geometry is degenerate"
            ),
        )

        x_m += step_x_m
        y_m += step_y_m

        if math.hypot(
            step_x_m,
            step_y_m,
        ) <= convergence_tolerance_m:
            converged = True
            break

    residuals_m = tuple(
        (
            math.hypot(
                x_m - constraint.anchor_x_m,
                y_m - constraint.anchor_y_m,
            )
            - constraint.range_m
        )
        for constraint in constraints
    )

    rms_residual_m = math.sqrt(
        sum(
            residual_m * residual_m
            for residual_m in residuals_m
        )
        / len(residuals_m)
    )

    max_abs_residual_m = max(
        abs(residual_m)
        for residual_m in residuals_m
    )

    return RangeMultilaterationResult2D(
        x_m=x_m,
        y_m=y_m,
        residuals_m=residuals_m,
        rms_residual_m=rms_residual_m,
        max_abs_residual_m=max_abs_residual_m,
        iterations=iterations,
        converged=converged,
    )


def _normalise_constraint(
    constraint: RangeConstraint2D,
) -> RangeConstraint2D:

    values = (
        constraint.anchor_x_m,
        constraint.anchor_y_m,
        constraint.range_m,
    )

    if not all(
        math.isfinite(float(value))
        for value in values
    ):
        raise ValueError(
            "range constraint values must be finite"
        )

    range_m = float(
        constraint.range_m
    )

    if range_m < 0.0:
        raise ValueError(
            "range_m must be >= 0"
        )

    return RangeConstraint2D(
        anchor_x_m=float(
            constraint.anchor_x_m
        ),
        anchor_y_m=float(
            constraint.anchor_y_m
        ),
        range_m=range_m,
    )


def _initial_estimate(
    constraints: tuple[RangeConstraint2D, ...],
    *,
    determinant_epsilon: float,
) -> tuple[float, float]:
    """
    Obtain an algebraic least-squares seed.

    Subtracting the first circle equation from each remaining
    circle equation cancels the common x^2 + y^2 terms and
    leaves a linear system in x and y.
    """

    reference = constraints[0]

    ata_xx = 0.0
    ata_xy = 0.0
    ata_yy = 0.0

    atb_x = 0.0
    atb_y = 0.0

    for constraint in constraints[1:]:

        row_x = 2.0 * (
            constraint.anchor_x_m
            - reference.anchor_x_m
        )

        row_y = 2.0 * (
            constraint.anchor_y_m
            - reference.anchor_y_m
        )

        rhs = (
            reference.range_m * reference.range_m
            - constraint.range_m * constraint.range_m
            + constraint.anchor_x_m * constraint.anchor_x_m
            - reference.anchor_x_m * reference.anchor_x_m
            + constraint.anchor_y_m * constraint.anchor_y_m
            - reference.anchor_y_m * reference.anchor_y_m
        )

        ata_xx += row_x * row_x
        ata_xy += row_x * row_y
        ata_yy += row_y * row_y

        atb_x += row_x * rhs
        atb_y += row_y * rhs

    return _solve_2x2(
        a=ata_xx,
        b=ata_xy,
        c=ata_xy,
        d=ata_yy,
        rhs_x=atb_x,
        rhs_y=atb_y,
        determinant_epsilon=determinant_epsilon,
        error_message=(
            "range anchors do not constrain a unique "
            "2D position"
        ),
    )


def _solve_2x2(
    *,
    a: float,
    b: float,
    c: float,
    d: float,
    rhs_x: float,
    rhs_y: float,
    determinant_epsilon: float,
    error_message: str,
) -> tuple[float, float]:

    determinant = a * d - b * c

    scale = max(
        1.0,
        abs(a * d),
        abs(b * c),
    )

    if (
        abs(determinant)
        <= determinant_epsilon * scale
    ):
        raise ValueError(
            error_message
        )

    x = (
        rhs_x * d
        - b * rhs_y
    ) / determinant

    y = (
        a * rhs_y
        - rhs_x * c
    ) / determinant

    return x, y


__all__ = [
    "RangeConstraint2D",
    "RangeMultilaterationResult2D",
    "range_multilateration_2d",
]
