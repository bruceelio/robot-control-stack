# localisation/geometry/bearing_resection_2d.py

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class BearingObservation2D:
    """
    One bearing observation to a known landmark.

    landmark_x_m / landmark_y_m:
        Known landmark position in a common Cartesian frame.

    bearing_rad:
        Bearing from the observer's forward axis to the landmark.
        Positive is counter-clockwise.

    Units:
        landmark position = metres
        bearing           = radians
    """

    landmark_x_m: float
    landmark_y_m: float
    bearing_rad: float


@dataclass(frozen=True)
class BearingResectionCandidate2D:
    """
    One candidate observer pose consistent with landmark bearings.

    residuals_rad contains, for each observation:

        predicted_relative_bearing - measured_bearing

    wrapped to [-pi, pi].
    """

    x_m: float
    y_m: float
    heading_rad: float

    residuals_rad: tuple[float, ...]
    rms_residual_rad: float
    max_abs_residual_rad: float

    iterations: int
    converged: bool


def bearing_resection_2d(
    observations: Iterable[BearingObservation2D],
    *,
    heading_seed_count: int = 16,
    max_iterations: int = 30,
    convergence_position_tolerance_m: float = 1e-6,
    convergence_heading_tolerance_rad: float = 1e-7,
    matrix_epsilon: float = 1e-12,
    duplicate_position_tolerance_m: float = 1e-4,
    duplicate_heading_tolerance_rad: float = 1e-4,
) -> tuple[BearingResectionCandidate2D, ...]:
    """
    Recover candidate 2D observer poses from bearings to known landmarks.

    At least three landmark bearings are required because the unknowns are:

        observer x
        observer y
        observer heading

    The solver does not require heading to be known beforehand.

    It uses several heading seeds. For each seed it first estimates position
    from the corresponding landmark-bearing lines, then refines x, y and
    heading together with Gauss-Newton least squares.

    Multiple candidates may be returned because bearing-only resection can
    be geometrically ambiguous. Results are sorted from lowest to highest
    RMS angular residual. Near-duplicate solutions are collapsed.

    The function performs geometry only. It does not apply arena bounds,
    prior-pose gating, confidence policy, timestamps, or sensor semantics.
    """

    observations = tuple(
        _normalise_observation(observation)
        for observation in observations
    )

    if len(observations) < 3:
        raise ValueError(
            "bearing resection requires at least "
            "three observations"
        )

    unique_landmarks = {
        (
            observation.landmark_x_m,
            observation.landmark_y_m,
        )
        for observation in observations
    }

    if len(unique_landmarks) < 3:
        raise ValueError(
            "bearing resection requires at least "
            "three distinct landmarks"
        )

    heading_seed_count = int(
        heading_seed_count
    )

    max_iterations = int(
        max_iterations
    )

    if heading_seed_count < 4:
        raise ValueError(
            "heading_seed_count must be >= 4"
        )

    if max_iterations <= 0:
        raise ValueError(
            "max_iterations must be > 0"
        )

    positive_parameters = (
        (
            "convergence_position_tolerance_m",
            convergence_position_tolerance_m,
        ),
        (
            "convergence_heading_tolerance_rad",
            convergence_heading_tolerance_rad,
        ),
        (
            "matrix_epsilon",
            matrix_epsilon,
        ),
        (
            "duplicate_position_tolerance_m",
            duplicate_position_tolerance_m,
        ),
        (
            "duplicate_heading_tolerance_rad",
            duplicate_heading_tolerance_rad,
        ),
    )

    for name, value in positive_parameters:
        value = float(value)

        if (
            not math.isfinite(value)
            or value <= 0.0
        ):
            raise ValueError(
                f"{name} must be finite and > 0"
            )

    convergence_position_tolerance_m = float(
        convergence_position_tolerance_m
    )

    convergence_heading_tolerance_rad = float(
        convergence_heading_tolerance_rad
    )

    matrix_epsilon = float(
        matrix_epsilon
    )

    duplicate_position_tolerance_m = float(
        duplicate_position_tolerance_m
    )

    duplicate_heading_tolerance_rad = float(
        duplicate_heading_tolerance_rad
    )

    candidates: list[
        BearingResectionCandidate2D
    ] = []

    for seed_index in range(
        heading_seed_count
    ):
        heading_seed_rad = (
            -math.pi
            + (2.0 * math.pi)
            * seed_index
            / heading_seed_count
        )

        try:
            x_m, y_m = _position_for_heading(
                observations,
                heading_rad=heading_seed_rad,
                matrix_epsilon=matrix_epsilon,
            )

            candidate = _refine_candidate(
                observations,
                x_m=x_m,
                y_m=y_m,
                heading_rad=heading_seed_rad,
                max_iterations=max_iterations,
                convergence_position_tolerance_m=(
                    convergence_position_tolerance_m
                ),
                convergence_heading_tolerance_rad=(
                    convergence_heading_tolerance_rad
                ),
                matrix_epsilon=matrix_epsilon,
            )

        except ValueError:
            continue

        if not _is_duplicate_candidate(
            candidate,
            candidates,
            position_tolerance_m=(
                duplicate_position_tolerance_m
            ),
            heading_tolerance_rad=(
                duplicate_heading_tolerance_rad
            ),
        ):
            candidates.append(
                candidate
            )

    if not candidates:
        raise ValueError(
            "bearing geometry produced no finite "
            "candidate pose"
        )

    candidates.sort(
        key=lambda candidate: (
            candidate.rms_residual_rad,
            candidate.max_abs_residual_rad,
        )
    )

    return tuple(candidates)


def _normalise_observation(
    observation: BearingObservation2D,
) -> BearingObservation2D:

    values = (
        observation.landmark_x_m,
        observation.landmark_y_m,
        observation.bearing_rad,
    )

    if not all(
        math.isfinite(float(value))
        for value in values
    ):
        raise ValueError(
            "bearing observation values must be finite"
        )

    return BearingObservation2D(
        landmark_x_m=float(
            observation.landmark_x_m
        ),
        landmark_y_m=float(
            observation.landmark_y_m
        ),
        bearing_rad=_wrap_angle_rad(
            float(observation.bearing_rad)
        ),
    )


def _position_for_heading(
    observations: tuple[BearingObservation2D, ...],
    *,
    heading_rad: float,
    matrix_epsilon: float,
) -> tuple[float, float]:
    """
    Estimate observer position for one assumed heading.

    Each measured bearing then defines an infinite line through the
    corresponding landmark and the unknown observer. Least squares finds
    the point best satisfying all of those lines.
    """

    ata_xx = 0.0
    ata_xy = 0.0
    ata_yy = 0.0

    atb_x = 0.0
    atb_y = 0.0

    for observation in observations:
        absolute_bearing_rad = (
            heading_rad
            + observation.bearing_rad
        )

        direction_x = math.cos(
            absolute_bearing_rad
        )

        direction_y = math.sin(
            absolute_bearing_rad
        )

        # Unit normal to the observer -> landmark line.
        normal_x = -direction_y
        normal_y = direction_x

        rhs = (
            normal_x
            * observation.landmark_x_m
            + normal_y
            * observation.landmark_y_m
        )

        ata_xx += normal_x * normal_x
        ata_xy += normal_x * normal_y
        ata_yy += normal_y * normal_y

        atb_x += normal_x * rhs
        atb_y += normal_y * rhs

    return _solve_2x2(
        a=ata_xx,
        b=ata_xy,
        c=ata_xy,
        d=ata_yy,
        rhs_x=atb_x,
        rhs_y=atb_y,
        matrix_epsilon=matrix_epsilon,
        error_message=(
            "bearing lines do not constrain a unique position"
        ),
    )


def _refine_candidate(
    observations: tuple[BearingObservation2D, ...],
    *,
    x_m: float,
    y_m: float,
    heading_rad: float,
    max_iterations: int,
    convergence_position_tolerance_m: float,
    convergence_heading_tolerance_rad: float,
    matrix_epsilon: float,
) -> BearingResectionCandidate2D:

    heading_rad = _wrap_angle_rad(
        heading_rad
    )

    converged = False
    iterations = 0

    for iteration in range(
        1,
        max_iterations + 1,
    ):
        iterations = iteration

        hessian = [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ]

        gradient = [
            0.0,
            0.0,
            0.0,
        ]

        usable_observations = 0

        for observation in observations:
            delta_x_m = (
                observation.landmark_x_m
                - x_m
            )

            delta_y_m = (
                observation.landmark_y_m
                - y_m
            )

            range_sq_m2 = (
                delta_x_m * delta_x_m
                + delta_y_m * delta_y_m
            )

            if range_sq_m2 <= 1e-18:
                continue

            usable_observations += 1

            predicted_bearing_rad = (
                math.atan2(
                    delta_y_m,
                    delta_x_m,
                )
                - heading_rad
            )

            residual_rad = _wrap_angle_rad(
                predicted_bearing_rad
                - observation.bearing_rad
            )

            jacobian = (
                delta_y_m / range_sq_m2,
                -delta_x_m / range_sq_m2,
                -1.0,
            )

            for row in range(3):
                gradient[row] += (
                    jacobian[row]
                    * residual_rad
                )

                for column in range(3):
                    hessian[row][column] += (
                        jacobian[row]
                        * jacobian[column]
                    )

        if usable_observations < 3:
            raise ValueError(
                "bearing geometry does not provide enough "
                "usable observations"
            )

        step = _solve_3x3(
            matrix=hessian,
            rhs=tuple(
                -value
                for value in gradient
            ),
            matrix_epsilon=matrix_epsilon,
        )

        step_x_m = step[0]
        step_y_m = step[1]
        step_heading_rad = step[2]

        x_m += step_x_m
        y_m += step_y_m

        heading_rad = _wrap_angle_rad(
            heading_rad
            + step_heading_rad
        )

        if (
            math.hypot(
                step_x_m,
                step_y_m,
            )
            <= convergence_position_tolerance_m
            and abs(step_heading_rad)
            <= convergence_heading_tolerance_rad
        ):
            converged = True
            break

    residuals_rad = tuple(
        _bearing_residual(
            observation,
            x_m=x_m,
            y_m=y_m,
            heading_rad=heading_rad,
        )
        for observation in observations
    )

    rms_residual_rad = math.sqrt(
        sum(
            residual_rad * residual_rad
            for residual_rad in residuals_rad
        )
        / len(residuals_rad)
    )

    max_abs_residual_rad = max(
        abs(residual_rad)
        for residual_rad in residuals_rad
    )

    if not all(
        math.isfinite(value)
        for value in (
            x_m,
            y_m,
            heading_rad,
            rms_residual_rad,
            max_abs_residual_rad,
        )
    ):
        raise ValueError(
            "bearing solver produced non-finite geometry"
        )

    return BearingResectionCandidate2D(
        x_m=x_m,
        y_m=y_m,
        heading_rad=heading_rad,
        residuals_rad=residuals_rad,
        rms_residual_rad=rms_residual_rad,
        max_abs_residual_rad=max_abs_residual_rad,
        iterations=iterations,
        converged=converged,
    )


def _bearing_residual(
    observation: BearingObservation2D,
    *,
    x_m: float,
    y_m: float,
    heading_rad: float,
) -> float:

    delta_x_m = (
        observation.landmark_x_m
        - x_m
    )

    delta_y_m = (
        observation.landmark_y_m
        - y_m
    )

    if (
        delta_x_m * delta_x_m
        + delta_y_m * delta_y_m
    ) <= 1e-18:
        raise ValueError(
            "observer position coincides with a landmark"
        )

    predicted_bearing_rad = (
        math.atan2(
            delta_y_m,
            delta_x_m,
        )
        - heading_rad
    )

    return _wrap_angle_rad(
        predicted_bearing_rad
        - observation.bearing_rad
    )


def _is_duplicate_candidate(
    candidate: BearingResectionCandidate2D,
    existing: list[BearingResectionCandidate2D],
    *,
    position_tolerance_m: float,
    heading_tolerance_rad: float,
) -> bool:

    for other in existing:
        if (
            math.hypot(
                candidate.x_m - other.x_m,
                candidate.y_m - other.y_m,
            )
            <= position_tolerance_m
            and abs(
                _wrap_angle_rad(
                    candidate.heading_rad
                    - other.heading_rad
                )
            )
            <= heading_tolerance_rad
        ):
            return True

    return False


def _solve_2x2(
    *,
    a: float,
    b: float,
    c: float,
    d: float,
    rhs_x: float,
    rhs_y: float,
    matrix_epsilon: float,
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
        <= matrix_epsilon * scale
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


def _solve_3x3(
    *,
    matrix: list[list[float]],
    rhs: tuple[float, float, float],
    matrix_epsilon: float,
) -> tuple[float, float, float]:
    """
    Solve a 3x3 linear system with partial-pivot Gaussian elimination.
    """

    augmented = [
        [
            float(matrix[row][0]),
            float(matrix[row][1]),
            float(matrix[row][2]),
            float(rhs[row]),
        ]
        for row in range(3)
    ]

    scale = max(
        1.0,
        max(
            abs(augmented[row][column])
            for row in range(3)
            for column in range(3)
        ),
    )

    pivot_threshold = (
        matrix_epsilon
        * scale
    )

    for column in range(3):
        pivot_row = max(
            range(column, 3),
            key=lambda row: abs(
                augmented[row][column]
            ),
        )

        if abs(
            augmented[pivot_row][column]
        ) <= pivot_threshold:
            raise ValueError(
                "bearing geometry is degenerate"
            )

        if pivot_row != column:
            augmented[column], augmented[pivot_row] = (
                augmented[pivot_row],
                augmented[column],
            )

        pivot = augmented[column][column]

        for value_index in range(
            column,
            4,
        ):
            augmented[column][value_index] /= pivot

        for row in range(3):
            if row == column:
                continue

            factor = augmented[row][column]

            for value_index in range(
                column,
                4,
            ):
                augmented[row][value_index] -= (
                    factor
                    * augmented[column][value_index]
                )

    return (
        augmented[0][3],
        augmented[1][3],
        augmented[2][3],
    )


def _wrap_angle_rad(
    angle_rad: float,
) -> float:
    return math.atan2(
        math.sin(angle_rad),
        math.cos(angle_rad),
    )


__all__ = [
    "BearingObservation2D",
    "BearingResectionCandidate2D",
    "bearing_resection_2d",
]
