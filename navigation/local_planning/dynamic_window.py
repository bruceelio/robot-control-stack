# navigation/dynamic_window.py

"""
Localisation-independent Dynamic Window Approach (DWA).

DWA consumes a perception-derived local obstacle field, the robot's current
motion, robot-profile dynamic limits, and a preferred local direction.  It
selects a dynamically reachable and collision-admissible (v, omega) command.

It does not require:
- arena/global localisation;
- a global map;
- a particular camera, ToF, ultrasonic, or LiDAR;
- Follow-the-Gap.

Follow-the-Gap may provide the preferred direction, but DWA remains usable
independently with any robot-relative preferred bearing.

Conventions
-----------
- Distances are millimetres.
- Linear speed is millimetres per second.
- Angles are radians.
- Angular speed is radians per second.
- Robot frame: +x forward, +y left, positive angle counter-clockwise.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, inf, isfinite, pi, sin
from typing import Iterable, Optional, Sequence, Tuple

from .models import (
    LocalObstacleField,
    PolarRangeSample,
    PreferredDirection,
    RobotCollisionGeometry,
    RobotDynamicLimits,
    RobotMotionState,
)


_EPS = 1e-9


@dataclass(frozen=True)
class DynamicWindowConfig:
    """
    DWA configuration.

    `control_dt_s`
        Time over which acceleration limits define the currently reachable
        velocity window.

    `prediction_horizon_s`
        How far each candidate motion is simulated.

    `integration_dt_s`
        Simulation step used for local collision checking.

    `linear_samples` / `angular_samples`
        Number of candidate values sampled across the reachable velocity
        window.

    `heading_weight`
        Preference for ending the rollout aligned with the requested local
        bearing.

    `clearance_weight`
        Preference for trajectories with more obstacle clearance.

    `speed_weight`
        Preference for greater forward speed.

    `clearance_normalization_mm`
        Clearance at or above this value receives the maximum clearance score.

    `braking_margin_mm`
        Extra path distance required beyond the theoretical linear stopping
        distance.

    `min_confidence`
        Perception observations below this confidence are unknown.

    `unknown_is_blocked`
        If True, unknown sectors are not silently treated as clear.

    `unknown_obstacle_distance_mm`
        Conservative range assigned to an unknown sector.  A value of 0 mm is
        fail-safe: any unknown sector overlapping the robot footprint can make
        candidate trajectories inadmissible.  A robot profile may provide a
        less conservative value when perception coverage is understood.
    """

    control_dt_s: float = 0.10
    prediction_horizon_s: float = 1.00
    integration_dt_s: float = 0.05

    linear_samples: int = 7
    angular_samples: int = 15

    heading_weight: float = 0.45
    clearance_weight: float = 0.35
    speed_weight: float = 0.20

    clearance_normalization_mm: float = 1000.0
    braking_margin_mm: float = 50.0

    min_confidence: float = 0.0
    unknown_is_blocked: bool = True
    unknown_obstacle_distance_mm: float = 0.0


@dataclass(frozen=True)
class DynamicWindow:
    """Velocity bounds reachable during the next control interval."""

    linear_min_mm_s: float
    linear_max_mm_s: float
    angular_min_rad_s: float
    angular_max_rad_s: float


@dataclass(frozen=True)
class VelocityCandidate:
    """One evaluated DWA velocity candidate."""

    linear_mm_s: float
    angular_rad_s: float
    score: float
    heading_error_rad: float
    min_clearance_mm: float
    collision_path_distance_mm: float
    stopping_distance_mm: float
    final_x_mm: float
    final_y_mm: float
    final_heading_rad: float


@dataclass(frozen=True)
class DynamicWindowEvaluation:
    """All statically admissible DWA candidates, best score first."""

    candidates: Tuple[VelocityCandidate, ...]
    dynamic_window: DynamicWindow
    evaluated_candidates: int
    admissible_candidates: int


@dataclass(frozen=True)
class DynamicWindowResult:
    """Best admissible velocity selected by DWA."""

    linear_mm_s: float
    angular_rad_s: float
    candidate: VelocityCandidate
    dynamic_window: DynamicWindow
    evaluated_candidates: int
    admissible_candidates: int



@dataclass(frozen=True)
class _ObstacleDisc:
    x_mm: float
    y_mm: float
    radius_mm: float


def evaluate_velocity_candidates(
    obstacle_field: LocalObstacleField,
    preferred_direction: PreferredDirection,
    motion: RobotMotionState,
    dynamic_limits: RobotDynamicLimits,
    collision_geometry: RobotCollisionGeometry,
    config: DynamicWindowConfig = DynamicWindowConfig(),
) -> DynamicWindowEvaluation:
    """
    Evaluate all statically admissible DWA candidates.

    Candidates are returned best-score first.

    This allows a composition layer such as LocalPlanningCoordinator
    to apply additional constraints, including Velocity Obstacles,
    without duplicating DWA candidate-generation logic.
    """

    _validate_inputs(
        preferred_direction=preferred_direction,
        motion=motion,
        dynamic_limits=dynamic_limits,
        collision_geometry=collision_geometry,
        config=config,
    )

    window = calculate_dynamic_window(
        motion=motion,
        dynamic_limits=dynamic_limits,
        control_dt_s=config.control_dt_s,
    )

    obstacles = _build_obstacle_discs(
        samples=obstacle_field.samples,
        min_confidence=config.min_confidence,
        unknown_is_blocked=config.unknown_is_blocked,
        unknown_obstacle_distance_mm=config.unknown_obstacle_distance_mm,
    )

    linear_values = _sample_range(
        window.linear_min_mm_s,
        window.linear_max_mm_s,
        config.linear_samples,
    )

    angular_values = _sample_range(
        window.angular_min_rad_s,
        window.angular_max_rad_s,
        config.angular_samples,
    )

    candidates = []
    evaluated = 0

    for linear_mm_s in linear_values:
        for angular_rad_s in angular_values:
            evaluated += 1

            candidate = _evaluate_candidate(
                linear_mm_s=linear_mm_s,
                angular_rad_s=angular_rad_s,
                preferred_direction=preferred_direction,
                dynamic_limits=dynamic_limits,
                dynamic_window_linear_min_mm_s=window.linear_min_mm_s,
                dynamic_window_linear_max_mm_s=window.linear_max_mm_s,
                collision_geometry=collision_geometry,
                obstacles=obstacles,
                config=config,
            )

            if candidate is not None:
                candidates.append(candidate)

    candidates.sort(
        key=_candidate_sort_key,
        reverse=True,
    )

    return DynamicWindowEvaluation(
        candidates=tuple(candidates),
        dynamic_window=window,
        evaluated_candidates=evaluated,
        admissible_candidates=len(candidates),
    )


def select_velocity(
    obstacle_field: LocalObstacleField,
    preferred_direction: PreferredDirection,
    motion: RobotMotionState,
    dynamic_limits: RobotDynamicLimits,
    collision_geometry: RobotCollisionGeometry,
    config: DynamicWindowConfig = DynamicWindowConfig(),
) -> Optional[DynamicWindowResult]:
    """
    Select the best dynamically reachable and collision-admissible velocity.

    This remains the normal stand-alone DWA API.

    Composition layers which need access to all candidates, such as
    LocalPlanningCoordinator applying Velocity Obstacles, should call
    evaluate_velocity_candidates() instead.
    """

    evaluation = evaluate_velocity_candidates(
        obstacle_field=obstacle_field,
        preferred_direction=preferred_direction,
        motion=motion,
        dynamic_limits=dynamic_limits,
        collision_geometry=collision_geometry,
        config=config,
    )

    if not evaluation.candidates:
        return None

    best = evaluation.candidates[0]

    return DynamicWindowResult(
        linear_mm_s=best.linear_mm_s,
        angular_rad_s=best.angular_rad_s,
        candidate=best,
        dynamic_window=evaluation.dynamic_window,
        evaluated_candidates=evaluation.evaluated_candidates,
        admissible_candidates=evaluation.admissible_candidates,
    )


def calculate_dynamic_window(
    motion: RobotMotionState,
    dynamic_limits: RobotDynamicLimits,
    control_dt_s: float,
) -> DynamicWindow:
    """Return the velocity window reachable during `control_dt_s`."""

    if not isfinite(control_dt_s) or control_dt_s <= 0.0:
        raise ValueError("control_dt_s must be finite and > 0")

    linear_low = max(
        dynamic_limits.linear_min_mm_s,
        motion.linear_mm_s - dynamic_limits.linear_decel_mm_s2 * control_dt_s,
    )
    linear_high = min(
        dynamic_limits.linear_max_mm_s,
        motion.linear_mm_s + dynamic_limits.linear_accel_mm_s2 * control_dt_s,
    )

    angular_low = max(
        -dynamic_limits.angular_max_rad_s,
        motion.angular_rad_s - dynamic_limits.angular_accel_rad_s2 * control_dt_s,
    )
    angular_high = min(
        dynamic_limits.angular_max_rad_s,
        motion.angular_rad_s + dynamic_limits.angular_accel_rad_s2 * control_dt_s,
    )

    if linear_low > linear_high:
        linear_low = linear_high = _clamp(
            motion.linear_mm_s,
            dynamic_limits.linear_min_mm_s,
            dynamic_limits.linear_max_mm_s,
        )

    if angular_low > angular_high:
        angular_low = angular_high = _clamp(
            motion.angular_rad_s,
            -dynamic_limits.angular_max_rad_s,
            dynamic_limits.angular_max_rad_s,
        )

    return DynamicWindow(
        linear_min_mm_s=linear_low,
        linear_max_mm_s=linear_high,
        angular_min_rad_s=angular_low,
        angular_max_rad_s=angular_high,
    )


def _evaluate_candidate(
    linear_mm_s: float,
    angular_rad_s: float,
    preferred_direction: PreferredDirection,
    dynamic_limits: RobotDynamicLimits,
    dynamic_window_linear_min_mm_s: float,
    dynamic_window_linear_max_mm_s: float,
    collision_geometry: RobotCollisionGeometry,
    obstacles: Sequence[_ObstacleDisc],
    config: DynamicWindowConfig,
) -> Optional[VelocityCandidate]:
    (
        final_x_mm,
        final_y_mm,
        final_heading_rad,
        min_clearance_mm,
        collision_path_distance_mm,
    ) = _rollout_candidate(
        linear_mm_s=linear_mm_s,
        angular_rad_s=angular_rad_s,
        obstacles=obstacles,
        collision_geometry=collision_geometry,
        prediction_horizon_s=config.prediction_horizon_s,
        integration_dt_s=config.integration_dt_s,
    )

    stopping_distance_mm = _linear_stopping_distance_mm(
        speed_mm_s=linear_mm_s,
        decel_mm_s2=dynamic_limits.linear_decel_mm_s2,
    )

    if collision_path_distance_mm < inf:
        required_stop_distance = stopping_distance_mm + config.braking_margin_mm
        if required_stop_distance > collision_path_distance_mm + _EPS:
            return None

    if min_clearance_mm <= 0.0:
        return None

    preferred_bearing = preferred_direction.bearing_rad
    heading_error_rad = abs(
        _wrap_angle(final_heading_rad - preferred_bearing)
    )

    heading_score = 1.0 - min(1.0, heading_error_rad / pi)

    if min_clearance_mm == inf:
        clearance_score = 1.0
    else:
        clearance_score = min(
            1.0,
            max(0.0, min_clearance_mm / config.clearance_normalization_mm),
        )

    speed_score = _speed_score(
        linear_mm_s,
        dynamic_window_linear_min_mm_s,
        dynamic_window_linear_max_mm_s,
    )

    score = (
        config.heading_weight * preferred_direction.weight * heading_score
        + config.clearance_weight * clearance_score
        + config.speed_weight * speed_score
    )

    return VelocityCandidate(
        linear_mm_s=linear_mm_s,
        angular_rad_s=angular_rad_s,
        score=score,
        heading_error_rad=heading_error_rad,
        min_clearance_mm=min_clearance_mm,
        collision_path_distance_mm=collision_path_distance_mm,
        stopping_distance_mm=stopping_distance_mm,
        final_x_mm=final_x_mm,
        final_y_mm=final_y_mm,
        final_heading_rad=final_heading_rad,
    )


def _rollout_candidate(
    linear_mm_s: float,
    angular_rad_s: float,
    obstacles: Sequence[_ObstacleDisc],
    collision_geometry: RobotCollisionGeometry,
    prediction_horizon_s: float,
    integration_dt_s: float,
) -> Tuple[float, float, float, float, float]:
    """
    Roll one constant-(v, omega) candidate forward.

    Returns:
        final_x_mm,
        final_y_mm,
        final_heading_rad,
        minimum_clearance_mm,
        path_distance_at_first_collision_mm

    If no collision occurs within the horizon, collision path distance is inf.
    """

    x_mm = 0.0
    y_mm = 0.0
    heading_rad = 0.0

    elapsed_s = 0.0
    travelled_mm = 0.0
    min_clearance_mm = inf
    collision_path_distance_mm = inf

    while elapsed_s < prediction_horizon_s - _EPS:
        dt_s = min(integration_dt_s, prediction_horizon_s - elapsed_s)

        if abs(angular_rad_s) <= _EPS:
            x_mm += linear_mm_s * cos(heading_rad) * dt_s
            y_mm += linear_mm_s * sin(heading_rad) * dt_s
        else:
            new_heading = heading_rad + angular_rad_s * dt_s
            radius_mm = linear_mm_s / angular_rad_s

            x_mm += radius_mm * (sin(new_heading) - sin(heading_rad))
            y_mm -= radius_mm * (cos(new_heading) - cos(heading_rad))
            heading_rad = new_heading

        if abs(angular_rad_s) <= _EPS:
            # Heading is unchanged for straight motion.
            pass
        else:
            heading_rad = _wrap_angle(heading_rad)

        travelled_mm += abs(linear_mm_s) * dt_s
        elapsed_s += dt_s

        clearance_mm = _clearance_at_pose(
            x_mm=x_mm,
            y_mm=y_mm,
            obstacles=obstacles,
            robot_radius_mm=collision_geometry.inflated_radius_mm,
        )
        min_clearance_mm = min(min_clearance_mm, clearance_mm)

        if clearance_mm <= 0.0 and collision_path_distance_mm == inf:
            collision_path_distance_mm = travelled_mm
            break

    return (
        x_mm,
        y_mm,
        heading_rad,
        min_clearance_mm,
        collision_path_distance_mm,
    )


def _build_obstacle_discs(
    samples: Sequence[PolarRangeSample],
    min_confidence: float,
    unknown_is_blocked: bool,
    unknown_obstacle_distance_mm: float,
) -> Tuple[_ObstacleDisc, ...]:
    """
    Convert perception-derived polar samples to local obstacle discs.

    `angular_width_rad` becomes lateral obstacle uncertainty.  This avoids
    pretending that a range sector is an infinitesimal point.

    Invalid / unavailable sectors may be represented conservatively at
    `unknown_obstacle_distance_mm`.
    """

    discs = []

    for sample in samples:
        if not isfinite(sample.bearing_rad):
            continue

        valid = sample.is_valid and sample.confidence >= min_confidence

        if valid:
            assert sample.range_mm is not None
            range_mm = sample.range_mm
        elif unknown_is_blocked:
            range_mm = unknown_obstacle_distance_mm
        else:
            continue

        if not isfinite(range_mm) or range_mm < 0.0:
            continue

        x_mm = range_mm * cos(sample.bearing_rad)
        y_mm = range_mm * sin(sample.bearing_rad)

        angular_width = (
            sample.angular_width_rad
            if isfinite(sample.angular_width_rad) and sample.angular_width_rad > 0.0
            else 0.0
        )

        # Approximate half-width of the observed sector at the measured range.
        # sin() remains bounded for unusually broad sectors.
        support_radius_mm = range_mm * sin(min(pi / 2.0, angular_width / 2.0))

        discs.append(
            _ObstacleDisc(
                x_mm=x_mm,
                y_mm=y_mm,
                radius_mm=max(0.0, support_radius_mm),
            )
        )

    return tuple(discs)


def _clearance_at_pose(
    x_mm: float,
    y_mm: float,
    obstacles: Sequence[_ObstacleDisc],
    robot_radius_mm: float,
) -> float:
    if not obstacles:
        return inf

    return min(
        hypot(obstacle.x_mm - x_mm, obstacle.y_mm - y_mm)
        - robot_radius_mm
        - obstacle.radius_mm
        for obstacle in obstacles
    )


def _linear_stopping_distance_mm(
    speed_mm_s: float,
    decel_mm_s2: float,
) -> float:
    if decel_mm_s2 <= 0.0:
        return inf if abs(speed_mm_s) > _EPS else 0.0

    return (abs(speed_mm_s) ** 2) / (2.0 * decel_mm_s2)


def _speed_score(
    linear_mm_s: float,
    linear_min_mm_s: float,
    linear_max_mm_s: float,
) -> float:
    span = linear_max_mm_s - linear_min_mm_s
    if span <= _EPS:
        return 1.0

    return min(
        1.0,
        max(0.0, (linear_mm_s - linear_min_mm_s) / span),
    )


def _sample_range(
    low: float,
    high: float,
    count: int,
) -> Tuple[float, ...]:
    if count <= 0:
        raise ValueError("sample count must be > 0")

    if count == 1 or abs(high - low) <= _EPS:
        return ((low + high) / 2.0,)

    step = (high - low) / float(count - 1)
    values = tuple(low + step * index for index in range(count))

    # Preserve exact bounds against floating-point accumulation.
    return (low, *values[1:-1], high)


def _candidate_sort_key(candidate: VelocityCandidate) -> Tuple[float, float, float, float]:
    """
    Deterministic tie breaking:
    score -> clearance -> forward speed -> smaller absolute turn rate.
    """

    return (
        candidate.score,
        candidate.min_clearance_mm,
        candidate.linear_mm_s,
        -abs(candidate.angular_rad_s),
    )


def _wrap_angle(angle_rad: float) -> float:
    return (angle_rad + pi) % (2.0 * pi) - pi


def _clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def _validate_inputs(
    preferred_direction: PreferredDirection,
    motion: RobotMotionState,
    dynamic_limits: RobotDynamicLimits,
    collision_geometry: RobotCollisionGeometry,
    config: DynamicWindowConfig,
) -> None:
    finite_values = (
        preferred_direction.bearing_rad,
        preferred_direction.weight,
        motion.linear_mm_s,
        motion.angular_rad_s,
        dynamic_limits.linear_min_mm_s,
        dynamic_limits.linear_max_mm_s,
        dynamic_limits.angular_max_rad_s,
        dynamic_limits.linear_accel_mm_s2,
        dynamic_limits.linear_decel_mm_s2,
        dynamic_limits.angular_accel_rad_s2,
        collision_geometry.collision_radius_mm,
        collision_geometry.safety_margin_mm,
        config.control_dt_s,
        config.prediction_horizon_s,
        config.integration_dt_s,
        config.heading_weight,
        config.clearance_weight,
        config.speed_weight,
        config.clearance_normalization_mm,
        config.braking_margin_mm,
        config.min_confidence,
        config.unknown_obstacle_distance_mm,
    )

    if not all(isfinite(value) for value in finite_values):
        raise ValueError("DWA inputs/configuration must be finite")

    if preferred_direction.weight < 0.0:
        raise ValueError("preferred direction weight must be >= 0")

    if dynamic_limits.linear_min_mm_s > dynamic_limits.linear_max_mm_s:
        raise ValueError("linear_min_mm_s must be <= linear_max_mm_s")

    if dynamic_limits.angular_max_rad_s < 0.0:
        raise ValueError("angular_max_rad_s must be >= 0")

    if dynamic_limits.linear_accel_mm_s2 <= 0.0:
        raise ValueError("linear_accel_mm_s2 must be > 0")

    if dynamic_limits.linear_decel_mm_s2 <= 0.0:
        raise ValueError("linear_decel_mm_s2 must be > 0")

    if dynamic_limits.angular_accel_rad_s2 <= 0.0:
        raise ValueError("angular_accel_rad_s2 must be > 0")

    if collision_geometry.collision_radius_mm < 0.0:
        raise ValueError("collision_radius_mm must be >= 0")

    if collision_geometry.safety_margin_mm < 0.0:
        raise ValueError("safety_margin_mm must be >= 0")

    if config.control_dt_s <= 0.0:
        raise ValueError("control_dt_s must be > 0")

    if config.prediction_horizon_s <= 0.0:
        raise ValueError("prediction_horizon_s must be > 0")

    if config.integration_dt_s <= 0.0:
        raise ValueError("integration_dt_s must be > 0")

    if config.linear_samples <= 0 or config.angular_samples <= 0:
        raise ValueError("DWA sample counts must be > 0")

    if config.heading_weight < 0.0:
        raise ValueError("heading_weight must be >= 0")

    if config.clearance_weight < 0.0:
        raise ValueError("clearance_weight must be >= 0")

    if config.speed_weight < 0.0:
        raise ValueError("speed_weight must be >= 0")

    if (
        config.heading_weight
        + config.clearance_weight
        + config.speed_weight
        <= 0.0
    ):
        raise ValueError("at least one DWA score weight must be > 0")

    if config.clearance_normalization_mm <= 0.0:
        raise ValueError("clearance_normalization_mm must be > 0")

    if config.braking_margin_mm < 0.0:
        raise ValueError("braking_margin_mm must be >= 0")

    if not 0.0 <= config.min_confidence <= 1.0:
        raise ValueError("min_confidence must be within [0, 1]")

    if config.unknown_obstacle_distance_mm < 0.0:
        raise ValueError("unknown_obstacle_distance_mm must be >= 0")
