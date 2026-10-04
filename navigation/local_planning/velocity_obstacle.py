# navigation/velocity_obstacle.py

"""
Localisation-independent Velocity Obstacle (VO) utilities.

This module predicts collisions with moving obstacles using only robot-relative
tracking information.  It does not require arena/global localisation, a map,
or any particular perception sensor.

The implementation uses the finite-horizon constant-velocity form of the
Velocity Obstacle idea:

    obstacle relative position
    + obstacle estimated velocity
    + candidate robot translational velocity
    -> future relative separation / time-to-collision

For differential-drive DWA trajectories, a caller may evaluate the average
translational velocity of a short candidate rollout.  The local-planning
coordinator will own that composition; this module remains generic 2-D VO.

Conventions
-----------
- Distances are millimetres.
- Linear velocities are millimetres per second.
- Robot frame: +x forward, +y left.
- `TrackedObstacle.vx_mm_s` / `vy_mm_s` are interpreted as obstacle velocity
  relative to the robot's CURRENT translational motion, as defined in
  local_planning.models.
- No global x/y/theta is used.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, inf, isfinite, sqrt
from typing import Iterable, Optional, Sequence, Tuple

from .models import (
    RobotCollisionGeometry,
    RobotMotionState,
    TrackedObstacle,
)


_EPS = 1e-9


@dataclass(frozen=True)
class VelocityObstacleConfig:
    """
    Configuration for finite-horizon Velocity Obstacles.

    `prediction_horizon_s`
        Only collisions predicted within this future horizon are considered
        forbidden.  This keeps VO local and avoids overreacting to remote
        trajectory intersections.

    `additional_margin_mm`
        Extra dynamic-obstacle separation added on top of the robot collision
        geometry and tracked-obstacle radius.

    `min_confidence`
        Tracked obstacles below this confidence are ignored by VO.  Perception
        owns object association and confidence generation.
    """

    prediction_horizon_s: float = 1.5
    additional_margin_mm: float = 0.0
    min_confidence: float = 0.0


@dataclass(frozen=True)
class CandidateVelocity2D:
    """
    Candidate robot translational velocity in the current robot frame.

    A nonholonomic candidate may be represented by its average translational
    velocity over a short DWA rollout.
    """

    vx_mm_s: float
    vy_mm_s: float


@dataclass(frozen=True)
class ObstacleCollisionPrediction:
    """VO prediction for one tracked obstacle."""

    track_id: Optional[str]
    forbidden: bool
    time_to_collision_s: Optional[float]
    closest_approach_time_s: float
    closest_separation_mm: float
    combined_radius_mm: float


@dataclass(frozen=True)
class VelocityObstacleAssessment:
    """Aggregate VO result for one candidate robot velocity."""

    safe: bool
    candidate_velocity: CandidateVelocity2D
    predictions: Tuple[ObstacleCollisionPrediction, ...]
    earliest_collision_s: Optional[float]
    minimum_separation_mm: float

    @property
    def conflicting_track_ids(self) -> Tuple[str, ...]:
        return tuple(
            prediction.track_id
            for prediction in self.predictions
            if prediction.forbidden and prediction.track_id is not None
        )


def assess_velocity(
    candidate_velocity: CandidateVelocity2D,
    current_motion: RobotMotionState,
    tracked_obstacles: Sequence[TrackedObstacle],
    collision_geometry: RobotCollisionGeometry,
    config: VelocityObstacleConfig = VelocityObstacleConfig(),
) -> VelocityObstacleAssessment:
    """
    Evaluate one candidate translational velocity against moving obstacles.

    The tracked obstacle velocity is robot-relative to the CURRENT robot
    motion.  VO first reconstructs the obstacle's local inertial translational
    velocity:

        v_obstacle = v_robot_current + v_obstacle_relative

    It then evaluates the candidate robot velocity against that estimated
    obstacle velocity.

    This is intentionally a short-horizon local approximation.  It does not
    assume a persistent global frame and does not predict obstacle intent.
    """

    _validate_inputs(
        candidate_velocity=candidate_velocity,
        current_motion=current_motion,
        collision_geometry=collision_geometry,
        config=config,
    )

    predictions = []

    current_robot_vx = current_motion.linear_mm_s
    current_robot_vy = 0.0

    for obstacle in tracked_obstacles:
        if not _valid_obstacle(obstacle, config.min_confidence):
            continue

        obstacle_vx = current_robot_vx + obstacle.vx_mm_s
        obstacle_vy = current_robot_vy + obstacle.vy_mm_s

        relative_vx = obstacle_vx - candidate_velocity.vx_mm_s
        relative_vy = obstacle_vy - candidate_velocity.vy_mm_s

        combined_radius_mm = (
            collision_geometry.inflated_radius_mm
            + obstacle.radius_mm
            + config.additional_margin_mm
        )

        predictions.append(
            _predict_collision(
                obstacle=obstacle,
                relative_vx_mm_s=relative_vx,
                relative_vy_mm_s=relative_vy,
                combined_radius_mm=combined_radius_mm,
                horizon_s=config.prediction_horizon_s,
            )
        )

    forbidden_predictions = [
        prediction for prediction in predictions if prediction.forbidden
    ]

    collision_times = [
        prediction.time_to_collision_s
        for prediction in forbidden_predictions
        if prediction.time_to_collision_s is not None
    ]

    earliest_collision_s = min(collision_times) if collision_times else None

    minimum_separation_mm = (
        min(prediction.closest_separation_mm for prediction in predictions)
        if predictions
        else inf
    )

    return VelocityObstacleAssessment(
        safe=not forbidden_predictions,
        candidate_velocity=candidate_velocity,
        predictions=tuple(predictions),
        earliest_collision_s=earliest_collision_s,
        minimum_separation_mm=minimum_separation_mm,
    )


def filter_safe_velocities(
    candidates: Iterable[CandidateVelocity2D],
    current_motion: RobotMotionState,
    tracked_obstacles: Sequence[TrackedObstacle],
    collision_geometry: RobotCollisionGeometry,
    config: VelocityObstacleConfig = VelocityObstacleConfig(),
) -> Tuple[CandidateVelocity2D, ...]:
    """Return only candidates outside all finite-horizon velocity obstacles."""

    safe = []

    for candidate in candidates:
        assessment = assess_velocity(
            candidate_velocity=candidate,
            current_motion=current_motion,
            tracked_obstacles=tracked_obstacles,
            collision_geometry=collision_geometry,
            config=config,
        )
        if assessment.safe:
            safe.append(candidate)

    return tuple(safe)


def average_velocity_from_rollout(
    final_x_mm: float,
    final_y_mm: float,
    horizon_s: float,
) -> CandidateVelocity2D:
    """
    Convert a short nonholonomic rollout into an average 2-D velocity.

    This provides a simple bridge from a DWA curved rollout to finite-horizon
    VO without making this module depend on `dynamic_window.py`.
    """

    if not all(isfinite(value) for value in (final_x_mm, final_y_mm, horizon_s)):
        raise ValueError("rollout values must be finite")
    if horizon_s <= 0.0:
        raise ValueError("horizon_s must be > 0")

    return CandidateVelocity2D(
        vx_mm_s=final_x_mm / horizon_s,
        vy_mm_s=final_y_mm / horizon_s,
    )


def _predict_collision(
    obstacle: TrackedObstacle,
    relative_vx_mm_s: float,
    relative_vy_mm_s: float,
    combined_radius_mm: float,
    horizon_s: float,
) -> ObstacleCollisionPrediction:
    """
    Predict collision for constant relative velocity.

    Relative separation evolves as:

        p(t) = p_obstacle + v_relative * t

    A velocity is forbidden when the combined collision discs overlap at any
    time in [0, horizon].
    """

    px = obstacle.x_mm
    py = obstacle.y_mm
    vx = relative_vx_mm_s
    vy = relative_vy_mm_s
    radius = combined_radius_mm

    distance_now = hypot(px, py)

    if distance_now <= radius:
        return ObstacleCollisionPrediction(
            track_id=obstacle.track_id,
            forbidden=True,
            time_to_collision_s=0.0,
            closest_approach_time_s=0.0,
            closest_separation_mm=distance_now,
            combined_radius_mm=radius,
        )

    speed_sq = vx * vx + vy * vy

    if speed_sq <= _EPS:
        return ObstacleCollisionPrediction(
            track_id=obstacle.track_id,
            forbidden=False,
            time_to_collision_s=None,
            closest_approach_time_s=0.0,
            closest_separation_mm=distance_now,
            combined_radius_mm=radius,
        )

    # Time of closest approach on the finite prediction horizon.
    closest_t = -((px * vx) + (py * vy)) / speed_sq
    closest_t = min(max(closest_t, 0.0), horizon_s)

    closest_x = px + vx * closest_t
    closest_y = py + vy * closest_t
    closest_separation = hypot(closest_x, closest_y)

    # Solve |p + vt|^2 = R^2 for first contact.
    a = speed_sq
    b = 2.0 * ((px * vx) + (py * vy))
    c = (px * px) + (py * py) - (radius * radius)

    discriminant = (b * b) - (4.0 * a * c)

    collision_time = None

    if discriminant >= 0.0:
        root = sqrt(max(0.0, discriminant))
        entry_t = (-b - root) / (2.0 * a)
        exit_t = (-b + root) / (2.0 * a)

        if exit_t >= 0.0 and entry_t <= horizon_s:
            collision_time = max(0.0, entry_t)

    forbidden = (
        collision_time is not None
        and collision_time <= horizon_s + _EPS
    )

    return ObstacleCollisionPrediction(
        track_id=obstacle.track_id,
        forbidden=forbidden,
        time_to_collision_s=collision_time if forbidden else None,
        closest_approach_time_s=closest_t,
        closest_separation_mm=closest_separation,
        combined_radius_mm=radius,
    )


def _valid_obstacle(
    obstacle: TrackedObstacle,
    min_confidence: float,
) -> bool:
    values = (
        obstacle.x_mm,
        obstacle.y_mm,
        obstacle.vx_mm_s,
        obstacle.vy_mm_s,
        obstacle.radius_mm,
        obstacle.confidence,
    )

    return (
        all(isfinite(value) for value in values)
        and obstacle.radius_mm >= 0.0
        and min_confidence <= obstacle.confidence <= 1.0
    )


def _validate_inputs(
    candidate_velocity: CandidateVelocity2D,
    current_motion: RobotMotionState,
    collision_geometry: RobotCollisionGeometry,
    config: VelocityObstacleConfig,
) -> None:
    values = (
        candidate_velocity.vx_mm_s,
        candidate_velocity.vy_mm_s,
        current_motion.linear_mm_s,
        current_motion.angular_rad_s,
        collision_geometry.collision_radius_mm,
        collision_geometry.safety_margin_mm,
        config.prediction_horizon_s,
        config.additional_margin_mm,
        config.min_confidence,
    )

    if not all(isfinite(value) for value in values):
        raise ValueError("VO inputs/configuration must be finite")

    if collision_geometry.collision_radius_mm < 0.0:
        raise ValueError("collision_radius_mm must be >= 0")

    if collision_geometry.safety_margin_mm < 0.0:
        raise ValueError("safety_margin_mm must be >= 0")

    if config.prediction_horizon_s <= 0.0:
        raise ValueError("prediction_horizon_s must be > 0")

    if config.additional_margin_mm < 0.0:
        raise ValueError("additional_margin_mm must be >= 0")

    if not 0.0 <= config.min_confidence <= 1.0:
        raise ValueError("min_confidence must be within [0, 1]")
