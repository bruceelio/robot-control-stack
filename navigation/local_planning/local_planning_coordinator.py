# navigation/local_planning_coordinator

"""
Composition layer for localisation-independent local planning.

The coordinator is intentionally thin:

- optional Follow-the-Gap selects a local free-space direction;
- DWA generates and scores dynamically reachable, statically safe velocities;
- optional Velocity Obstacles remove DWA candidates predicted to collide with
  tracked moving obstacles;
- the best remaining velocity is returned through the shared planning result.

The coordinator performs no sensor fusion, object classification, global
localisation, or algorithm-specific mathematics.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .dynamic_window import (
    DynamicWindowConfig,
    VelocityCandidate,
    evaluate_velocity_candidates,
)
from .follow_the_gap import FollowTheGapConfig, find_best_gap
from .models import (
    LocalPlanningRequest,
    LocalPlanningResult,
    LocalPlanningStatus,
    PreferredDirection,
)
from .velocity_obstacle import (
    VelocityObstacleAssessment,
    VelocityObstacleConfig,
    assess_velocity,
    average_velocity_from_rollout,
)


@dataclass(frozen=True)
class LocalPlanningCoordinatorConfig:
    """
    Enabled Stage-2 local-planning composition.

    DWA is the final velocity selector for this coordinator.

    `follow_the_gap=None`
        Use the request's preferred direction directly.

    `follow_the_gap=<config>`
        Run FTG first and use its selected free-space bearing as DWA's
        preferred direction.

    `velocity_obstacle=None`
        Do not apply moving-obstacle prediction.

    `velocity_obstacle=<config>`
        Reject DWA candidates that enter a finite-horizon velocity obstacle.
    """

    dynamic_window: DynamicWindowConfig
    follow_the_gap: Optional[FollowTheGapConfig] = None
    velocity_obstacle: Optional[VelocityObstacleConfig] = None


@dataclass(frozen=True)
class LocalPlanningDiagnostics:
    """Optional detail useful for logs/tests without changing command output."""

    ftg_used: bool
    vo_used: bool
    dwa_candidates: int
    vo_rejected_candidates: int
    selected_vo_assessment: Optional[VelocityObstacleAssessment] = None


@dataclass(frozen=True)
class CoordinatedLocalPlanningResult:
    """Planner result plus composition diagnostics."""

    result: LocalPlanningResult
    diagnostics: LocalPlanningDiagnostics


def plan_local_motion(
    request: LocalPlanningRequest,
    config: LocalPlanningCoordinatorConfig,
) -> CoordinatedLocalPlanningResult:
    """
    Run the enabled local-planning composition for one control update.

    No arena/global pose is required.

    Returns zero commanded motion with a non-OK status when FTG cannot find a
    gap or when no DWA candidate survives static/dynamic collision constraints.
    """

    preferred_direction = request.preferred_direction
    selected_gap = None

    if config.follow_the_gap is not None:
        selected_gap = find_best_gap(
            obstacle_field=request.obstacle_field,
            preferred_direction=request.preferred_direction,
            collision_geometry=request.collision_geometry,
            config=config.follow_the_gap,
        )

        if selected_gap is None:
            return CoordinatedLocalPlanningResult(
                result=LocalPlanningResult(
                    status=LocalPlanningStatus.NO_GAP,
                    linear_mm_s=0.0,
                    angular_rad_s=0.0,
                    reason="follow_the_gap found no traversable local gap",
                ),
                diagnostics=LocalPlanningDiagnostics(
                    ftg_used=True,
                    vo_used=config.velocity_obstacle is not None,
                    dwa_candidates=0,
                    vo_rejected_candidates=0,
                ),
            )

        preferred_direction = PreferredDirection(
            bearing_rad=selected_gap.selected_bearing_rad,
            weight=request.preferred_direction.weight,
        )

    dwa = evaluate_velocity_candidates(
        obstacle_field=request.obstacle_field,
        preferred_direction=preferred_direction,
        motion=request.motion,
        dynamic_limits=request.dynamic_limits,
        collision_geometry=request.collision_geometry,
        config=config.dynamic_window,
    )

    if not dwa.candidates:
        return CoordinatedLocalPlanningResult(
            result=LocalPlanningResult(
                status=LocalPlanningStatus.NO_SAFE_VELOCITY,
                linear_mm_s=0.0,
                angular_rad_s=0.0,
                selected_bearing_rad=preferred_direction.bearing_rad,
                selected_gap=selected_gap,
                reason="DWA found no statically admissible velocity",
            ),
            diagnostics=LocalPlanningDiagnostics(
                ftg_used=config.follow_the_gap is not None,
                vo_used=config.velocity_obstacle is not None,
                dwa_candidates=0,
                vo_rejected_candidates=0,
            ),
        )

    selected_candidate: Optional[VelocityCandidate] = None
    selected_vo_assessment: Optional[VelocityObstacleAssessment] = None
    vo_rejected = 0

    if (
        config.velocity_obstacle is not None
        and request.tracked_obstacles
    ):
        for candidate in dwa.candidates:
            candidate_velocity = average_velocity_from_rollout(
                final_x_mm=candidate.final_x_mm,
                final_y_mm=candidate.final_y_mm,
                horizon_s=config.dynamic_window.prediction_horizon_s,
            )

            assessment = assess_velocity(
                candidate_velocity=candidate_velocity,
                current_motion=request.motion,
                tracked_obstacles=request.tracked_obstacles,
                collision_geometry=request.collision_geometry,
                config=config.velocity_obstacle,
            )

            if assessment.safe:
                selected_candidate = candidate
                selected_vo_assessment = assessment
                break

            vo_rejected += 1
    else:
        selected_candidate = dwa.candidates[0]

    if selected_candidate is None:
        return CoordinatedLocalPlanningResult(
            result=LocalPlanningResult(
                status=LocalPlanningStatus.NO_SAFE_VELOCITY,
                linear_mm_s=0.0,
                angular_rad_s=0.0,
                selected_bearing_rad=preferred_direction.bearing_rad,
                selected_gap=selected_gap,
                reason="all DWA candidates rejected by Velocity Obstacles",
            ),
            diagnostics=LocalPlanningDiagnostics(
                ftg_used=config.follow_the_gap is not None,
                vo_used=config.velocity_obstacle is not None,
                dwa_candidates=len(dwa.candidates),
                vo_rejected_candidates=vo_rejected,
            ),
        )

    return CoordinatedLocalPlanningResult(
        result=LocalPlanningResult(
            status=LocalPlanningStatus.OK,
            linear_mm_s=selected_candidate.linear_mm_s,
            angular_rad_s=selected_candidate.angular_rad_s,
            selected_bearing_rad=preferred_direction.bearing_rad,
            selected_gap=selected_gap,
            reason="local planning succeeded",
        ),
        diagnostics=LocalPlanningDiagnostics(
            ftg_used=config.follow_the_gap is not None,
            vo_used=config.velocity_obstacle is not None,
            dwa_candidates=len(dwa.candidates),
            vo_rejected_candidates=vo_rejected,
            selected_vo_assessment=selected_vo_assessment,
        ),
    )
