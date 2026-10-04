# navigation/control/smooth_pose_selector.py

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum
import math

from navigation.control.smooth_control_law import (
    Pose2D,
    SmoothControlLaw,
)
from navigation.control.smooth_fov_admissibility import (
    SmoothFovPrediction,
    camera_bearing_to_point,
    predict_smooth_fov,
)


_RELAXATION_FRACTIONS = (
    0.0,
    0.50,
    0.75,
    1.0,
)

_EPS = 1e-12
_BOUNDARY_EPS_RAD = 1e-9


class SmoothPoseSelectionMode(Enum):
    EXACT = "exact"
    RELAXED = "relaxed"
    UNAVAILABLE = "unavailable"


@dataclass(frozen=True)
class SmoothPoseSelection:
    """
    Result from Smooth terminal-pose selection.

    heading_offset_rad:
        Selected signed heading offset from the ideal goal.

    max_heading_offset_rad:
        Maximum useful visibility-safe heading relaxation.

    relaxation_fraction:
        One of:
            0.0
            0.50
            0.75
            1.0

        or None when no Smooth pose was admitted.

    tested_fractions:
        Fractions for which a full Smooth FOV prediction was actually run.
    """

    mode: SmoothPoseSelectionMode

    pose: Pose2D | None
    prediction: SmoothFovPrediction | None

    relaxation_fraction: float | None

    heading_offset_rad: float
    max_heading_offset_rad: float
    approach_heading_rad: float

    tested_fractions: tuple[float, ...]

    @property
    def admissible(self) -> bool:
        return self.pose is not None


def _wrap_angle_rad(
    angle_rad: float,
) -> float:
    return math.atan2(
        math.sin(angle_rad),
        math.cos(angle_rad),
    )


def _positive_angle_rad(
    angle_rad: float,
) -> float:
    return (
        float(angle_rad)
        % (2.0 * math.pi)
    )


def _terminal_pose_visible(
    *,
    pose: Pose2D,
    visibility_points: Sequence[
        tuple[float, float]
    ],
    camera_mount: Pose2D,
    safe_half_fov_rad: float,
) -> bool:
    """
    Check terminal visibility only.

    This is cheap geometry. It does NOT forward-simulate Smooth.
    """

    for point_x_m, point_y_m in visibility_points:

        bearing_rad, in_front = (
            camera_bearing_to_point(
                robot_pose=pose,
                camera_mount=camera_mount,
                point_x_m=point_x_m,
                point_y_m=point_y_m,
            )
        )

        if (
            not in_front
            or abs(bearing_rad) > safe_half_fov_rad
        ):
            return False

    return True


def _fov_boundary_headings_for_point(
    *,
    goal_x_m: float,
    goal_y_m: float,
    point_x_m: float,
    point_y_m: float,
    camera_mount: Pose2D,
    boundary_bearing_rad: float,
) -> tuple[float, ...]:
    """
    Calculate the base headings which place one fixed visibility
    point exactly on one horizontal camera-FOV boundary.

    This includes camera x/y/yaw mount offset.

    This is an analytical calculation, not a trial-and-error search.

    Let:

        q =
            fixed feature position
            - fixed base staging position

        m =
            camera translation relative to base_link

        gamma =
            camera yaw relative to base_link

        theta =
            robot/base heading

        b =
            requested camera FOV boundary bearing

    The feature in camera coordinates is:

        u =
            R(-(theta + gamma)) q
            - R(-gamma) m

    Setting:

        atan2(u_y, u_x) = b

    reduces to a scalar sine equation in theta.

    It produces two mathematical roots. One normally corresponds to
    the feature being behind the camera; that root is rejected later.
    """

    gamma = float(
        camera_mount.heading_rad
    )

    cos_gamma = math.cos(gamma)
    sin_gamma = math.sin(gamma)

    dx_m = (
        float(point_x_m)
        - float(goal_x_m)
    )

    dy_m = (
        float(point_y_m)
        - float(goal_y_m)
    )

    # q = R(-gamma) * (point - goal)
    q_x = (
        cos_gamma * dx_m
        + sin_gamma * dy_m
    )

    q_y = (
        -sin_gamma * dx_m
        + cos_gamma * dy_m
    )

    # a = R(-gamma) * camera mount translation
    mount_x_m = float(
        camera_mount.x_m
    )

    mount_y_m = float(
        camera_mount.y_m
    )

    a_x = (
        cos_gamma * mount_x_m
        + sin_gamma * mount_y_m
    )

    a_y = (
        -sin_gamma * mount_x_m
        + cos_gamma * mount_y_m
    )

    q_range_m = math.hypot(
        q_x,
        q_y,
    )

    # If the feature lies exactly at the base staging point,
    # changing robot heading does not create this normal boundary
    # equation.
    if q_range_m <= _EPS:
        return ()

    boundary = float(
        boundary_bearing_rad
    )

    rhs = (
        a_y * math.cos(boundary)
        - a_x * math.sin(boundary)
    ) / q_range_m

    if (
        rhs < -1.0 - _EPS
        or rhs > 1.0 + _EPS
    ):
        return ()

    rhs = max(
        -1.0,
        min(
            1.0,
            rhs,
        ),
    )

    alpha = math.asin(rhs)

    phi = math.atan2(
        q_y,
        q_x,
    )

    heading_a = _wrap_angle_rad(
        phi
        - boundary
        - alpha
    )

    heading_b = _wrap_angle_rad(
        phi
        - boundary
        - math.pi
        + alpha
    )

    return (
        heading_a,
        heading_b,
    )


def _directional_heading_distance_rad(
    *,
    start_heading_rad: float,
    candidate_heading_rad: float,
    direction: float,
) -> float:
    """
    Positive angular travel from start to candidate while moving only
    in the requested direction.
    """

    if direction > 0.0:

        return _positive_angle_rad(
            candidate_heading_rad
            - start_heading_rad
        )

    return _positive_angle_rad(
        start_heading_rad
        - candidate_heading_rad
    )


def _maximum_heading_relaxation_rad(
    *,
    ideal_goal_pose: Pose2D,
    visibility_points: Sequence[
        tuple[float, float]
    ],
    camera_mount: Pose2D,
    safe_half_fov_rad: float,
) -> tuple[float, float]:
    """
    Return:

        (
            approach_heading_rad,
            signed_max_heading_offset_rad,
        )

    The staging x/y position remains fixed.

    The straight-line heading from the CURRENT robot origin to the
    staging position is used only to choose the useful DIRECTION of
    terminal-heading relaxation.

    It does NOT limit the amount of relaxation.

    Once a relaxation direction has been selected, the final heading
    may continue past the straight-line approach heading until the
    first target visibility feature reaches the configured SAFE
    horizontal FOV boundary.

    Therefore:

        0%
            exact requested heading

        100%
            maximum terminal-heading offset in the selected direction
            before target visibility reaches the safe FOV limit

    Intermediate candidates currently tested by select_smooth_pose():

        50%
        75%
        100%

    The first Smooth-admissible candidate is accepted.

    The FOV boundary is solved analytically here. This function does
    not forward-simulate Smooth.
    """

    goal_x_m = float(
        ideal_goal_pose.x_m
    )

    goal_y_m = float(
        ideal_goal_pose.y_m
    )

    ideal_heading_rad = float(
        ideal_goal_pose.heading_rad
    )

    # --------------------------------------------------
    # Select relaxation direction
    # --------------------------------------------------
    #
    # The straight-line approach heading is useful for determining
    # which side of the ideal terminal orientation requires less
    # heading change from the incoming approach.
    #
    # IMPORTANT:
    #     It determines DIRECTION ONLY.
    #
    # It is NOT the 100% relaxation limit.

    approach_heading_rad = math.atan2(
        goal_y_m,
        goal_x_m,
    )

    toward_approach_rad = _wrap_angle_rad(
        approach_heading_rad
        - ideal_heading_rad
    )

    if abs(toward_approach_rad) <= _EPS:
        # There is no preferred relaxation side from the straight-line
        # geometry.
        #
        # A future refinement could select a side from the signed
        # limiting FOV feature, or evaluate both directions.
        return (
            approach_heading_rad,
            0.0,
        )

    direction = (
        1.0
        if toward_approach_rad > 0.0
        else -1.0
    )

    # --------------------------------------------------
    # Find first safe-FOV boundary in that direction
    # --------------------------------------------------

    first_boundary_rad = math.inf

    for point_x_m, point_y_m in visibility_points:

        for boundary_bearing_rad in (
            -safe_half_fov_rad,
            safe_half_fov_rad,
        ):

            roots = (
                _fov_boundary_headings_for_point(
                    goal_x_m=goal_x_m,
                    goal_y_m=goal_y_m,
                    point_x_m=point_x_m,
                    point_y_m=point_y_m,
                    camera_mount=camera_mount,
                    boundary_bearing_rad=(
                        boundary_bearing_rad
                    ),
                )
            )

            for boundary_heading_rad in roots:

                boundary_pose = Pose2D(
                    x_m=goal_x_m,
                    y_m=goal_y_m,
                    heading_rad=(
                        boundary_heading_rad
                    ),
                )

                (
                    actual_bearing_rad,
                    in_front,
                ) = camera_bearing_to_point(
                    robot_pose=boundary_pose,
                    camera_mount=camera_mount,
                    point_x_m=point_x_m,
                    point_y_m=point_y_m,
                )

                # The analytical solution also produces a root with
                # the feature behind the camera. Reject it.
                if not in_front:
                    continue

                if (
                    abs(
                        _wrap_angle_rad(
                            actual_bearing_rad
                            - boundary_bearing_rad
                        )
                    )
                    > 1e-7
                ):
                    continue

                travel_rad = (
                    _directional_heading_distance_rad(
                        start_heading_rad=(
                            ideal_heading_rad
                        ),
                        candidate_heading_rad=(
                            boundary_heading_rad
                        ),
                        direction=direction,
                    )
                )

                if travel_rad <= _BOUNDARY_EPS_RAD:
                    first_boundary_rad = 0.0

                else:
                    first_boundary_rad = min(
                        first_boundary_rad,
                        travel_rad,
                    )

    if not math.isfinite(
        first_boundary_rad
    ):
        # No valid visibility boundary could be established.
        # Fail conservatively rather than inventing an angular limit.
        return (
            approach_heading_rad,
            0.0,
        )

    maximum_relaxation_rad = max(
        0.0,
        first_boundary_rad
        - _BOUNDARY_EPS_RAD,
    )

    return (
        approach_heading_rad,
        direction * maximum_relaxation_rad,
    )

def select_smooth_pose(
    *,
    law: SmoothControlLaw,
    ideal_goal_pose: Pose2D,
    visibility_points: Sequence[
        tuple[float, float]
    ],
    safe_half_fov_rad: float,
    camera_mount: Pose2D = Pose2D(),
    dt_s: float = 0.10,
    max_time_s: float = 5.0,
    position_tolerance_m: float = 0.01,
) -> SmoothPoseSelection:
    """
    Select a visibility-admissible terminal pose for SmoothControlLaw.

    The staging x/y position is NEVER changed.

    Only terminal heading may be relaxed.


    CURRENT SELECTION POLICY
    ========================

    First test the ideal pose:

        0%

    If the exact Smooth trajectory is not FOV-admissible, calculate
    the maximum useful visibility-safe heading relaxation and test:

        50%
        75%
        100%

    The first admissible candidate is accepted.

    If 50% works, for example, we intentionally use 50%. We do NOT
    then spend additional Smooth predictions finding whether 25%,
    37.5%, 43.75%, etc. would also work.

    The subsequent closed-loop visual alignment is expected to remove
    the remaining terminal target bearing.


    WHY THE MAXIMUM IS NOT A SMOOTH SEARCH
    ======================================

    The maximum heading relaxation is calculated from terminal geometry:

    fixed staging x/y
    camera mount
    target visibility-feature positions
    safe horizontal FOV

    The straight-line approach heading is used only to choose the
    relaxation DIRECTION. It does not limit the magnitude of relaxation.

    100% means the terminal heading has been relaxed as far as permitted
    in that direction before the first visibility feature reaches the
    safe FOV boundary.

    The FOV boundary itself is solved analytically.

    Therefore the expensive part remains bounded to at most:

        0%
        50%
        75%
        100%

    = four Smooth forward predictions.


    ALTERNATIVES DELIBERATELY NOT IMPLEMENTED
    ==========================================

    These are documented so future development does not have to restart
    the design discussion from scratch:

        - fixed-degree stepping:
              2 deg, 4 deg, 6 deg, ...

        - binary search for the minimum admissible Smooth offset

        - exhaustive heading search

        - optimisation against predicted FOV margin

        - trying both heading-relaxation directions

        - PBVS fallback inside this selector

    These remain valid future alternatives if real testing shows the
    coarse policy is insufficient.


    FAILURE
    =======

    If none of the four candidates is admissible, return UNAVAILABLE.

    The supervising application then chooses its next strategy.

    Intended current policy:

        HIGH:
            Smooth exact
            -> Smooth relaxed
            -> position-only / target-curvature fallback

        LOW:
            Smooth exact
            -> Smooth relaxed
            -> direct range/bearing visual servo fallback


    COORDINATE CONVENTION
    =====================

        +x       = forward
        +y       = left
        +heading = left / counter-clockwise
        +bearing = left / counter-clockwise

    Units are metres and radians.

    SmoothControlLaw retains its documented Navigation2/OpenNav
    egocentric sign convention internally. That private convention does
    not alter this public positive-left interface.
    """

    values = (
        ideal_goal_pose.x_m,
        ideal_goal_pose.y_m,
        ideal_goal_pose.heading_rad,
        camera_mount.x_m,
        camera_mount.y_m,
        camera_mount.heading_rad,
        safe_half_fov_rad,
        dt_s,
        max_time_s,
        position_tolerance_m,
    )

    if not all(
        math.isfinite(float(value))
        for value in values
    ):
        raise ValueError(
            "Smooth pose selector inputs must be finite"
        )

    safe_half_fov_rad = float(
        safe_half_fov_rad
    )

    if not (
        0.0
        < safe_half_fov_rad
        < math.pi / 2.0
    ):
        raise ValueError(
            "safe_half_fov_rad must be in (0, pi/2)"
        )

    if dt_s <= 0.0:
        raise ValueError(
            "dt_s must be > 0"
        )

    if max_time_s <= 0.0:
        raise ValueError(
            "max_time_s must be > 0"
        )

    if position_tolerance_m < 0.0:
        raise ValueError(
            "position_tolerance_m must be >= 0"
        )

    points = tuple(
        (
            float(point_x_m),
            float(point_y_m),
        )
        for point_x_m, point_y_m
        in visibility_points
    )

    if not points:
        raise ValueError(
            "visibility_points must not be empty"
        )

    if not all(
        math.isfinite(value)
        for point in points
        for value in point
    ):
        raise ValueError(
            "visibility_points must be finite"
        )

    approach_heading_rad = math.atan2(
        ideal_goal_pose.y_m,
        ideal_goal_pose.x_m,
    )

    tested_fractions: list[float] = []

    # --------------------------------------------------
    # Prediction horizon
    # --------------------------------------------------
    #
    # max_time_s is the minimum prediction horizon.
    #
    # A fixed 5-second horizon can incorrectly reject a valid Smooth
    # trajectory simply because the target is far away or curvature
    # reduces translational speed.
    #
    # Allow approximately twice the theoretical straight-line travel
    # time, plus two seconds for terminal slowdown / curvature.

    goal_distance_m = math.hypot(
        ideal_goal_pose.x_m,
        ideal_goal_pose.y_m,
    )

    linear_max_mps = float(
        law.params.linear_max_mps
    )

    if linear_max_mps <= 0.0:
        raise ValueError(
            "Smooth linear_max_mps must be > 0"
        )

    nominal_time_s = (
            goal_distance_m
            / linear_max_mps
    )

    prediction_max_time_s = max(
        float(max_time_s),
        nominal_time_s * 2.0 + 2.0,
    )

    # --------------------------------------------------
    # A. Exact pose: 0%
    #
    # Always try the requested terminal pose first.
    #
    # A trajectory may use a large fraction of the safe FOV and still
    # be perfectly admissible. Heading relaxation is only considered
    # if the exact Smooth trajectory actually fails the FOV prediction.

    exact_prediction = predict_smooth_fov(
        law=law,
        target_pose=ideal_goal_pose,
        visibility_points=points,
        safe_half_fov_rad=(
            safe_half_fov_rad
        ),
        dt_s=dt_s,
        max_time_s=prediction_max_time_s,

        position_tolerance_m=(
            position_tolerance_m
        ),
        camera_mount=camera_mount,
    )

    tested_fractions.append(
        _RELAXATION_FRACTIONS[0]
    )

    print(
        "[SMOOTH_SELECT][EXACT] "
        f"goal_dist={goal_distance_m:.3f}m "
        f"horizon={prediction_max_time_s:.2f}s "
        f"reached={exact_prediction.reached_goal} "
        f"front={exact_prediction.features_stayed_in_front} "
                f"max_abs_bearing="
        f"{math.degrees(exact_prediction.max_abs_bearing_rad):.1f}deg "
        f"peak_bearing="
        f"{math.degrees(exact_prediction.peak_bearing_rad):+.1f}deg "
        f"peak_step={exact_prediction.peak_step} "
        f"peak_time="
        f"{exact_prediction.peak_step * dt_s:.2f}s "
        f"safe="
        f"{math.degrees(safe_half_fov_rad):.1f}deg "
        f"steps={exact_prediction.steps}"
    )

    if exact_prediction.admissible:
        return SmoothPoseSelection(
            mode=SmoothPoseSelectionMode.EXACT,
            pose=ideal_goal_pose,
            prediction=exact_prediction,
            relaxation_fraction=0.0,
            heading_offset_rad=0.0,
            max_heading_offset_rad=0.0,
            approach_heading_rad=(
                approach_heading_rad
            ),
            tested_fractions=tuple(
                tested_fractions
            ),
        )

    # --------------------------------------------------
    # B. Same staging position, relaxed heading
    # --------------------------------------------------

    (
        approach_heading_rad,
        max_heading_offset_rad,
    ) = _maximum_heading_relaxation_rad(
        ideal_goal_pose=ideal_goal_pose,
        visibility_points=points,
        camera_mount=camera_mount,
        safe_half_fov_rad=(
            safe_half_fov_rad
        ),
    )

    if (
            abs(max_heading_offset_rad)
            <= _EPS
    ):
        print(
            "[SMOOTH_SELECT][RELAXATION] "
            "no usable relaxation range "
            f"approach_heading="
            f"{math.degrees(approach_heading_rad):+.1f}deg"
        )

        return SmoothPoseSelection(
            mode=(
                SmoothPoseSelectionMode.UNAVAILABLE
            ),
            pose=None,
            prediction=None,
            relaxation_fraction=None,
            heading_offset_rad=0.0,
            max_heading_offset_rad=0.0,
            approach_heading_rad=(
                approach_heading_rad
            ),
            tested_fractions=tuple(
                tested_fractions
            ),
        )

    for relaxation_fraction in (
        _RELAXATION_FRACTIONS[1:]
    ):

        heading_offset_rad = (
            relaxation_fraction
            * max_heading_offset_rad
        )

        candidate_pose = Pose2D(
            x_m=ideal_goal_pose.x_m,
            y_m=ideal_goal_pose.y_m,
            heading_rad=_wrap_angle_rad(
                ideal_goal_pose.heading_rad
                + heading_offset_rad
            ),
        )

        # Cheap terminal guard before performing another complete
        # Smooth forward prediction.
        if not _terminal_pose_visible(
            pose=candidate_pose,
            visibility_points=points,
            camera_mount=camera_mount,
            safe_half_fov_rad=(
                    safe_half_fov_rad
            ),
        ):
            print(
                "[SMOOTH_SELECT][RELAXED_SKIP] "
                f"fraction={relaxation_fraction:.2f} "
                f"offset="
                f"{math.degrees(heading_offset_rad):+.1f}deg "
                "reason=terminal_fov"
            )

            continue

        prediction = predict_smooth_fov(
            law=law,
            target_pose=candidate_pose,
            visibility_points=points,
            safe_half_fov_rad=(
                safe_half_fov_rad
            ),
            dt_s=dt_s,
            max_time_s=prediction_max_time_s,
            position_tolerance_m=(
                position_tolerance_m
            ),
            camera_mount=camera_mount,
        )

        tested_fractions.append(
            relaxation_fraction
        )

        print(
            "[SMOOTH_SELECT][RELAXED] "
            f"fraction={relaxation_fraction:.2f} "
            f"offset="
            f"{math.degrees(heading_offset_rad):+.1f}deg "
            f"max_offset="
            f"{math.degrees(max_heading_offset_rad):+.1f}deg "
            f"reached={prediction.reached_goal} "
            f"front={prediction.features_stayed_in_front} "
            f"max_bearing="
            f"{math.degrees(prediction.max_abs_bearing_rad):.1f}deg "
            f"peak_bearing="
            f"{math.degrees(prediction.peak_bearing_rad):+.1f}deg "
            f"peak_step={prediction.peak_step} "
            f"peak_time="
            f"{prediction.peak_step * dt_s:.2f}s "
            f"safe="
            f"{math.degrees(safe_half_fov_rad):.1f}deg "
            f"steps={prediction.steps}"
        )

        if prediction.admissible:

            return SmoothPoseSelection(
                mode=(
                    SmoothPoseSelectionMode.RELAXED
                ),
                pose=candidate_pose,
                prediction=prediction,
                relaxation_fraction=(
                    relaxation_fraction
                ),
                heading_offset_rad=(
                    heading_offset_rad
                ),
                max_heading_offset_rad=(
                    max_heading_offset_rad
                ),
                approach_heading_rad=(
                    approach_heading_rad
                ),
                tested_fractions=tuple(
                    tested_fractions
                ),
            )

    return SmoothPoseSelection(
        mode=(
            SmoothPoseSelectionMode.UNAVAILABLE
        ),
        pose=None,
        prediction=None,
        relaxation_fraction=None,
        heading_offset_rad=0.0,
        max_heading_offset_rad=(
            max_heading_offset_rad
        ),
        approach_heading_rad=(
            approach_heading_rad
        ),
        tested_fractions=tuple(
            tested_fractions
        ),
    )


__all__ = [
    "SmoothPoseSelection",
    "SmoothPoseSelectionMode",
    "select_smooth_pose",
]