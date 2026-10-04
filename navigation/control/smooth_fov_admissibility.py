# navigation/control/smooth_fov_admissibility.py

from __future__ import annotations

from dataclasses import dataclass
import math

from navigation.control.smooth_control_law import (
    Pose2D,
    SmoothControlLaw,
)

from collections.abc import Sequence

@dataclass(frozen=True)
class SmoothFovPrediction:
    admissible: bool

    max_abs_bearing_rad: float

    # Signed bearing of the feature which produced the
    # maximum absolute FOV excursion.
    #
    # Canonical convention:
    #     positive = left / counter-clockwise
    #     negative = right / clockwise
    peak_bearing_rad: float

    # Prediction step at which peak_bearing_rad occurred.
    peak_step: int

    reached_goal: bool
    features_stayed_in_front: bool
    steps: int

    @property
    def marker_stayed_in_front(self) -> bool:
        """
        Backwards-compatible alias for older diagnostics.
        """

        return self.features_stayed_in_front


def _wrap_angle(angle_rad: float) -> float:
    return math.atan2(
        math.sin(angle_rad),
        math.cos(angle_rad),
    )


def camera_bearing_to_point(
    *,
    robot_pose: Pose2D,
    camera_mount: Pose2D,
    point_x_m: float,
    point_y_m: float,
) -> tuple[float, bool]:
    """
    Return horizontal bearing from the camera to a fixed point.

    Conventions:
        robot/camera +x = forward
        robot/camera +y = left
        positive yaw = left / CCW
        returned bearing positive = left / counter-clockwise
    """

    robot_c = math.cos(robot_pose.heading_rad)
    robot_s = math.sin(robot_pose.heading_rad)

    camera_x_m = (
        robot_pose.x_m
        + robot_c * camera_mount.x_m
        - robot_s * camera_mount.y_m
    )
    camera_y_m = (
        robot_pose.y_m
        + robot_s * camera_mount.x_m
        + robot_c * camera_mount.y_m
    )
    camera_heading_rad = _wrap_angle(
        robot_pose.heading_rad
        + camera_mount.heading_rad
    )

    dx_m = point_x_m - camera_x_m
    dy_m = point_y_m - camera_y_m

    camera_c = math.cos(camera_heading_rad)
    camera_s = math.sin(camera_heading_rad)

    point_camera_x_m = (
        camera_c * dx_m
        + camera_s * dy_m
    )
    point_camera_y_m = (
        -camera_s * dx_m
        + camera_c * dy_m
    )

    bearing_rad = math.atan2(
        point_camera_y_m,
        point_camera_x_m,
    )

    return bearing_rad, point_camera_x_m > 0.0


def predict_smooth_fov(
    *,
    law: SmoothControlLaw,
    target_pose: Pose2D,
    visibility_points: Sequence[
        tuple[float, float]
    ],
    safe_half_fov_rad: float,
    dt_s: float = 0.10,
    max_time_s: float = 5.0,
    position_tolerance_m: float = 0.01,
    camera_mount: Pose2D = Pose2D(),
) -> SmoothFovPrediction:
    """
    Forward-simulate SmoothControlLaw and predict visibility-feature
    FOV usage.

    target_pose and every visibility point are expressed in the robot
    frame at the instant prediction begins. That initial frame is
    treated as the fixed world frame for the simulation.

    This function is diagnostic/pure calculation only. It does not
    command motion, inspect perception, or select a controller.
    """

    if dt_s <= 0.0:
        raise ValueError("dt_s must be > 0")

    if max_time_s <= 0.0:
        raise ValueError("max_time_s must be > 0")

    if position_tolerance_m < 0.0:
        raise ValueError(
            "position_tolerance_m must be >= 0"
        )

    if not (0.0 < safe_half_fov_rad < math.pi):
        raise ValueError(
            "safe_half_fov_rad must be between 0 and pi"
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

    current_pose = Pose2D()

    max_abs_bearing_rad = 0.0
    peak_bearing_rad = 0.0
    peak_step = 0

    features_stayed_in_front = True
    reached_goal = False

    max_steps = max(
        1,
        int(math.ceil(max_time_s / dt_s)),
    )

    steps = 0

    for step_index in range(max_steps + 1):
        step_features_in_front = True

        for point_x_m, point_y_m in points:
            bearing_rad, feature_in_front = (
                camera_bearing_to_point(
                    robot_pose=current_pose,
                    camera_mount=camera_mount,
                    point_x_m=point_x_m,
                    point_y_m=point_y_m,
                )
            )

            abs_bearing_rad = abs(
                bearing_rad
            )

            if (
                    abs_bearing_rad
                    > max_abs_bearing_rad
            ):
                max_abs_bearing_rad = (
                    abs_bearing_rad
                )

                peak_bearing_rad = (
                    bearing_rad
                )

                peak_step = (
                    step_index
                )

            if not feature_in_front:
                step_features_in_front = False

        if not step_features_in_front:
            features_stayed_in_front = False
            steps = step_index
            break

        control = law.calculate_regular_velocity(
            target_pose,
            current_pose,
        )

        if (
            control.ego.r_m
            <= position_tolerance_m
        ):
            reached_goal = True
            steps = step_index
            break

        if step_index == max_steps:
            steps = step_index
            break

        current_pose = law.calculate_next_pose(
            dt_s,
            target_pose,
            current_pose,
        )

        steps = step_index + 1

    admissible = (
        reached_goal
        and features_stayed_in_front
        and (
            max_abs_bearing_rad
            <= safe_half_fov_rad
        )
    )

    return SmoothFovPrediction(
        admissible=admissible,
        max_abs_bearing_rad=max_abs_bearing_rad,
        peak_bearing_rad=peak_bearing_rad,
        peak_step=peak_step,
        reached_goal=reached_goal,
        features_stayed_in_front=(
            features_stayed_in_front
        ),
        steps=steps,
    )

__all__ = [
    "SmoothFovPrediction",
    "camera_bearing_to_point",
    "predict_smooth_fov",
]