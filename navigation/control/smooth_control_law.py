# navigation/control/smooth_control_law.py
#
# SPDX-License-Identifier: Apache-2.0
#
# Python adaptation of the SmoothControlLaw used by Navigation2 / OpenNav.
#
# Upstream reference:
#   ros-navigation/navigation2
#   nav2_graceful_controller/src/smooth_control_law.cpp
#   nav2_graceful_controller/include/nav2_graceful_controller/ego_polar_coords.hpp
#
# The underlying pose-control method is based on:
#   Park & Kuipers, "A Smooth Control Law for Graceful Motion of Differential
#   Wheeled Mobile Robots in 2D Environment", ICRA 2011.
#
# This file intentionally contains only the generic pose-control mathematics.
# It does NOT depend on ROS, Nav2, AprilTags, cameras, perception, localisation,
# costmaps, path planning, collision checking, or robot hardware.
#
# Units:
#   position: metres
#   heading: radians
#   linear velocity: metres/second
#   angular velocity: radians/second
#
# Robot Cartesian convention:
#   +x = forward
#   +y = left
#   +heading = counter-clockwise / left
#
# The egocentric line-of-sight convention follows Navigation2's
# SmoothControlLaw formulation. A target to the robot's right therefore gives
# a positive line-of-sight angle inside the control law.

# Coordinate-convention note:
#
# The public Cartesian interface follows the robot-wide navigation convention:
#   +x       = forward
#   +y       = left
#   +heading = counter-clockwise / left
#
# Internally, however, the egocentric line-of-sight variables used by
# SmoothControlLaw intentionally retain the Navigation2 / OpenNav formulation.
# In those private Smooth coordinates a target to the robot's RIGHT gives a
# positive line-of-sight angle.
#
# Do not change that internal sign convention merely to match the public
# navigation bearing convention; doing so would require re-deriving the
# Smooth control law.

from __future__ import annotations

from dataclasses import dataclass
import math


_EPS = 1e-9


def _wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


@dataclass(frozen=True)
class Pose2D:
    """Planar pose in a common Cartesian frame."""

    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0


@dataclass(frozen=True)
class Velocity2D:
    """Differential-drive/unicycle velocity command."""

    linear_x_mps: float
    angular_z_rps: float


@dataclass(frozen=True)
class EgocentricPolar:
    """Target pose expressed in the SmoothControlLaw coordinates."""

    r_m: float
    phi_rad: float
    delta_rad: float


@dataclass(frozen=True)
class SmoothControlResult:
    """Velocity command plus intermediate values useful for diagnostics."""

    command: Velocity2D
    ego: EgocentricPolar
    curvature_per_m: float


@dataclass(frozen=True)
class SmoothControlParams:
    """
    SmoothControlLaw parameters.

    The defaults mirror the OpenNav docking controller defaults as a documented
    reference point. Robot-specific limits should be supplied by the caller.
    """

    k_phi: float = 3.0
    k_delta: float = 2.0
    beta: float = 0.4
    lambda_: float = 2.0

    slowdown_radius_m: float = 0.25
    deceleration_max_mps2: float = 2.5

    linear_min_mps: float = 0.10
    linear_max_mps: float = 0.25
    angular_max_rps: float = 0.75


class SmoothControlLaw:
    """
    Generic relative-pose controller for a differential-drive/unicycle robot.

    The controller receives a target pose relative to the robot. For the normal
    perception-only use case, the current robot pose is simply the origin:

        current = Pose2D(0, 0, 0)

    A fresh perception observation can therefore provide a fresh relative target
    pose on every update; global localisation is not required by this control
    law itself.

    Success/failure, observation freshness, face selection, docking-entry
    tolerances and controller handover are intentionally responsibilities of
    the caller.
    """

    def __init__(self, params: SmoothControlParams):
        self.params = params
        self._validate_params()

    def _validate_params(self) -> None:
        p = self.params

        if p.k_phi < 0.0 or p.k_delta < 0.0:
            raise ValueError("k_phi and k_delta must be >= 0")

        if p.beta < 0.0 or p.lambda_ < 0.0:
            raise ValueError("beta and lambda_ must be >= 0")

        if p.slowdown_radius_m <= 0.0:
            raise ValueError("slowdown_radius_m must be > 0")

        if p.deceleration_max_mps2 <= 0.0:
            raise ValueError("deceleration_max_mps2 must be > 0")

        if not (0.0 <= p.linear_min_mps <= p.linear_max_mps):
            raise ValueError(
                "require 0 <= linear_min_mps <= linear_max_mps"
            )

        if p.angular_max_rps <= 0.0:
            raise ValueError("angular_max_rps must be > 0")

    @staticmethod
    def egocentric_coordinates(
        target: Pose2D,
        current: Pose2D = Pose2D(),
        *,
        backward: bool = False,
    ) -> EgocentricPolar:
        dx = target.x_m - current.x_m
        dy = target.y_m - current.y_m

        line_of_sight = math.atan2(-dy, dx)

        if backward:
            line_of_sight += math.pi

        r_m = math.hypot(dx, dy)

        phi_rad = _wrap_angle(
            target.heading_rad + line_of_sight
        )

        delta_rad = _wrap_angle(
            current.heading_rad + line_of_sight
        )

        return EgocentricPolar(
            r_m=r_m,
            phi_rad=phi_rad,
            delta_rad=delta_rad,
        )

    def calculate_curvature(
        self,
        r_m: float,
        phi_rad: float,
        delta_rad: float,
    ) -> float:
        """
        Return requested path curvature in 1/metre.

        SmoothControlLaw is singular at r=0. The behaviour layer should declare
        success before that point. Returning zero here prevents numerical failure
        during diagnostics or forward simulation.
        """

        if r_m <= _EPS:
            return 0.0

        p = self.params

        proportional = p.k_delta * (
            delta_rad - math.atan(-p.k_phi * phi_rad)
        )

        feedback = (
            1.0
            + p.k_phi / (
                1.0 + (p.k_phi * phi_rad) ** 2
            )
        ) * math.sin(delta_rad)

        return -(proportional + feedback) / r_m

    def calculate_regular_velocity(
        self,
        target: Pose2D,
        current: Pose2D = Pose2D(),
        *,
        backward: bool = False,
    ) -> SmoothControlResult:
        ego = self.egocentric_coordinates(
            target,
            current,
            backward=backward,
        )

        if ego.r_m <= _EPS:
            return SmoothControlResult(
                command=Velocity2D(0.0, 0.0),
                ego=ego,
                curvature_per_m=0.0,
            )

        p = self.params

        curvature = self.calculate_curvature(
            ego.r_m,
            ego.phi_rad,
            ego.delta_rad,
        )

        if backward:
            curvature = -curvature

        # Reduce speed for tighter curvature.
        linear = p.linear_max_mps / (
            1.0
            + p.beta
            * abs(curvature) ** p.lambda_
        )

        # Reduce speed near the target pose.
        linear = min(
            p.linear_max_mps
            * (ego.r_m / p.slowdown_radius_m),
            linear,
        )

        # Respect the configured maximum deceleration.
        linear = min(
            math.sqrt(
                2.0
                * ego.r_m
                * p.deceleration_max_mps2
            ),
            linear,
        )

        linear = _clamp(
            linear,
            p.linear_min_mps,
            p.linear_max_mps,
        )

        if backward:
            linear = -linear

        angular = curvature * linear

        angular_limited = _clamp(
            angular,
            -p.angular_max_rps,
            p.angular_max_rps,
        )

        # If angular velocity is limited, reduce linear velocity as well so
        # that the requested path curvature is preserved.
        if abs(curvature) > _EPS:
            linear = angular_limited / curvature

        return SmoothControlResult(
            command=Velocity2D(
                linear_x_mps=linear,
                angular_z_rps=angular_limited,
            ),
            ego=ego,
            curvature_per_m=curvature,
        )

    def calculate_next_pose(
        self,
        dt_s: float,
        target: Pose2D,
        current: Pose2D = Pose2D(),
        *,
        backward: bool = False,
    ) -> Pose2D:
        """
        Forward-simulate one control step.

        This is for diagnostics/trajectory prediction. It is not localisation.
        """

        if dt_s < 0.0:
            raise ValueError("dt_s must be >= 0")

        result = self.calculate_regular_velocity(
            target,
            current,
            backward=backward,
        )

        cmd = result.command

        next_x = (
            current.x_m
            + cmd.linear_x_mps
            * dt_s
            * math.cos(current.heading_rad)
        )

        next_y = (
            current.y_m
            + cmd.linear_x_mps
            * dt_s
            * math.sin(current.heading_rad)
        )

        next_heading = _wrap_angle(
            current.heading_rad
            + cmd.angular_z_rps * dt_s
        )

        return Pose2D(
            x_m=next_x,
            y_m=next_y,
            heading_rad=next_heading,
        )