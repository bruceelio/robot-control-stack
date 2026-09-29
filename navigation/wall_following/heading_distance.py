# navigation/wall_following/heading_distance.py

"""
Heading-and-distance spatial wall-following controller.

This controller is used when both wall heading and perpendicular wall
distance are known directly.

It combines:

    wall-distance error
    +
    wall-heading error

and expresses both terms spatially so that approximately the same path
curvature is requested at different forward speeds.

Distance contribution:

    omega_distance
        = side_sign
        * |v|
        * K_distance
        * distance_error

where:

    side_sign = +1 for a LEFT wall
                -1 for a RIGHT wall

Heading contribution:

    heading_error
        = desired_heading - measured_heading

    omega_heading
        = -|v|
        * K_heading
        * sin(heading_error)

Total:

    omega
        = omega_distance
        + omega_heading

Wall-heading convention:

    WallGeometry.heading_rad is the wall-normal heading relative to
    base_link.

    0 rad      = wall directly ahead
    +pi / 2    = wall on the robot's left, robot parallel to wall
    -pi / 2    = wall on the robot's right, robot parallel to wall

This controller should normally be preferred over distance-only wall
following when reliable direct wall heading is available because it
does not need to infer heading from the time derivative of one side
range.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from navigation.wall_following.models import (
    WallFollowCommand,
    WallFollowResult,
    WallSide,
)


def _clamp(
    value: float,
    minimum: float,
    maximum: float,
) -> float:
    return max(
        minimum,
        min(value, maximum),
    )


def _wrap_pi(angle_rad: float) -> float:
    return math.atan2(
        math.sin(angle_rad),
        math.cos(angle_rad),
    )


@dataclass(frozen=True)
class HeadingDistanceParams:
    """
    Spatial heading-and-distance wall-following gains.

    distance_curvature_kp_per_m2:
        Curvature response to wall-distance error.

        Units:
            1 / m^2

        Contribution:
            side_sign
            * |v|
            * K_distance
            * distance_error

    heading_curvature_gain_per_m:
        Curvature response to directly measured wall-heading error.

        Units:
            1 / m

        Contribution:
            -|v|
            * K_heading
            * sin(heading_error)

    angular_max_rps:
        Maximum commanded yaw rate.

    Defaults preserve approximately the previous controller response at
    the 0.50 m/s reference speed:

        0.50 * 4.0 = 2.0 rad/s per m
        0.50 * 3.0 = 1.5 rad/s per rad
    """

    distance_curvature_kp_per_m2: float = 4.0
    heading_curvature_gain_per_m: float = 3.0
    angular_max_rps: float = 0.8


class HeadingDistanceWallFollower:
    """
    Follow a wall using directly measured wall heading and distance.
    """

    def __init__(
        self,
        *,
        params: HeadingDistanceParams | None = None,
    ):
        self.params = (
            params
            if params is not None
            else HeadingDistanceParams()
        )

    def reset(self) -> None:
        """
        Stateless controller; provided for a common wall-follower API.
        """
        return None

    def update(
        self,
        *,
        heading_rad: float,
        distance_mm: float,
        desired_heading_rad: float,
        desired_distance_mm: float,
        wall_side: WallSide,
        linear_x_mps: float,
        **_,
    ) -> WallFollowResult:

        heading_rad = float(heading_rad)
        distance_mm = float(distance_mm)
        desired_heading_rad = float(
            desired_heading_rad
        )
        desired_distance_mm = float(
            desired_distance_mm
        )
        linear_x_mps = float(linear_x_mps)

        if not math.isfinite(heading_rad):
            raise ValueError(
                "heading_rad must be finite"
            )

        if not math.isfinite(desired_heading_rad):
            raise ValueError(
                "desired_heading_rad must be finite"
            )

        if (
            not math.isfinite(distance_mm)
            or distance_mm <= 0.0
        ):
            raise ValueError(
                "distance_mm must be positive and finite"
            )

        if (
            not math.isfinite(desired_distance_mm)
            or desired_distance_mm <= 0.0
        ):
            raise ValueError(
                "desired_distance_mm must be positive and finite"
            )

        if not math.isfinite(linear_x_mps):
            raise ValueError(
                "linear_x_mps must be finite"
            )

        if wall_side not in (
            WallSide.LEFT,
            WallSide.RIGHT,
        ):
            raise ValueError(
                "wall_side must be WallSide.LEFT or WallSide.RIGHT"
            )

        distance_error_mm = (
            distance_mm
            - desired_distance_mm
        )

        distance_error_m = (
            distance_error_mm
            / 1000.0
        )

        heading_error_rad = _wrap_pi(
            desired_heading_rad
            - heading_rad
        )

        side_sign = (
            +1.0
            if wall_side == WallSide.LEFT
            else -1.0
        )

        distance_angular_z_rps = (
            side_sign
            * abs(linear_x_mps)
            * self.params.distance_curvature_kp_per_m2
            * distance_error_m
        )

        heading_angular_z_rps = (
            -abs(linear_x_mps)
            * self.params.heading_curvature_gain_per_m
            * math.sin(heading_error_rad)
        )

        unclamped_angular_z_rps = (
            distance_angular_z_rps
            + heading_angular_z_rps
        )

        angular_z_rps = _clamp(
            unclamped_angular_z_rps,
            -self.params.angular_max_rps,
            +self.params.angular_max_rps,
        )

        return WallFollowResult(
            command=WallFollowCommand(
                linear_x_mps=linear_x_mps,
                angular_z_rps=angular_z_rps,
            ),
            distance_error_mm=distance_error_mm,
            distance_proportional_angular_z_rps=(
                distance_angular_z_rps
            ),
            heading_angular_z_rps=(
                heading_angular_z_rps
            ),
            unclamped_angular_z_rps=(
                unclamped_angular_z_rps
            ),
        )
