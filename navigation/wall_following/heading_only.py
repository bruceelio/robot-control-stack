# navigation/wall_following/heading_only.py

"""
Heading-only spatial wall-following controller.

This controller is used when wall heading is known directly but wall
distance is not available.

The controller is expressed spatially so that approximately the same
path curvature is requested at different forward speeds.

For wall-heading error e_h:

    curvature_heading = -K_heading * sin(e_h)

For a differential-drive / unicycle robot:

    omega = v * curvature

therefore:

    omega_heading = -|v| * K_heading * sin(e_h)

Wall-heading convention:

    WallGeometry.heading_rad is the wall-normal heading relative to
    base_link.

    0 rad      = wall directly ahead
    +pi / 2    = wall on the robot's left, robot parallel to wall
    -pi / 2    = wall on the robot's right, robot parallel to wall

The heading error is:

    heading_error = desired_heading - measured_heading

With this convention:

    positive heading_error -> robot is pointing too far toward the left
    negative heading_error -> robot is pointing too far toward the right

and the negative control sign turns the robot back toward the desired
parallel heading.

This is a moving wall-following controller. At zero forward speed the
spatial law intentionally commands zero angular velocity. A stationary
"turn parallel to wall" operation should be a separate orientation
controller / skill.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from navigation.wall_following.models import (
    WallFollowCommand,
    WallFollowResult,
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
class HeadingOnlyParams:
    """
    Spatial heading-only wall-following gains.

    heading_curvature_gain_per_m:
        Curvature response to wall-heading error.

        Units:
            1 / m

        Contribution:
            -|v| * K_heading * sin(heading_error)

    angular_max_rps:
        Maximum commanded yaw rate.

    The default heading gain of 3.0 1/m is the spatial equivalent of
    the previous 1.5 rad/s per rad controller at the 0.50 m/s reference
    speed:

        0.50 * 3.0 = 1.5
    """

    heading_curvature_gain_per_m: float = 3.0
    angular_max_rps: float = 0.8


class HeadingOnlyWallFollower:
    """
    Follow a wall using directly measured wall heading only.

    No wall-distance correction is performed.
    """

    def __init__(
        self,
        *,
        params: HeadingOnlyParams | None = None,
    ):
        self.params = (
            params
            if params is not None
            else HeadingOnlyParams()
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
        desired_heading_rad: float,
        linear_x_mps: float,
        **_,
    ) -> WallFollowResult:

        heading_rad = float(heading_rad)
        desired_heading_rad = float(desired_heading_rad)
        linear_x_mps = float(linear_x_mps)

        if not math.isfinite(heading_rad):
            raise ValueError(
                "heading_rad must be finite"
            )

        if not math.isfinite(desired_heading_rad):
            raise ValueError(
                "desired_heading_rad must be finite"
            )

        if not math.isfinite(linear_x_mps):
            raise ValueError(
                "linear_x_mps must be finite"
            )

        heading_error_rad = _wrap_pi(
            desired_heading_rad
            - heading_rad
        )

        heading_angular_z_rps = (
            -abs(linear_x_mps)
            * self.params.heading_curvature_gain_per_m
            * math.sin(heading_error_rad)
        )

        unclamped_angular_z_rps = (
            heading_angular_z_rps
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
            heading_angular_z_rps=(
                heading_angular_z_rps
            ),
            unclamped_angular_z_rps=(
                unclamped_angular_z_rps
            ),
        )
