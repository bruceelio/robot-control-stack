# navigation/wall_following/distance_only.py

"""
Distance-only wall-following controller.

This controller follows a wall using one side-range measurement and
forward robot velocity.

A single side-range sensor does not directly measure wall heading.
However, while the robot is moving, the rate of change of wall
distance contains geometric information about the robot's heading
relative to the wall.

For forward speed v and wall-heading error theta:

    distance_rate
        = -side_sign * v * sin(theta)

where:

    side_sign = +1 for a left wall
    side_sign = -1 for a right wall

Therefore an inferred heading error can be recovered from:

    theta
        = asin(
            -side_sign
            * distance_rate
            / v
        )

The controller combines:

    1. wall-distance error
    2. inferred wall-heading error

as a spatial curvature controller.

The commanded angular velocity is:

    omega
        = side_sign * v * Kd * distance_error
        - v * Kh * sin(theta)

where:

    Kd = distance curvature gain
    Kh = heading curvature gain

Scaling both terms with forward speed makes the controller response
approximately spatial: similar wall-distance and heading errors
produce similar path curvature at different travelling speeds.

When wall heading is directly available from another geometry source,
the heading-distance wall follower should normally be preferred.

References:

    A. Bemporad, M. Di Marco, A. Tesi,
    "Wall-Following Controllers for Sonar-Based Mobile Robots",
    IEEE Conference on Decision and Control, 1997.
    DOI: 10.1109/CDC.1997.657920

    A. Bemporad, M. Di Marco, A. Tesi,
    "Sonar-Based Wall-Following Control of Mobile Robots",
    Journal of Dynamic Systems, Measurement, and Control,
    122(1), 226-230, 2000.
    DOI: 10.1115/1.482468

The implementation here is not a verbatim implementation of either
published controller. It uses the same underlying unicycle geometry
to construct a lightweight distance-only fallback controller.
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


@dataclass(frozen=True)
class DistanceOnlyParams:

    """
    Spatial wall-following gains.

    curvature_kp_per_m2:
        Curvature response to wall-distance error.

        Units:
            1 / m^2

        Contribution:
            side_sign * |v| * K_distance * distance_error

    heading_curvature_gain_per_m:
        Curvature response to inferred wall-heading error.

        Units:
            1 / m

        Contribution:
            -|v| * K_heading * sin(heading_error)

    distance_rate_filter_tau_s:
        First-order low-pass filter time constant applied to the
        measured wall-distance rate before heading inference.

        Set to 0.0 to disable filtering.

    angular_max_rps:
        Maximum commanded yaw rate.
    """

    curvature_kp_per_m2: float = 16.0
    heading_curvature_gain_per_m: float = 4.0
    distance_rate_filter_tau_s: float = 0.0
    angular_max_rps: float = 0.8


class DistanceOnlyWallFollower:
    """
    Follow a wall using one side-distance measurement.

    The controller does not directly measure wall heading. While the
    robot is moving, it infers heading relative to the wall from the
    rate of change of the side-distance measurement.

    It combines:

        wall-distance error
        +
        inferred wall-heading error

    to steer the robot toward a trajectory parallel to the wall at
    the requested distance.

    Coordinate convention:

        +angular_z = counter-clockwise / left

    Therefore:

        LEFT wall:
            too far from wall -> positive angular command

        RIGHT wall:
            too far from wall -> negative angular command
    """

    def __init__(
        self,
        *,
        params: DistanceOnlyParams | None = None,
    ):
        self.params = (
            params
            if params is not None
            else DistanceOnlyParams()
        )

        self._previous_distance_mm: float | None = None
        self._filtered_distance_rate_mm_s: float | None = None

    def reset(self) -> None:
        self._previous_distance_mm = None
        self._filtered_distance_rate_mm_s = None

    def update(
        self,
        *,
        distance_mm: float,
        desired_distance_mm: float,
        wall_side: WallSide,
        linear_x_mps: float,
        dt_s: float,
    ) -> WallFollowResult:

        distance_mm = float(distance_mm)
        desired_distance_mm = float(
            desired_distance_mm
        )
        linear_x_mps = float(linear_x_mps)
        dt_s = float(dt_s)

        if distance_mm <= 0.0:
            raise ValueError(
                "distance_mm must be positive"
            )

        if desired_distance_mm <= 0.0:
            raise ValueError(
                "desired_distance_mm must be positive"
            )

        if dt_s <= 0.0:
            raise ValueError(
                "dt_s must be positive"
            )

        distance_error_mm = (
            distance_mm
            - desired_distance_mm
        )

        if self._previous_distance_mm is None:
            distance_rate_mm_s = 0.0
            distance_rate_filtered_mm_s = 0.0
        else:
            distance_rate_mm_s = (
                                         distance_mm
                                         - self._previous_distance_mm
                                 ) / dt_s

            tau_s = max(
                0.0,
                self.params.distance_rate_filter_tau_s,
            )

            if tau_s == 0.0:
                distance_rate_filtered_mm_s = (
                    distance_rate_mm_s
                )
            else:
                alpha = dt_s / (
                        tau_s + dt_s
                )

                previous_filtered = (
                    self._filtered_distance_rate_mm_s
                    if self._filtered_distance_rate_mm_s
                       is not None
                    else distance_rate_mm_s
                )

                distance_rate_filtered_mm_s = (
                        previous_filtered
                        + alpha
                        * (
                                distance_rate_mm_s
                                - previous_filtered
                        )
                )

        self._previous_distance_mm = distance_mm
        self._filtered_distance_rate_mm_s = (
            distance_rate_filtered_mm_s
        )

        distance_error_m = (
            distance_error_mm
            / 1000.0
        )

        distance_rate_mps = (
            distance_rate_filtered_mm_s
            / 1000.0
        )

        if abs(linear_x_mps) > 1e-6:
            rate_ratio = (
                    distance_rate_mps
                    / abs(linear_x_mps)
            )

            rate_ratio = _clamp(
                rate_ratio,
                -1.0,
                +1.0,
            )

            inferred_heading_error_rad = (
                    (-1.0 if wall_side == WallSide.LEFT else +1.0)
                    * math.asin(rate_ratio)
            )
        else:
            inferred_heading_error_rad = None

        # Spatial wall-following law:
        #
        # The controller combines wall-distance error with an inferred
        # wall-heading error.
        #
        # Distance contribution:
        #
        #     omega_distance
        #         = side_sign * |v| * K_distance * distance_error
        #
        # Heading contribution:
        #
        #     omega_heading
        #         = -|v| * K_heading * sin(heading_error)
        #
        # Therefore:
        #
        #     omega
        #         = omega_distance + omega_heading
        #
        # The heading error is inferred above from the rate of change of
        # side-wall distance while the robot is moving.
        #
        # Scaling both contributions with forward speed makes the controller
        # approximately spatial: the same geometric errors produce similar
        # path curvature at different travelling speeds.

        proportional_correction_rps = (
            abs(linear_x_mps)
            * self.params.curvature_kp_per_m2
            * distance_error_m
        )

        heading_correction_rps = (
                -abs(linear_x_mps)
                * self.params.heading_curvature_gain_per_m
                * math.sin(inferred_heading_error_rad)
        )



        side_sign = (
            +1.0
            if wall_side == WallSide.LEFT
            else -1.0
        )

        proportional_angular_z_rps = (
                side_sign
                * proportional_correction_rps
        )

        heading_angular_z_rps = (
            heading_correction_rps
        )

        unclamped_angular_z_rps = (
                proportional_angular_z_rps
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
            inferred_heading_error_rad=(
                inferred_heading_error_rad
            ),
            distance_error_mm=distance_error_mm,
            distance_rate_mm_s=distance_rate_mm_s,
            distance_rate_filtered_mm_s=(
                distance_rate_filtered_mm_s
            ),
            distance_proportional_angular_z_rps=(
                proportional_angular_z_rps
            ),
            heading_angular_z_rps=(
                heading_angular_z_rps
            ),
            unclamped_angular_z_rps=(
                unclamped_angular_z_rps
            ),
        )