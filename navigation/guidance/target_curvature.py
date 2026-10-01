# navigation/guidance/target_curvature.py

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class TargetCurvatureResult:
    """
    Geometric curvature toward a target expressed in the robot frame.

    Sign convention:

        bearing_rad > 0
            target lies on the positive lateral side.

        curvature_per_m > 0
            curve toward that same positive lateral side.

    The caller is responsible for ensuring that its bearing and
    angular-velocity conventions are consistent with this convention.
    """

    distance_m: float
    bearing_rad: float

    target_x_m: float
    target_y_m: float

    curvature_per_m: float


def target_curvature(
    *,
    distance_m: float,
    bearing_rad: float,
) -> TargetCurvatureResult:
    """
    Compute the constant-curvature arc from the robot origin toward
    a relative target point.

    The target is represented in polar robot-relative coordinates:

        distance_m
            Straight-line range from robot origin to target.

        bearing_rad
            Target bearing relative to robot forward.

    For a target point:

        x = r cos(beta)
        y = r sin(beta)

    the corresponding target-point curvature is:

        kappa = 2 y / r^2

    equivalently:

        kappa = 2 sin(beta) / r

    Units:

        distance_m       metres
        bearing_rad      radians
        curvature_per_m  1 / metres

    This function performs geometry only. It does not choose speed,
    apply drivetrain limits, command motion, or determine whether an
    application should use curvature guidance.
    """

    distance_m = float(distance_m)
    bearing_rad = float(bearing_rad)

    if not math.isfinite(distance_m):
        raise ValueError(
            "distance_m must be finite"
        )

    if not math.isfinite(bearing_rad):
        raise ValueError(
            "bearing_rad must be finite"
        )

    if distance_m <= 0.0:
        raise ValueError(
            "distance_m must be > 0"
        )

    target_x_m = (
        distance_m * math.cos(bearing_rad)
    )

    target_y_m = (
        distance_m * math.sin(bearing_rad)
    )

    curvature_per_m = (
        2.0 * target_y_m
        / (distance_m * distance_m)
    )

    return TargetCurvatureResult(
        distance_m=distance_m,
        bearing_rad=bearing_rad,
        target_x_m=target_x_m,
        target_y_m=target_y_m,
        curvature_per_m=curvature_per_m,
    )