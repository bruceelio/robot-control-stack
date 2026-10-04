# navigation/wall_geometry/multi_ray_plane.py

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from navigation.wall_geometry.models import (
    RangeRay2D,
    WallGeometry,
)


@dataclass(frozen=True)
class WallPlaneFit:
    """
    Best-fit planar wall through a set of range-ray hit points.

    geometry:
        Robot-relative wall normal heading and perpendicular distance.

    support_count:
        Number of rays used in the fit.

    rms_residual_mm:
        Root-mean-square perpendicular distance of the hit points from
        the fitted wall line.

    max_residual_mm:
        Largest perpendicular distance of any supporting hit point from
        the fitted wall line.

    span_mm:
        Distance between the extreme supporting hit points measured
        along the fitted wall direction.

    linearity_ratio:
        Minor / major covariance eigenvalue.

        Near zero:
            observations are strongly line-like.

        Larger values:
            observations are increasingly inconsistent with one line.

    This object deliberately reports geometry and diagnostics only.
    It does not decide whether a fit is trustworthy enough for a
    particular application.
    """

    geometry: WallGeometry

    support_count: int

    rms_residual_mm: float
    max_residual_mm: float

    span_mm: float
    linearity_ratio: float


def ray_hit_point(
    ray: RangeRay2D,
) -> tuple[float, float]:
    """
    Convert one RangeRay2D into its measured hit point.

    The returned point is expressed in the same reference frame as the
    ray origin and heading.
    """

    distance_mm = float(
        ray.distance_mm
    )

    origin_x_mm = float(
        ray.origin_x_mm
    )

    origin_y_mm = float(
        ray.origin_y_mm
    )

    heading_rad = float(
        ray.ray_heading_rad
    )

    values = (
        distance_mm,
        origin_x_mm,
        origin_y_mm,
        heading_rad,
    )

    if not all(
        math.isfinite(value)
        for value in values
    ):
        raise ValueError(
            "range ray contains non-finite geometry"
        )

    if distance_mm <= 0.0:
        raise ValueError(
            "range distance must be positive"
        )

    x_mm = (
        origin_x_mm
        + distance_mm
        * math.cos(heading_rad)
    )

    y_mm = (
        origin_y_mm
        + distance_mm
        * math.sin(heading_rad)
    )

    return x_mm, y_mm


def fit_wall_from_rays(
    rays: Sequence[RangeRay2D],
) -> WallPlaneFit:
    """
    Fit one 2D wall line to two or more RangeRay2D observations.

    The fit uses orthogonal least squares / total least squares:

        1. convert each ray into its measured hit point;
        2. calculate the hit-point centroid;
        3. calculate the 2D covariance matrix;
        4. use the principal covariance direction as the wall tangent;
        5. use the perpendicular direction as the wall normal;
        6. orient the normal from base_link toward the observed wall.

    Unlike estimate_wall_from_two_rays(), this function does not assume
    that the observations genuinely describe one wall.

    Instead it reports residual and linearity diagnostics so a higher
    layer can decide whether the single-wall hypothesis is credible.

    This distinction is important near corners, obstacles and target
    objects where different rays may legitimately hit different
    surfaces.
    """

    rays = tuple(rays)

    if len(rays) < 2:
        raise ValueError(
            "at least two range rays are required"
        )

    points = tuple(
        ray_hit_point(ray)
        for ray in rays
    )

    count = len(points)

    mean_x = (
        sum(
            point[0]
            for point in points
        )
        / count
    )

    mean_y = (
        sum(
            point[1]
            for point in points
        )
        / count
    )

    covariance_xx = 0.0
    covariance_xy = 0.0
    covariance_yy = 0.0

    for x_mm, y_mm in points:

        dx = x_mm - mean_x
        dy = y_mm - mean_y

        covariance_xx += dx * dx
        covariance_xy += dx * dy
        covariance_yy += dy * dy

    covariance_xx /= count
    covariance_xy /= count
    covariance_yy /= count

    trace = (
        covariance_xx
        + covariance_yy
    )

    discriminant = math.hypot(
        covariance_xx - covariance_yy,
        2.0 * covariance_xy,
    )

    eigenvalue_major = (
        trace + discriminant
    ) * 0.5

    eigenvalue_minor = (
        trace - discriminant
    ) * 0.5

    if eigenvalue_major <= 1e-12:
        raise ValueError(
            "range-ray hit points do not define a wall direction"
        )

    # Principal covariance direction.
    #
    # Line orientation is modulo pi, so either direction along the wall
    # is geometrically equivalent.
    wall_heading_rad = 0.5 * math.atan2(
        2.0 * covariance_xy,
        covariance_xx - covariance_yy,
    )

    wall_x = math.cos(
        wall_heading_rad
    )

    wall_y = math.sin(
        wall_heading_rad
    )

    # One of the two perpendicular unit normals.
    normal_x = -wall_y
    normal_y = wall_x

    # Choose the normal which points from base_link toward the observed
    # wall, matching WallGeometry's convention.
    if (
        normal_x * mean_x
        + normal_y * mean_y
    ) < 0.0:

        normal_x = -normal_x
        normal_y = -normal_y

    distance_mm = (
        normal_x * mean_x
        + normal_y * mean_y
    )

    normal_heading_rad = math.atan2(
        normal_y,
        normal_x,
    )

    residuals: list[float] = []
    wall_projections: list[float] = []

    for x_mm, y_mm in points:

        residual_mm = abs(
            normal_x * x_mm
            + normal_y * y_mm
            - distance_mm
        )

        residuals.append(
            residual_mm
        )

        wall_projection_mm = (
            wall_x * x_mm
            + wall_y * y_mm
        )

        wall_projections.append(
            wall_projection_mm
        )

    rms_residual_mm = math.sqrt(
        sum(
            value * value
            for value in residuals
        )
        / count
    )

    max_residual_mm = max(
        residuals
    )

    span_mm = (
        max(wall_projections)
        - min(wall_projections)
    )

    linearity_ratio = max(
        0.0,
        eigenvalue_minor,
    ) / eigenvalue_major

    return WallPlaneFit(
        geometry=WallGeometry(
            heading_rad=normal_heading_rad,
            distance_mm=distance_mm,
        ),
        support_count=count,
        rms_residual_mm=rms_residual_mm,
        max_residual_mm=max_residual_mm,
        span_mm=span_mm,
        linearity_ratio=linearity_ratio,
    )


__all__ = [
    "WallPlaneFit",
    "fit_wall_from_rays",
    "ray_hit_point",
]