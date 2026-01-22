# navigation/wall_angle_one_ultrasonic.py

"""
Wall angle estimation using one forward-facing ultrasonic + 2-angle scan.

We measure distance d1 at angle theta1 and d2 at angle theta2 (robot frame),
and solve for the wall normal direction using:

    1/d = A cos(theta) + B sin(theta)

Where A = cos(n)/p, B = sin(n)/p, n is the wall-normal angle, p is perpendicular distance.

From n we get wall direction w = n + 90deg. The parallel error is the smallest
angle to rotate robot heading to align with wall direction.

Returns:
- parallel_error_deg in [-90, 90] (0 = parallel).
"""

from __future__ import annotations

import math
from typing import Optional, Tuple


def wrap_deg_180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def wrap_deg_90(a: float) -> float:
    """
    Map an angle to [-90, 90] by treating +180 as same parallel direction.
    """
    a = wrap_deg_180(a)
    if a > 90.0:
        a -= 180.0
    if a < -90.0:
        a += 180.0
    return a


def solve_wall_normal_from_two_scans(
    *,
    theta1_deg: float,
    d1_mm: float,
    theta2_deg: float,
    d2_mm: float,
) -> Optional[float]:
    """
    Returns wall-normal angle n in degrees (robot frame) or None if singular.
    """
    # y = 1/d = A cosθ + B sinθ
    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)

    y1 = 1.0 / float(d1_mm)
    y2 = 1.0 / float(d2_mm)

    c1, s1 = math.cos(t1), math.sin(t1)
    c2, s2 = math.cos(t2), math.sin(t2)

    det = c1 * s2 - s1 * c2
    if abs(det) < 1e-9:
        return None

    # Solve:
    # [c1 s1][A] = [y1]
    # [c2 s2][B]   [y2]
    A = (y1 * s2 - s1 * y2) / det
    B = (c1 * y2 - y1 * c2) / det

    n_rad = math.atan2(B, A)
    return wrap_deg_180(math.degrees(n_rad))


def parallel_error_from_two_scans(
    *,
    theta1_deg: float,
    d1_mm: float,
    theta2_deg: float,
    d2_mm: float,
) -> Optional[float]:
    """
    Returns parallel error in degrees ([-90, 90]) or None.
    """
    n_deg = solve_wall_normal_from_two_scans(
        theta1_deg=theta1_deg,
        d1_mm=d1_mm,
        theta2_deg=theta2_deg,
        d2_mm=d2_mm,
    )
    if n_deg is None:
        return None

    # wall direction is +90 from wall normal
    wall_dir = wrap_deg_180(n_deg + 90.0)

    # robot heading is 0; error is "how far wall_dir is from heading"
    return wrap_deg_90(wall_dir)
