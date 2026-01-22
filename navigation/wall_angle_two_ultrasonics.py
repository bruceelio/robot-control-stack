# navigation/wall_angle_two_ultrasonics.py

"""
Wall angle estimation using two forward-facing ultrasonics.

Assumes two sensors are separated laterally by a known baseline and both measure
distance to the same (approximately planar) wall.

Angle convention:
- Returns "parallel error" in degrees: 0 means robot is parallel to wall.
- Positive means rotate + (CCW) to become parallel (right sensor farther than left -> nose points toward wall on left).
"""

from __future__ import annotations

import math
from typing import Optional


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def wrap_deg_180(a: float) -> float:
    """Wrap to (-180, 180]."""
    a = (a + 180.0) % 360.0 - 180.0
    # keep +180 not -180 if you care; either is fine for control
    return a


def parallel_error_from_two_distances(
    *,
    left_mm: float,
    right_mm: float,
    baseline_mm: float,
) -> float:
    """
    Simple geometry:
      error ~= atan2((right - left), baseline)

    This gives an angle which is 0 when left==right (parallel).
    """
    if baseline_mm <= 0:
        raise ValueError("baseline_mm must be > 0")

    dx = float(right_mm) - float(left_mm)
    err_rad = math.atan2(dx, float(baseline_mm))
    return wrap_deg_180(math.degrees(err_rad))


def estimate_wall_parallel_error_two_ultrasonics(
    *,
    left_mm: Optional[float],
    right_mm: Optional[float],
    baseline_mm: float,
    min_mm: float,
    max_mm: float,
) -> Optional[float]:
    """
    Returns parallel error in degrees or None if invalid.
    """
    if left_mm is None or right_mm is None:
        return None

    if not (min_mm <= left_mm <= max_mm):
        return None
    if not (min_mm <= right_mm <= max_mm):
        return None

    return parallel_error_from_two_distances(
        left_mm=float(left_mm),
        right_mm=float(right_mm),
        baseline_mm=float(baseline_mm),
    )
