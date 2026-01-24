# navigation/wall_angle_ultrasonic2.py

"""
Wall angle estimation using two forward-facing ultrasonics.

Assumes two sensors are separated laterally by a known baseline and both measure
distance to the same (approximately planar) wall.

Angle convention:
- Returns "parallel error" in degrees: 0 means robot is parallel to wall.
- Positive means rotate + (CCW) to become parallel.
"""

from __future__ import annotations

import math
from typing import Optional


def wrap_deg_180(a: float) -> float:
    """Wrap to (-180, 180]."""
    return (a + 180.0) % 360.0 - 180.0


def parallel_error_from_two_distances(
    *,
    left_mm: float,
    right_mm: float,
    baseline_mm: float,
) -> float:
    """
    Simple geometry:
      error ~= atan2((right - left), baseline)

    0 when left==right (parallel).
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

    IMPORTANT:
    - Treat None as invalid.
    - Treat <=0 as invalid (0 often means "timeout / no echo" in SR-land). :contentReference[oaicite:1]{index=1}
    - Enforce max bound strictly.
    - Do NOT discard small-but-positive values just because they're below min_mm.
      (min_mm is a "sanity" hint, not a hard reject in practice.)
    """
    if left_mm is None or right_mm is None:
        return None

    if left_mm <= 0 or right_mm <= 0:
        return None

    if left_mm > max_mm or right_mm > max_mm:
        return None

    # Keep min_mm as informational; caller may warn.
    return parallel_error_from_two_distances(
        left_mm=float(left_mm),
        right_mm=float(right_mm),
        baseline_mm=float(baseline_mm),
    )
