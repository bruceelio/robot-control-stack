# skills/navigation/align_to_target_utils.py

"""
Alignment utilities (pure logic).

- No motion
- No hardware access
- No control-loop state
- Safe for use in behaviors, planning, and tests
"""

from __future__ import annotations


def is_aligned(bearing_deg: float, *, tolerance_deg: float) -> bool:
    """True if bearing is within deadband."""
    return abs(float(bearing_deg)) < float(tolerance_deg)


def clamp_rotation_deg(bearing_deg: float, *, max_rotate_deg: float) -> float:
    """Clamp requested rotation to +/- max_rotate_deg."""
    b = float(bearing_deg)
    m = float(max_rotate_deg)
    if b > m:
        return m
    if b < -m:
        return -m
    return b
