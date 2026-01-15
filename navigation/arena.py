# navigation/arena.py

"""
Static arena geometry.

Defines fixed world landmark positions.
No robot state. No perception. No motion.
"""

from enum import Enum
import math


class BaseID(Enum):
    BASE_0 = 0
    BASE_1 = 1
    BASE_2 = 2
    BASE_3 = 3


def marker_locations(arena_size_mm: float):
    """
    Return world coordinates of arena markers.

    Marker IDs are assumed to be:
    0–4   top wall (left → right)
    5–9   right wall (top → bottom)
    10–14 bottom wall (right → left)
    15–19 left wall (bottom → top)
    """
    half = arena_size_mm / 2
    spacing = arena_size_mm / 6

    offsets = [-2 * spacing, -spacing, 0.0, spacing, 2 * spacing]

    markers = {}
    marker_id = 0

    # Top wall
    for x in offsets:
        markers[marker_id] = (x, +half)
        marker_id += 1

    # Right wall
    for y in reversed(offsets):
        markers[marker_id] = (+half, y)
        marker_id += 1

    # Bottom wall
    for x in reversed(offsets):
        markers[marker_id] = (x, -half)
        marker_id += 1

    # Left wall
    for y in offsets:
        markers[marker_id] = (-half, y)
        marker_id += 1

    return markers

def base_bounds(arena_size_mm: float, base: BaseID):
    """
    Return (xmin, xmax, ymin, ymax) for a base region.
    """
    half = arena_size_mm / 2

    width = 2000.0   # long edge
    height = 1000.0  # short edge

    if base == BaseID.BASE_0:      # top-left
        return (-half, -half + width,
                half - height, half)

    if base == BaseID.BASE_1:      # top-right
        return (half - height, half,
                half - width, half)

    if base == BaseID.BASE_2:      # bottom-right
        return (half - width, half,
                -half, -half + height)

    if base == BaseID.BASE_3:      # bottom-left
        return (-half, -half + height,
                -half, -half + width)

    raise ValueError(base)

def base_dock_pose(arena_size_mm: float, base: BaseID):
    """
    Canonical pose inside base: (x, y, heading_rad)
    """
    xmin, xmax, ymin, ymax = base_bounds(arena_size_mm, base)
    margin = 300.0

    if base == BaseID.BASE_0:
        return xmin + margin, ymax - margin, -math.pi / 4

    if base == BaseID.BASE_1:
        return xmax - margin, ymax - margin, -3 * math.pi / 4

    if base == BaseID.BASE_2:
        return xmax - margin, ymin + margin, 3 * math.pi / 4

    if base == BaseID.BASE_3:
        return xmin + margin, ymin + margin, math.pi / 4

    raise ValueError(base)
