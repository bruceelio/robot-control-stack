# config/arena.py

"""
Static arena geometry.

Defines fixed world landmark positions.
No robot state. No perception. No motion.
"""

from enum import Enum
import math

# --------------------------------------------------
# Arena dimensions
# --------------------------------------------------

ARENA_SIZE = 4575

BASE_WIDTH_MM  = 2000.0   # long edge
BASE_HEIGHT_MM = 1000.0   # short edge
BASE_MARGIN_MM = 300

CENTRAL_PLATFORM_SIZE_MM = 1220
CENTRAL_PLATFORM_HEIGHT_MM = 180


# --------------------------------------------------
# Base identifiers
# --------------------------------------------------

class BaseID(Enum):
    BASE_0 = 0   # top-left
    BASE_1 = 1   # top-right
    BASE_2 = 2   # bottom-right
    BASE_3 = 3   # bottom-left


# --------------------------------------------------
# Arena markers
# --------------------------------------------------

def marker_locations(arena_size: int):
    """
    Return world coordinates of arena boundary markers.

    Marker IDs:
    0–4   top wall (left → right)
    5–9   right wall (top → bottom)
    10–14 bottom wall (right → left)
    15–19 left wall (bottom → top)
    """
    half = arena_size / 2
    spacing = arena_size / 6

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

# --------------------------------------------------
# April Tags
# --------------------------------------------------

APRILTAG_FAMILY = "tag36h11"
APRILTAG_ID_GROUPS = {
    "arena": tuple(range(0, 20)),
    "object": tuple(range(100, 200)),
}

APRILTAG_SIZE_BY_GROUP_M = {
    "arena": 0.15,
    "object": 0.08,
}

# --------------------------------------------------
# Base geometry
# --------------------------------------------------

def base_bounds(base: BaseID, arena_size: int):
    """
    Return (xmin, xmax, ymin, ymax) for a base region.
    """
    h = arena_size / 2
    w = BASE_WIDTH_MM
    d = BASE_HEIGHT_MM

    if base == BaseID.BASE_0:      # top-left
        return (-h, -h + w,
                +h - d, +h)

    if base == BaseID.BASE_1:      # top-right
        return (+h - w, +h,
                +h - d, +h)

    if base == BaseID.BASE_2:      # bottom-right
        return (+h - w, +h,
                -h, -h + d)

    if base == BaseID.BASE_3:      # bottom-left
        return (-h, -h + w,
                -h, -h + d)

    raise ValueError(base)


def base_dock_pose(base: BaseID, arena_size: int):
    """
    Canonical pose inside base: (x, y, heading_rad)
    """
    xmin, xmax, ymin, ymax = base_bounds(base, arena_size)
    margin = BASE_MARGIN_MM

    if base == BaseID.BASE_0:
        return xmin + margin, ymax - margin, -math.pi / 4

    if base == BaseID.BASE_1:
        return xmax - margin, ymax - margin, -3 * math.pi / 4

    if base == BaseID.BASE_2:
        return xmax - margin, ymin + margin, 3 * math.pi / 4

    if base == BaseID.BASE_3:
        return xmin + margin, ymin + margin, math.pi / 4

    raise ValueError(base)

START_POSES = {
    1: {  # slot 1
        0: (-2015.5, +1823.5, 0.0),
        1: (+1823.5, +2015.5, -1.5708),
        2: (+2015.5, -1823.5, 3.1416),
        3: (-1823.5, -2015.5, +1.5708),
    },
    2: {  # slot 2
        0: (-1787.5, -162.5, 0.0),
        1: (-162.5, +1787.5, -1.5708),
        2: (+1787.5, +162.5, 3.1416),
        3: (+162.5, -1787.5, +1.5708),
    },
}

def get_start_pose(base: int, slot: int):
    try:
        return START_POSES[slot][base]
    except KeyError as e:
        raise ValueError(f"Unknown start pose base={base} slot={slot}") from e