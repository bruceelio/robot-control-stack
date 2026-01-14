# navigation/arena.py

"""
Static arena geometry.

Defines fixed world landmark positions.
No robot state. No perception. No motion.
"""

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


