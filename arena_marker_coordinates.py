# ============================================================
# Arena configuration
# ============================================================

arena_size_mm = 6000
# ↑ Change this value if the arena size changes (e.g. 5.0, 7.0)

# ============================================================
# Marker position generator
# ============================================================

def generate_arena_markers(arena_size_mm: float):
    """
    Generate (x, y) positions for SR arena boundary markers 0–19.

    - Square arena
    - No corner markers
    - 5 markers per wall
    - Equal spacing
    - Wall centre markers:
        Top    -> ID 2
        Right  -> ID 7
        Bottom -> ID 12
        Left   -> ID 17

    Coordinate system:
        (0, 0) = arena centre
        +x     = right
        +y     = forward / up
    """

    half = arena_size_mm / 2
    spacing = arena_size_mm / 6

    offsets = [-2 * spacing, -spacing, 0.0, spacing, 2 * spacing]

    markers = {}
    marker_id = 0

    # Top wall (IDs 0–4): left -> right
    for x in offsets:
        markers[marker_id] = (x, +half)
        marker_id += 1

    # Right wall (IDs 5–9): top -> bottom
    for y in reversed(offsets):
        markers[marker_id] = (+half, y)
        marker_id += 1

    # Bottom wall (IDs 10–14): right -> left
    for x in reversed(offsets):
        markers[marker_id] = (x, -half)
        marker_id += 1

    # Left wall (IDs 15–19): bottom -> top
    for y in offsets:
        markers[marker_id] = (-half, y)
        marker_id += 1

    return markers


# ============================================================
# Generate marker lookup table
# ============================================================

MARKER_POSITIONS = generate_arena_markers(arena_size_mm)

# ============================================================
# OPTIONAL: Debug printout of marker coordinates
# ============================================================

# The block below uses a special Python check:
#
#     if __name__ == "__main__":
#
# Python automatically sets the variable "__name__".
#
# - If this file is RUN directly (e.g. "python arena_marker_coordinates.py"),
#   then __name__ is set to "__main__" and the code inside this
#   block WILL run.
#
# - If this file is IMPORTED from another file (e.g. robot.py),
#   then __name__ is set to the filename ("arena_marker_coordinates"),
#   and the code inside this block WILL NOT run.
#
# This lets us include helpful debug output here without printing
# anything during a competition run.

if __name__ == "__main__":
    print(f"Arena size: {arena_size_mm} m x {arena_size_mm} m\n")
    print("Marker ID : (x, y) in meters")
    print("-" * 30)

    for marker_id in range(20):
        x, y = MARKER_POSITIONS[marker_id]
        print(f"{marker_id:>2} : ({x:>5.2f}, {y:>5.2f})")
