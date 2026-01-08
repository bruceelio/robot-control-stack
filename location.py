import math

def marker_location(arena_size_mm: float):
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

def find_location(A, B, AC, BC, angle_C=None):
    """
    Find coordinates of point C given:
    A, B: tuples of coordinates (x, y)
    AC: distance from A to C
    BC: distance from B to C
    angle_C: optional, angle at C in degrees to pick correct solution
    """
    x1, y1 = A
    x2, y2 = B

    # Calculate distance between A and B
    AB = math.hypot(x2 - x1, y2 - y1)

    # Solve for intersection of two circles
    # https://math.stackexchange.com/questions/256100/how-can-i-find-the-points-at-which-two-circles-intersect
    # Distance between centers
    d = AB

    # Check if solution exists
    if d > AC + BC or d < abs(AC - BC):
        raise ValueError("No triangle possible with these lengths.")

    # Distance from A along the line AB to the point P where the line through intersection points crosses AB
    a = (AC**2 - BC**2 + d**2) / (2*d)

    # Height from point P to intersection points
    h = math.sqrt(max(0, AC**2 - a**2))

    # Point P coordinates
    Px = x1 + a * (x2 - x1) / d
    Py = y1 + a * (y2 - y1) / d

    # Offsets for the two possible solutions
    offset_x = -h * (y2 - y1) / d
    offset_y = h * (x2 - x1) / d

    # Two possible points
    C1 = (Px + offset_x, Py + offset_y)
    C2 = (Px - offset_x, Py - offset_y)

    # If angle_C is given, pick the correct one
    if angle_C is not None:
        # Compute angle at C for both candidates using Law of Cosines
        def angle_at_C(C):
            AC_len = math.hypot(C[0] - x1, C[1] - y1)
            BC_len = math.hypot(C[0] - x2, C[1] - y2)
            cos_angle = (AC_len**2 + BC_len**2 - AB**2) / (2 * AC_len * BC_len)
            cos_angle = max(-1, min(1, cos_angle))  # clamp for numerical stability
            return math.degrees(math.acos(cos_angle))
        
        C1_angle = angle_at_C(C1)
        C2_angle = angle_at_C(C2)
        
        if abs(C1_angle - angle_C) < abs(C2_angle - angle_C):
            return C1
        else:
            return C2

    # If no angle, just return both possibilities
    return C1, C2

'''
# Example usage:
A = (0, 1)
B = (0, 2)
AC = 2.236
BC = 2.828
angle_C = 18.43  # optional

C = find_third_vertex(A, B, AC, BC, angle_C)
print("C =", C)
'''
