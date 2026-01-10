# navigation.py
from motion import DRIVE_FOR, ROTATE_FOR

# =========================
# Target selection
# =========================

def get_closest_target(perception, kind="acidic"):
    memory = perception.objects.get(kind, {})
    if not memory:
        return None

    # Always prefer closest by distance
    targets = sorted(memory.values(), key=lambda t: t["distance"])
    return targets[0]


# =========================
# Navigation utilities
# =========================

def drive_to_target(lvl2, target, position, heading, max_drive_mm=500, tolerance_mm=50):
    """
    Drive a single step toward a target object.

    Returns updated (position, heading)
    """
    distance = target["distance"]
    bearing = target["bearing"]



    # --- Drive forward if not yet within tolerance ---
    if distance > tolerance_mm:
        step = min(distance, max_drive_mm)
        dx, dy = DRIVE_FOR(lvl2, step, heading)
        position = (position[0] + dx, position[1] + dy)

    return position, heading


# =========================
# High-level navigation
# =========================

def seek_and_collect(lvl2, perception, position, heading, kind="acidic",
                     max_drive_mm=500, tolerance_mm=50):
    """
    Seek the closest target of the given kind and move one step toward it.

    Returns (found, position, heading)
    - found = True if the robot is within tolerance of the target
    """
    target = get_closest_target(perception, kind)
    if target is None:
        return False, position, heading

    # Rotate towards target
    heading = ROTATE_FOR(lvl2, target["bearing"], heading)

    # Drive one step toward the target
    position, heading = drive_to_target(
        lvl2, target, position, heading,
        max_drive_mm=max_drive_mm, tolerance_mm=tolerance_mm
    )

    # Recompute distance to target to see if reached
    distance = target["distance"]
    found = distance <= tolerance_mm

    return found, position, heading