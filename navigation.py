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

def drive_to_target(lvl2, target, position, heading, max_drive_mm=300, tolerance_mm=50):
    """
    Drive toward a target object using bearing and distance.

    target: dict with 'distance' in mm and 'bearing' in degrees
    """
    distance = target["distance"]
    bearing = target["bearing"]

    # Rotate to face the target
    if abs(bearing) > 2.0:
        heading = ROTATE_FOR(lvl2, bearing, heading)

    # Drive forward
    if distance > tolerance_mm:
        dx, dy = DRIVE_FOR(lvl2, min(distance, max_drive_mm), heading)
        position = (position[0] + dx, position[1] + dy)

    return position, heading

# =========================
# High-level navigation
# =========================

def seek_and_collect(lvl2, perception, position, heading, kind="acidic"):
    """
    Seek the closest target of the given kind and move toward it.
    Returns (found, position, heading)
    """
    target = get_closest_target(perception, kind)
    if target is None:
        return False, position, heading

    position, heading = drive_to_target(lvl2, target, position, heading)
    return True, position, heading
