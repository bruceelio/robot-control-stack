# navigation.py
from motion import drive_distance, rotate_angle

# =========================
# Target selection
# =========================

def get_closest_target(perception, kind="acidic"):
    memory = perception.objects.get(kind, {})
    if not memory:
        return None

    # Always prefer closest by distance (relative or global)
    targets = sorted(memory.values(), key=lambda t: t["distance"])
    return targets[0]


# =========================
# Navigation utilities
# =========================

def drive_to_target(robot, target, max_drive_mm=500, tolerance_mm=50):
    """
    Drive toward a target object using bearing and distance.

    target: dict with 'distance' in mm and 'bearing' in degrees
    """
    distance = target["distance"]
    bearing = target["bearing"]

    # Rotate to face the target
    if abs(bearing) > 2.0:
        rotate_angle(robot, bearing)

    # Drive forward
    if distance > tolerance_mm:
        drive_distance(robot, min(distance, max_drive_mm))

# =========================
# High-level navigation
# =========================

def seek_and_collect(robot, perception, kind="acidic"):
    """
    Seek the closest target of the given kind and move toward it.
    Returns True if a target was found and approached.
    """
    target = get_closest_target(perception, kind)
    if target is None:
        return False

    drive_to_target(robot, target)
    return True
