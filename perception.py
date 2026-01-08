# perception.py
import time
import math
from location import find_location, marker_location
from config import distance_scale, ARENA_SIZE

# =========================
# Configuration
# =========================

ARENA_MARKER_MAX_ID = 19
ACIDIC_RANGE = range(100, 140)
BASIC_RANGE  = range(140, 180)

MEMORY_TIMEOUT = 5.0      # seconds before forgetting an object
POSITION_JUMP_MM = 300    # warn if object jumps this much
DEBUG = True

# =========================
# Simple logger
# =========================

def log(tag, msg):
    if DEBUG:
        print(f"[{tag}] {msg}")

# =========================
# Perception State
# =========================

class Perception:
    def __init__(self):
        self.arena_markers = marker_location(ARENA_SIZE)
        self.objects = {"acidic": {}, "basic": {}}
        self.last_pose = None

# =========================
# Main sensing API
# =========================

def sense(robot, perception: Perception, stop_robot=True):
    """
    Stop robot (optional), read camera, classify markers,
    estimate pose, update memory, prune old objects.
    """
    now = time.time()

    # 1. Stop robot for accurate bearings
    if stop_robot:
        time.sleep(0.05)  # small pause to let the simulator stabilize

    seen = robot.camera.see()  # returns list of markers
    if seen:
        m = seen[0]
        print("DEBUG MARKER dir:", dir(m))  # list all attributes
        print("DEBUG MARKER id:", m.id)
        print("DEBUG MARKER position:", m.position)
        # If position has attributes, print them too
        if hasattr(m.position, "x") and hasattr(m.position, "y"):
            print("DEBUG MARKER position x,y:", m.position.x, m.position.y)
        if hasattr(m.position, "distance") and hasattr(m.position, "angle"):
            print("DEBUG MARKER distance,angle:", m.position.distance, m.position.angle)

    arena_markers, acidic_markers, basic_markers = classify_markers(seen)
    pose = estimate_pose(arena_markers, perception)

    update_objects("acidic", acidic_markers, pose, perception, now)
    update_objects("basic", basic_markers, pose, perception, now)
    prune_objects(perception, now)

    log(
        "PERCEPTION",
        f"Seen total={len(seen)} arena={len(arena_markers)} "
        f"acidic={len(acidic_markers)} basic={len(basic_markers)} "
        f"pose={'OK' if pose else 'FAIL'}"
    )

    return pose, perception.objects

# =========================
# Marker classification
# =========================

def classify_markers(markers):
    arena, acidic, basic = [], [], []
    for m in markers:
        if m.id <= ARENA_MARKER_MAX_ID:
            arena.append(m)
        elif m.id in ACIDIC_RANGE:
            acidic.append(m)
        elif m.id in BASIC_RANGE:
            basic.append(m)
    return arena, acidic, basic

# =========================
# Pose estimation
# =========================

def estimate_pose(arena_markers, perception: Perception):
    if len(arena_markers) < 2:
        return None

    positions = []
    for i in range(len(arena_markers)):
        for j in range(i + 1, len(arena_markers)):
            m1, m2 = arena_markers[i], arena_markers[j]
            A = perception.arena_markers[m1.id]
            B = perception.arena_markers[m2.id]
            AC = m1.position.distance * distance_scale
            BC = m2.position.distance * distance_scale

            try:
                C1, C2 = find_location(A, B, AC, BC)
                for C in (C1, C2):
                    if inside_arena(C):
                        positions.append(C)
            except ValueError:
                continue

    if not positions:
        log("WARN", "Pose estimation failed with arena markers")
        return None

    x = sum(p[0] for p in positions) / len(positions)
    y = sum(p[1] for p in positions) / len(positions)
    perception.last_pose = (x, y)

    log("POSE", f"x={x:.0f} y={y:.0f} using {len(arena_markers)} markers")
    return perception.last_pose

def inside_arena(pos):
    half = ARENA_SIZE / 2
    return -half <= pos[0] <= half and -half <= pos[1] <= half

# =========================
# Object tracking
# =========================

def get_marker_relative_xy(m):
    """
    Returns (dx, dy) in robot coordinates as floats.
    Works with both old distance/angle or new x/y API.
    """
    pos = m.position

    # If x/y exist
    if hasattr(pos, "x") and hasattr(pos, "y"):
        return float(pos.x), float(pos.y)

    # Fallback to distance/angle
    if hasattr(pos, "distance") and hasattr(pos, "angle"):
        angle_rad = math.radians(float(pos.angle or 0.0))
        dx = float(pos.distance) * math.cos(angle_rad)
        dy = float(pos.distance) * math.sin(angle_rad)
        return dx, dy

    # Last resort: assume straight ahead
    try:
        return float(pos), 0.0
    except Exception:
        return 0.0, 0.0


def update_objects(obj_type, markers, robot_pose, perception: Perception, now, distance_scale=1.0):
    """
    Updates the perception dictionary for objects of type `obj_type`.
    Supports new Marker.position API.
    """
    if robot_pose is None:
        # Can't compute arena positions without robot pose
        log("WARN", f"Skipping {obj_type} update — no robot pose")
        return

    if obj_type not in perception.objects:
        perception.objects[obj_type] = {}

    rx, ry = robot_pose[0], robot_pose[1]
    rtheta = robot_pose[2] if len(robot_pose) > 2 else 0.0  # default heading

    for m in markers:
        # Get dx/dy safely
        try:
            dx = float(m.position.distance) * math.cos(float(m.position.horizontal_angle)) * distance_scale
            dy = float(m.position.distance) * math.sin(float(m.position.horizontal_angle)) * distance_scale
        except Exception:
            dx, dy = 0.0, 0.0

        # Rotate to arena coordinates
        arena_dx = dx * math.cos(rtheta) - dy * math.sin(rtheta)
        arena_dy = dx * math.sin(rtheta) + dy * math.cos(rtheta)

        ax = rx + arena_dx
        ay = ry + arena_dy

        perception.objects[obj_type][m.id] = {
            "x": ax,
            "y": ay,
            "last_seen": now
        }

        log(obj_type.upper(), f"id={m.id} pos=({ax:.1f}, {ay:.1f})")


def prune_objects(perception: Perception, now):
    for kind in ("acidic", "basic"):
        memory = perception.objects[kind]
        to_remove = [mid for mid, data in memory.items() if now - data["last_seen"] > MEMORY_TIMEOUT]
        for mid in to_remove:
            log(kind.upper(), f"id={mid} LOST (timeout)")
            del memory[mid]

# =========================
# Helper for getting visible targets
# =========================

def get_visible_targets(perception: Perception, kind: str):
    memory = perception.objects.get(kind, {})
    targets = list(memory.values())
    targets.sort(key=lambda t: t["distance"])
    return targets
