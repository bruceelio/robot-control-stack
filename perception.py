# perception.py
import time
import math
from navigation.geometry import trilaterate_point
from navigation.arena import marker_locations
from config import distance_scale, ARENA_SIZE

# =========================
# Configuration
# =========================

ARENA_MARKER_MAX_ID = 19
ACIDIC_RANGE = range(100, 140)
BASIC_RANGE  = range(140, 180)

MEMORY_TIMEOUT = 5.0      # seconds before forgetting an object
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
        self.arena_markers = marker_locations(ARENA_SIZE)
        self.objects = {"acidic": {}, "basic": {}}
        self.last_pose = None

    def has_target(self, kind, grace_frames=3):
        memory = self.objects.get(kind, {})
        for obj in memory.values():
            if obj.get("age", 0) <= grace_frames:
                return True
        return False


def age_objects(perception: Perception):
    """
    Increment age (in frames) for all remembered objects.
    Seen objects will be reset to age=0 later in this frame.
    """
    for kind in perception.objects.values():
        for obj in kind.values():
            obj["age"] = obj.get("age", 0) + 1


# =========================
# Main sensing API
# =========================

def sense(robot, perception: Perception, stop_robot=True):
    """
    Stop robot (optional), read camera, classify markers,
    estimate pose, update memory, prune old objects.
    """
    now = time.time()

    age_objects(perception)

    if stop_robot:
        time.sleep(0.05)  # small pause for stability

    seen = robot.camera.see()  # returns list of markers

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
                C1, C2 = trilaterate_point(A, B, AC, BC)
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
    perception.last_pose = (x, y, 0.0)  # assume heading=0 if unknown

    log("POSE", f"x={x:.0f} y={y:.0f} using {len(arena_markers)} markers")
    return perception.last_pose

def inside_arena(pos):
    half = ARENA_SIZE / 2
    return -half <= pos[0] <= half and -half <= pos[1] <= half

# =========================
# Object tracking
# =========================

def update_objects(obj_type, markers, robot_pose, perception: Perception, now, distance_scale=1.0):
    if robot_pose is None:
        for m in markers:
            perception.objects[obj_type][m.id] = {
                "id": m.id,
                "marker": m,  # ← ADD
                "distance": float(m.position.distance),
                "bearing": math.degrees(float(m.position.horizontal_angle)),
                "last_seen": now,
                "age": 0,
                "relative": True,
            }

            log(obj_type.upper(), f"id={m.id} REL dist={m.position.distance:.0f}")
        return

    if obj_type not in perception.objects:
        perception.objects[obj_type] = {}

    rx, ry, rtheta = robot_pose  # robot's arena position and heading

    for m in markers:
        # Convert relative marker position to arena coordinates
        try:
            dx = float(m.position.distance) * math.cos(float(m.position.horizontal_angle)) * distance_scale
            dy = float(m.position.distance) * math.sin(float(m.position.horizontal_angle)) * distance_scale
        except Exception:
            dx, dy = 0.0, 0.0

        arena_dx = dx * math.cos(rtheta) - dy * math.sin(rtheta)
        arena_dy = dx * math.sin(rtheta) + dy * math.cos(rtheta)

        ax = rx + arena_dx
        ay = ry + arena_dy

        perception.objects[obj_type][m.id] = {
            "id": m.id,
            "marker": m,  # ← ADD
            "x": ax,
            "y": ay,
            "distance": math.hypot(ax - rx, ay - ry),
            "bearing": math.degrees(float(m.position.horizontal_angle)),
            "last_seen": now,
            "age": 0,
            "relative": False,
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
