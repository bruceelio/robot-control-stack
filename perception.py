# perception.py
import time
import math
from typing import Dict, List, Tuple

from navigation.geometry import trilaterate_point
from config import CONFIG
from config.arena import marker_locations
from calibration import CALIBRATION


# ==================================================
# Configuration
# ==================================================

ARENA_MARKER_MAX_ID = 19
ACIDIC_RANGE = range(100, 140)
BASIC_RANGE  = range(140, 180)

MEMORY_TIMEOUT = 5.0      # seconds
DEBUG = True

# Select primary camera for now
PRIMARY_CAMERA = "front"


# ==================================================
# Simple logger
# ==================================================

def log(tag, msg):
    if DEBUG:
        print(f"[{tag}] {msg}")


# ==================================================
# Perception State
# ==================================================

class Perception:
    def __init__(self):
        self.arena_markers = marker_locations(CONFIG.arena_size)
        self.objects = {"acidic": {}, "basic": {}}
        self.last_pose = None


# ==================================================
# Object aging
# ==================================================

def age_objects(perception: Perception):
    for kind in perception.objects.values():
        for obj in kind.values():
            obj["age"] = obj.get("age", 0) + 1


# ==================================================
# Main sensing entry point
# ==================================================

def sense(robot, perception: Perception, stop_robot=True):
    now = time.time()
    age_objects(perception)

    if stop_robot:
        time.sleep(0.05)

    # --------------------------------------------------
    # Camera selection & calibration
    # --------------------------------------------------

    if PRIMARY_CAMERA not in CALIBRATION.cameras:
        raise RuntimeError(f"Camera '{PRIMARY_CAMERA}' not found in calibration")

    cam_cal = CALIBRATION.cameras[PRIMARY_CAMERA]

    # NOTE: robot.camera is assumed to correspond to PRIMARY_CAMERA
    seen = robot.camera.see()

    arena_markers, acidic_markers, basic_markers = classify_markers(seen)

    pose = estimate_pose(arena_markers, perception, cam_cal)

    update_objects("acidic", acidic_markers, pose, perception, now, cam_cal)
    update_objects("basic",  basic_markers,  pose, perception, now, cam_cal)

    prune_objects(perception, now)

    log(
        "PERCEPTION",
        f"Seen total={len(seen)} arena={len(arena_markers)} "
        f"acidic={len(acidic_markers)} basic={len(basic_markers)} "
        f"pose={'OK' if pose else 'FAIL'}"
    )

    return pose, perception.objects


# ==================================================
# Marker classification
# ==================================================

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


# ==================================================
# Camera-corrected measurement helpers
# ==================================================

def corrected_distance(m, cam):
    return float(m.position.distance) * cam.optical.distance_scale


def corrected_bearing_deg(m, cam):
    raw = math.degrees(float(m.position.horizontal_angle))

    bearing = raw
    bearing *= cam.optical.bearing_sign
    bearing += cam.optical.bearing_offset_deg
    bearing += cam.mount.yaw_offset_deg

    return bearing


# ==================================================
# Pose estimation (arena markers)
# ==================================================

def estimate_pose(arena_markers, perception: Perception, cam):
    if len(arena_markers) < 2:
        return None

    positions = []

    for i in range(len(arena_markers)):
        for j in range(i + 1, len(arena_markers)):
            m1, m2 = arena_markers[i], arena_markers[j]

            A = perception.arena_markers[m1.id]
            B = perception.arena_markers[m2.id]

            AC = corrected_distance(m1, cam)
            BC = corrected_distance(m2, cam)

            try:
                C1, C2 = trilaterate_point(A, B, AC, BC)
                for C in (C1, C2):
                    if inside_arena(C):
                        positions.append(C)
            except ValueError:
                continue

    if not positions:
        log("WARN", "Pose estimation failed")
        return None

    x = sum(p[0] for p in positions) / len(positions)
    y = sum(p[1] for p in positions) / len(positions)

    perception.last_pose = (x, y, 0.0)

    log("POSE", f"x={x:.0f} y={y:.0f} using {len(arena_markers)} markers")
    return perception.last_pose


def inside_arena(pos):
    half = CONFIG.arena_size / 2
    return -half <= pos[0] <= half and -half <= pos[1] <= half


# ==================================================
# Object tracking
# ==================================================

def update_objects(kind, markers, robot_pose, perception: Perception, now, cam):
    memory = perception.objects[kind]

    for m in markers:
        dist = corrected_distance(m, cam)
        bearing_deg = corrected_bearing_deg(m, cam)
        bearing_rad = math.radians(bearing_deg)

        if robot_pose is None:
            memory[m.id] = {
                "id": m.id,
                "marker": m,
                "distance": dist,
                "bearing": bearing_deg,
                "last_seen": now,
                "age": 0,
                "relative": True,
            }
            log(kind.upper(), f"id={m.id} REL dist={dist:.0f}")
            continue

        rx, ry, rtheta = robot_pose

        dx = dist * math.cos(bearing_rad)
        dy = dist * math.sin(bearing_rad)

        arena_dx = dx * math.cos(rtheta) - dy * math.sin(rtheta)
        arena_dy = dx * math.sin(rtheta) + dy * math.cos(rtheta)

        ax = rx + arena_dx
        ay = ry + arena_dy

        memory[m.id] = {
            "id": m.id,
            "marker": m,
            "x": ax,
            "y": ay,
            "distance": math.hypot(ax - rx, ay - ry),
            "bearing": bearing_deg,
            "last_seen": now,
            "age": 0,
            "relative": False,
        }

        log(kind.upper(), f"id={m.id} pos=({ax:.1f}, {ay:.1f})")


# ==================================================
# Memory pruning
# ==================================================

def prune_objects(perception: Perception, now):
    for kind in ("acidic", "basic"):
        memory = perception.objects[kind]
        to_remove = [
            mid for mid, data in memory.items()
            if now - data["last_seen"] > MEMORY_TIMEOUT
        ]
        for mid in to_remove:
            log(kind.upper(), f"id={mid} LOST")
            del memory[mid]


# ==================================================
# Target access helper
# ==================================================

def get_visible_targets(perception: Perception, kind: str):
    targets = list(perception.objects.get(kind, {}).values())
    targets.sort(key=lambda t: t["distance"])
    return targets
