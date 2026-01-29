# perception.py
import time
import math
import hashlib
from calibration import CALIBRATION
from hw_io.base import IOMap

# ==================================================
# Configuration
# ==================================================

ARENA_MARKER_MAX_ID = 19
ACIDIC_RANGE = range(100, 140)
BASIC_RANGE  = range(140, 180)

MEMORY_TIMEOUT = 5.0      # seconds
DEBUG = True

# ==================================================
# Log throttling tuning
# ==================================================
LOG_DEFAULT_INTERVAL_S   = 1.0
LOG_SEEN_INTERVAL_S      = 1.0
LOG_CAMLIST_INTERVAL_S   = 10.0


# Select primary camera for now
PRIMARY_CAMERA = "front"

# ==================================================
# Frame-order logging helper
# ==================================================

_FRAME_ORDER_BUFFER = []


# ==================================================
# Simple logger
# ==================================================

def log(tag, msg):
    if DEBUG:
        print(f"[{tag}] {msg}")

_LOG_STATE = {}  # key -> {"t": float, "sig": str}

def _sig(s: str) -> str:
    # stable-ish short signature for change detection
    return hashlib.sha1(s.encode("utf-8")).hexdigest()

def log_throttled(
    tag: str,
    msg: str,
    *,
    key: str | None = None,
    now: float | None = None,
    min_interval_s: float = LOG_DEFAULT_INTERVAL_S,
    change_only: bool = False,
):
    """
    Throttle logs per (tag,key).
      - If change_only=True: only log when message signature differs.
      - Always allow logging if (now - last_time) >= min_interval_s.
    """
    if not DEBUG:
        return
    if now is None:
        now = time.time()
    if key is None:
        key = tag  # stable default
    # fallback (still provides throttling if message repeats)

    state_key = (tag, key)
    entry = _LOG_STATE.get(state_key)
    sig = _sig(msg)

    if entry is None:
        _LOG_STATE[state_key] = {"t": now, "sig": sig}
        log(tag, msg)
        return

    elapsed = now - entry["t"]
    changed = (sig != entry["sig"])

    should_log = False

    if change_only:
        # "change OR interval"
        should_log = changed or (elapsed >= min_interval_s)
    else:
        # "interval"
        should_log = elapsed >= min_interval_s

    if should_log:
        entry["t"] = now
        entry["sig"] = sig
        log(tag, msg)


# ==================================================
# Perception State
# ==================================================

class Perception:
    def __init__(self, io: IOMap):
        self.io = io
        self.objects = {"acidic": {}, "basic": {}}


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

def sense(io: IOMap, perception: Perception, stop_robot=True):
    now = time.time()
    _FRAME_ORDER_BUFFER.clear()
    age_objects(perception)

    if stop_robot:
        time.sleep(0.05)

    # --------------------------------------------------
    # Camera selection & calibration
    # --------------------------------------------------

    # Camera selection & calibration
    calibrated = set(CALIBRATION.cameras.keys())
    available = set(io.cameras().keys())

    missing_in_io = calibrated - available
    missing_in_cal = available - calibrated

    # Camera lists: mostly static, so log on change (or very infrequently)
    log_throttled(
        "PERCEPTION",
        f"Calibrated cameras: {sorted(calibrated)}",
        key="cam_calibrated",
        now=now,
        min_interval_s=LOG_CAMLIST_INTERVAL_S,
        change_only=True,
    )
    log_throttled(
        "PERCEPTION",
        f"Available cameras: {sorted(available)}",
        key="cam_available",
        now=now,
        min_interval_s=LOG_CAMLIST_INTERVAL_S,
        change_only=True,
    )

    if PRIMARY_CAMERA not in calibrated:
        raise RuntimeError(
            f"PRIMARY_CAMERA={PRIMARY_CAMERA!r} not in calibration. "
            f"Calibrated={sorted(calibrated)}"
        )

    if PRIMARY_CAMERA not in available:
        raise RuntimeError(
            f"PRIMARY_CAMERA={PRIMARY_CAMERA!r} not in IO. "
            f"Available={sorted(available)}"
        )

    # Optional: strict mode (recommended once you add a 2nd camera)
    if missing_in_io:
        raise RuntimeError(f"Calibrated cameras missing in IO: {sorted(missing_in_io)}")
    # if missing_in_cal:
    #     raise RuntimeError(f"IO cameras missing calibration: {sorted(missing_in_cal)}")

    cam_cal = CALIBRATION.cameras[PRIMARY_CAMERA]
    seen = io.cameras()[PRIMARY_CAMERA].see()

    # --------------------------------------------------

    arena_markers, acidic_markers, basic_markers = classify_markers(seen)

    # Export arena-marker measurements for localisation providers
    arena_observations = build_arena_observations(arena_markers, cam_cal)

    # For now, keep object updates relative unless an external pose is provided elsewhere
    pose = None

    update_objects("acidic", acidic_markers, pose, perception, now, cam_cal)
    update_objects("basic", basic_markers, pose, perception, now, cam_cal)

    # Log current seen markers left -> right (most negative bearing first)
    if DEBUG and _FRAME_ORDER_BUFFER:
        _FRAME_ORDER_BUFFER.sort(key=lambda r: r["bearing"])
        # Build a stable signature of the frame's visible markers.
        # This lets us log the whole set only when it changes (or on an interval).
        frame_lines = []
        for r in _FRAME_ORDER_BUFFER:
            frame_lines.append(
                f"id={r['id']} kind={r['kind']} "
                f"dist={r['distance']:.0f} "
                f"bearing={r['bearing']:.1f}deg "
                f"va={r['va_deg']:.2f}deg"
            )
        seen_blob = "\n".join(frame_lines)
        log_throttled(
            "SEEN",
            seen_blob,
            key="frame_seen_set",
            now=now,
            min_interval_s=LOG_SEEN_INTERVAL_S,
            change_only=True,
        )

    prune_objects(perception, now)

    # Summary line: log at most once per second unless it changes.
    log_throttled(
        "PERCEPTION",
        f"Seen total={len(seen)} arena={len(arena_markers)} "
        f"acidic={len(acidic_markers)} basic={len(basic_markers)} "
        f"pose={'OK' if pose else 'FAIL'}",
        key="perception_summary",
        now=now,
        min_interval_s=LOG_DEFAULT_INTERVAL_S,
        change_only=False,
    )

    # NEW return shape: (arena_observations, objects)
    return arena_observations, perception.objects


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

def build_arena_observations(arena_markers, cam):
    """
    Convert raw arena marker detections into normalised observations suitable for localisation.

    Returns list[dict] with keys:
      id, distance_mm, bearing_deg, camera
    """
    obs = []
    for m in arena_markers:
        obs.append({
            "id": int(m.id),
            "distance_mm": corrected_distance(m, cam),
            "bearing_deg": corrected_bearing_deg(m, cam),
            "camera": PRIMARY_CAMERA,
        })
    return obs

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
# Object tracking
# ==================================================

def update_objects(kind, markers, robot_pose, perception: Perception, now, cam):
    memory = perception.objects[kind]

    for m in markers:
        dist = corrected_distance(m, cam)
        bearing_deg = corrected_bearing_deg(m, cam)
        bearing_rad = math.radians(bearing_deg)

        # If we don't have a usable heading, keep targets in robot-relative coordinates
        if robot_pose is None or len(robot_pose) < 3 or robot_pose[2] is None:
            va_rad = float(m.position.vertical_angle)
            va_deg = math.degrees(va_rad)

            memory[m.id] = {
                "id": m.id,
                "marker": m,
                "kind": kind,
                "distance": dist,
                "bearing": bearing_deg,
                "vertical_angle_rad": va_rad,
                "vertical_angle_deg": va_deg,
                "last_seen": now,
                "age": 0,
                "relative": True,
                "camera": PRIMARY_CAMERA,  # or "front" if you prefer
            }

            _FRAME_ORDER_BUFFER.append({
                "id": int(m.id),
                "kind": kind,
                "bearing": float(bearing_deg),
                "distance": float(dist),
                "va_deg": float(va_deg),
            })
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

        log_throttled(
            kind.upper(),
            f"id={m.id} pos=({ax:.1f}, {ay:.1f})",
            key=f"{kind}_pos_{int(m.id)}",
            now=now,
            min_interval_s=LOG_DEFAULT_INTERVAL_S,
            change_only=True,  # log when moved, otherwise once/sec
        )


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

def get_visible_targets(perception: Perception, kind: str, *, now: float | None = None, max_age_s: float = 0.35):
    """
    Only return targets seen very recently (vision-visible),
    not just anything still in memory.
    """
    if now is None:
        now = time.time()

    targets = [
        t for t in perception.objects.get(kind, {}).values()
        if (now - t.get("last_seen", 0.0)) <= max_age_s
    ]
    targets.sort(key=lambda t: t["distance"])
    return targets

