# diagnostics/apriltag_pose_check.py

from __future__ import annotations

import math
import time

from config import CONFIG
from config.arena import marker_locations
from localisation.localisation import Localisation
from localisation.providers.vision.pose_cam1_markers2 import Cam1Markers2Provider

CAMERA_NAME = "front"
LOOP_DELAY_S = 0.2


def _deg(rad):
    if rad is None:
        return None
    return float(rad) * (180.0 / math.pi)


def _safe_round(value, digits=1):
    if value is None:
        return None
    return round(float(value), digits)


def _extract_distance_mm(marker):
    pos = getattr(marker, "position", None)
    if pos is None:
        return None

    dist = getattr(pos, "distance", None)
    if dist is None:
        return None

    # Assumes the camera already returns millimetres.
    # If it returns metres instead, change to:
    # return float(dist) * 1000.0
    return float(dist)


def _extract_bearing_deg(marker):
    pos = getattr(marker, "position", None)
    if pos is None:
        return 0.0

    bearing = getattr(pos, "horizontal_angle", None)
    if bearing is None:
        return 0.0

    return _deg(bearing) or 0.0


def _to_arena_detection(marker):
    marker_id = int(getattr(marker, "id", -1))
    distance_mm = _extract_distance_mm(marker)
    bearing_deg = _extract_bearing_deg(marker)

    if distance_mm is None or distance_mm <= 0.0:
        return None

    return {
        "id": marker_id,
        "distance_mm": distance_mm,
        "bearing_deg": bearing_deg,
        "camera": CAMERA_NAME,
    }


def _pose_tuple(obs):
    if obs is None:
        return None

    return (
        _safe_round(obs.x, 1),
        _safe_round(obs.y, 1),
        _safe_round(_deg(obs.heading), 1),
        _safe_round(obs.confidence, 2),
        getattr(obs, "source", None),
        getattr(obs, "quality", None),
    )


def _state_tuple(
    visible_ids,
    arena_ids,
    object_ids,
    arena_detections,
    provider_pose,
    localisation_pose,
):
    return (
        tuple(visible_ids),
        tuple(arena_ids),
        tuple(object_ids),
        tuple(
            (
                d["id"],
                round(d["distance_mm"], 1),
                round(d["bearing_deg"], 1),
            )
            for d in arena_detections
        ),
        provider_pose,
        localisation_pose,
    )


def _print_pose(label, obs):
    if obs is None:
        print(f"{label}: unavailable")
        return

    x, y, h, c, source, quality = _pose_tuple(obs)
    print(
        f"{label}: x={x} y={y} heading={h} deg "
        f"conf={c} source={source} quality={quality}"
    )

    diagnostics = getattr(obs, "diagnostics", None)
    if diagnostics:
        print(f"{label}_diag: {diagnostics}")


def run(robot, io):
    print("\n=== APRILTAG VISIBILITY + POSE DIAGNOSTIC ===")
    print("Press Ctrl+C to stop")

    cams = io.cameras()
    print(f"[DIAG] Available cameras: {list(cams.keys())}")

    cam = cams.get(CAMERA_NAME)
    if cam is None:
        print(f"[DIAG] No camera named {CAMERA_NAME!r}")
        return

    arena_marker_map = marker_locations(float(CONFIG.arena_size))
    arena_ids_set = set(arena_marker_map.keys())

    localisation = Localisation()
    direct_provider = Cam1Markers2Provider()

    last_state = None

    try:
        while True:
            now_s = time.time()
            seen = cam.see() or []

            visible_ids = sorted(int(getattr(m, "id", -1)) for m in seen)

            arena_detections = []
            arena_ids = []
            object_ids = []

            for marker in seen:
                marker_id = int(getattr(marker, "id", -1))

                if marker_id in arena_ids_set:
                    arena_ids.append(marker_id)
                    det = _to_arena_detection(marker)
                    if det is not None:
                        arena_detections.append(det)
                else:
                    object_ids.append(marker_id)

            arena_ids = sorted(arena_ids)
            object_ids = sorted(object_ids)

            direct_provider.set_detections(arena_detections)
            provider_obs = direct_provider.get_observation(now_s=now_s)

            localisation_obs = localisation.estimate(
                now_s=now_s,
                io=io,
                arena_detections=arena_detections,
            )

            current_state = _state_tuple(
                visible_ids=visible_ids,
                arena_ids=arena_ids,
                object_ids=object_ids,
                arena_detections=arena_detections,
                provider_pose=_pose_tuple(provider_obs),
                localisation_pose=_pose_tuple(localisation_obs),
            )

            if current_state != last_state:
                print("\n--- update ---")
                print(f"visible: {visible_ids}")
                print(f"arena:   {arena_ids}")
                print(f"objects: {object_ids}")
                print(f"arena_detections: {arena_detections}")

                if len(arena_detections) < 2:
                    print("note: fewer than 2 usable arena detections")
                else:
                    print("note: 2+ usable arena detections present")

                _print_pose("direct_provider_pose", provider_obs)
                _print_pose("localisation_pose", localisation_obs)

                last_state = current_state

            time.sleep(LOOP_DELAY_S)

    except KeyboardInterrupt:
        print("\n[DIAG] stopped by user")