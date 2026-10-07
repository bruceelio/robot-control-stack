# perception/providers/pickup_face_resolver.py
"""Fresh, camera-corrected observations of individual faces of ONE cube.

perception.objects[kind][id] stores only one record for an ID, so it cannot
represent two AprilTags with the same ID on different faces. The canonical
latest_apriltag_observations list retains both observations instead.
"""

from __future__ import annotations

import math


from calibration import CALIBRATION


def resolve_pickup_faces(
    *,
    perception,
    target_id: int,
    camera_name: str = "front",
    max_age_s: float,
    now_s: float,
) -> list[dict]:
    """Return corrected, same-camera face observations for one selected ID.

    Each result has id, bearing_deg (+right), distance_mm, camera, timestamp.
    All results are from the newest camera frame, not object-memory entries.
    """
    if perception is None or target_id is None:
        return []

    now = float(now_s)
    cam = CALIBRATION.cameras[camera_name]
    results = []

    for obs in getattr(perception, "latest_apriltag_observations", ()):
        if int(obs.tag_id) != int(target_id) or obs.camera != camera_name:
            continue

        timestamp = float(obs.timestamp)
        age_s = now - timestamp
        if not math.isfinite(age_s) or age_s < -0.1 or age_s > max_age_s:
            continue

        if obs.horizontal_angle_rad is None or obs.distance_mm is None:
            continue

        # Use exactly the same optical corrections as perception.update_objects:
        # corrected_bearing_deg(m, cam).
        bearing = (
            math.degrees(float(obs.horizontal_angle_rad))
            * float(cam.optical.bearing_sign)
            + float(cam.optical.bearing_offset_deg)
        )
        distance = (
            float(obs.distance_mm)
            * float(cam.optical.distance_scale)
        )

        if (
            not math.isfinite(bearing)
            or not math.isfinite(distance)
            or distance <= 0.0
        ):
            continue

        results.append({
            "id": int(obs.tag_id),
            "bearing_deg": bearing,
            "distance_mm": distance,
            "camera": camera_name,
            "timestamp": timestamp,
        })

    return sorted(results, key=lambda face: face["bearing_deg"])
