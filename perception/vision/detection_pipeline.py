# perception/vision/detection_pipeline.py

from __future__ import annotations

import math
from typing import Any, Iterable


def corrected_distance(marker: Any, cam_cal: Any) -> float:
    return float(marker.position.distance) * cam_cal.optical.distance_scale


def corrected_bearing_deg(
    marker: Any,
    cam_cal: Any,
    camera_yaw_deg: float,
) -> float:
    raw = math.degrees(float(marker.position.horizontal_angle))

    bearing = raw
    bearing *= cam_cal.optical.bearing_sign
    bearing += cam_cal.optical.bearing_offset_deg
    bearing += camera_yaw_deg

    return bearing


def build_vision_message(
    *,
    camera_name: str,
    timestamp: float,
    markers: Iterable[Any],
    cam_cal: Any,
    camera_yaw_deg: float,
    status: str = "ok",
) -> dict:
    detections = []

    for marker in markers:
        if marker.position.distance is None:
            continue

        if marker.position.horizontal_angle is None:
            continue

        detections.append(
            {
                "id": int(marker.id),
                "distance_mm": corrected_distance(marker, cam_cal),
                "bearing_deg": corrected_bearing_deg(
                    marker,
                    cam_cal,
                    camera_yaw_deg,
                ),
                "camera": camera_name,
            }
        )

    return {
        "camera": camera_name,
        "timestamp": float(timestamp),
        "detections": detections,
        "status": status,
    }


def arena_detections_from_vision_message(vision_message: dict) -> list[dict]:
    return list(vision_message.get("detections", []))



