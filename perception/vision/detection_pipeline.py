# hw_io/cameras/detection_pipeline.py

from __future__ import annotations

import math
from typing import Any, Iterable


def corrected_distance(marker: Any, cam_cal: Any) -> float:
    return float(marker.position.distance) * cam_cal.optical.distance_scale


def corrected_bearing_deg(marker: Any, cam_cal: Any) -> float:
    raw = math.degrees(float(marker.position.horizontal_angle))

    bearing = raw
    bearing *= cam_cal.optical.bearing_sign
    bearing += cam_cal.optical.bearing_offset_deg
    bearing += cam_cal.mount.yaw_offset_deg

    return bearing


def build_vision_message(
    *,
    camera_name: str,
    timestamp: float,
    markers: Iterable[Any],
    cam_cal: Any,
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
                "bearing_deg": corrected_bearing_deg(marker, cam_cal),
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


def arena_detections_from_message(message: dict) -> list[dict]:
    """
    Legacy compatibility wrapper.

    Prefer arena_detections_from_vision_message(...)
    in new code.
    """
    return arena_detections_from_vision_message(message)

def apriltag_observations_from_vision_message(vision_message: dict) -> list[dict]:
    observations = []

    camera_name = vision_message.get("camera", "unknown")
    timestamp = float(vision_message.get("timestamp", 0.0))

    for marker in vision_message.get("markers", []):
        observations.append(
            {
                "tag_id": int(marker.id),
                "camera": camera_name,
                "timestamp": timestamp,

                "distance_mm": marker.position.distance,
                "horizontal_angle_rad": marker.position.horizontal_angle,
                "vertical_angle_rad": marker.position.vertical_angle,

                "yaw_rad": marker.orientation.yaw,
                "pitch_rad": marker.orientation.pitch,
                "roll_rad": marker.orientation.roll,

                "center_px": marker.center_px,
                "corners_px": marker.corners_px,

                "tag_size_m": marker.size,
                "decision_margin": marker.decision_margin,
                "family": marker.family,

                "tag_x_m": marker.x_m,
                "tag_y_m": marker.y_m,
                "tag_z_m": marker.z_m,
                "pose_err": marker.pose_err,
            }
        )

    return observations