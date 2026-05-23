# localisation/providers/vision/apriltag_observations.py

from config import CONFIG


def apriltag_observations_by_source(
    vision_message: dict,
) -> tuple[str | None, list[dict]]:

    camera_name = vision_message.get("camera")

    source_id = None

    for vision_key, cfg in CONFIG.vision_sources.items():
        if cfg["camera"] == camera_name:
            source_id = vision_key
            break

    if source_id is None:
        return None, []

    observations = []

    timestamp = float(vision_message.get("timestamp", 0.0))

    for marker in vision_message.get("markers", []):

        observations.append(
            {
                "source_id": source_id,
                "camera": camera_name,
                "timestamp": timestamp,

                "tag_id": int(marker.id),

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

                "tag_x_m": marker.x_m,
                "tag_y_m": marker.y_m,
                "tag_z_m": marker.z_m,
                "pose_err": marker.pose_err,
            }
        )

    print(
        f"[VISION_SOURCE] "
        f"camera={camera_name} "
        f"source={source_id} "
        f"observations={len(observations)}"
    )

    return source_id, observations