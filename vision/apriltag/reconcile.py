# vision/apriltag/reconcile.py

from __future__ import annotations

from config import CONFIG
from vision.apriltag.observations import AprilTagObservation


def _source_for_camera(camera_name: str) -> str | None:
    for source_id, cfg in CONFIG.vision_sources.items():
        if not cfg.get("enabled", True):
            continue

        if cfg["camera"] == camera_name:
            return source_id

    return None


def reconcile_apriltag_markers(
    *,
    camera_name: str,
    timestamp: float,
    markers,
) -> tuple[str | None, list[AprilTagObservation]]:

    source_id = _source_for_camera(camera_name)

    if source_id is None:
        return None, []

    observations: list[AprilTagObservation] = []

    for marker in markers:

        position = getattr(marker, "position", None)
        orientation = getattr(marker, "orientation", None)

        distance_mm = (
            getattr(position, "distance", None)
            if position is not None
            else None
        )

        horizontal_angle_rad = (
            getattr(position, "horizontal_angle", None)
            if position is not None
            else None
        )

        vertical_angle_rad = (
            getattr(position, "vertical_angle", None)
            if position is not None
            else None
        )

        yaw_rad = (
            getattr(orientation, "yaw", None)
            if orientation is not None
            else None
        )

        pitch_rad = (
            getattr(orientation, "pitch", None)
            if orientation is not None
            else None
        )

        roll_rad = (
            getattr(orientation, "roll", None)
            if orientation is not None
            else None
        )

        corners_px = getattr(
            marker,
            "corners_px",
            None,
        )

        observation = AprilTagObservation(
            source_id=source_id,
            camera=camera_name,
            timestamp=float(timestamp),

            tag_id=int(marker.id),

            distance_mm=distance_mm,
            horizontal_angle_rad=horizontal_angle_rad,
            vertical_angle_rad=vertical_angle_rad,

            yaw_rad=yaw_rad,
            pitch_rad=pitch_rad,
            roll_rad=roll_rad,

            center_px=getattr(
                marker,
                "center_px",
                None,
            ),

            corners_px=corners_px,

            # Only our Pi/USB marker model currently uses
            # size in metres together with pixel corners.
            tag_size_m=(
                getattr(marker, "size", None)
                if corners_px is not None
                else None
            ),

            decision_margin=getattr(
                marker,
                "decision_margin",
                None,
            ),

            family=getattr(
                marker,
                "family",
                None,
            ),

            tag_x_m=getattr(
                marker,
                "x_m",
                None,
            ),

            tag_y_m=getattr(
                marker,
                "y_m",
                None,
            ),

            tag_z_m=getattr(
                marker,
                "z_m",
                None,
            ),

            pose_err=getattr(
                marker,
                "pose_err",
                None,
            ),
        )

        observations.append(observation)

    return source_id, observations