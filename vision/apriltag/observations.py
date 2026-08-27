# vision/apriltag/observations.py

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class AprilTagObservation:
    source_id: str
    camera: str
    timestamp: float

    tag_id: int

    # Common geometric information
    distance_mm: float | None = None
    horizontal_angle_rad: float | None = None
    vertical_angle_rad: float | None = None

    yaw_rad: float | None = None
    pitch_rad: float | None = None
    roll_rad: float | None = None

    # Pixel/image information.
    # May not exist for SR/Webots observations.
    center_px: Any = None
    corners_px: Any = None

    # Tag/detector metadata.
    tag_size_m: float | None = None
    decision_margin: float | None = None
    family: str | None = None

    # Optional detector/PnP relative-position information.
    tag_x_m: float | None = None
    tag_y_m: float | None = None
    tag_z_m: float | None = None
    pose_err: float | None = None