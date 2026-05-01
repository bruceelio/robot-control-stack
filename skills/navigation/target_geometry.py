from __future__ import annotations

import math
from typing import Any


def target_from_gripper(*, distance_mm: float, bearing_deg: float, config):
    bearing_rad = math.radians(float(bearing_deg))

    target_x_cam = float(distance_mm) * math.cos(bearing_rad)

    # Detector/control convention appears to be:
    #   +bearing = right
    # Robot geometry convention is:
    #   +y = left
    target_y_cam = -float(distance_mm) * math.sin(bearing_rad)

    grip = config.gripper_from_camera
    grip_x_cam = float(grip["x_mm"])
    grip_y_cam = float(grip["y_mm"])

    target_x_grip = target_x_cam - grip_x_cam
    target_y_grip = target_y_cam - grip_y_cam

    grip_distance = math.hypot(target_x_grip, target_y_grip)

    # Convert +y-left geometry back into +bearing-right command convention.
    grip_bearing = -math.degrees(math.atan2(target_y_grip, target_x_grip))

    return grip_distance, grip_bearing