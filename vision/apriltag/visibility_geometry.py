# vision/apriltag/visibility_geometry.py

from __future__ import annotations

from dataclasses import dataclass
import math

from config.arena_tags import resolve_tag_size_m


@dataclass(frozen=True)
class HorizontalTagFeature:
    """
    One horizontal visibility feature of an AprilTag.

    Camera coordinates:
        +x = forward from camera
        +y = camera left
        +bearing = camera right

    Base coordinates:
        +x = robot forward
        +y = robot left
    """

    x_m: float
    y_m: float
    base_x_m: float
    base_y_m: float
    bearing_rad: float
    range_m: float


@dataclass(frozen=True)
class HorizontalTagVisibility:
    """
    Horizontal image-space extent of one AprilTag.

    left/right refer to image position, not physical tag sides.
    """

    centre: HorizontalTagFeature
    left: HorizontalTagFeature
    right: HorizontalTagFeature

    @property
    def most_endangered(self) -> HorizontalTagFeature:
        """
        Return the horizontal feature closest to either FOV boundary.
        """

        return max(
            (self.left, self.right),
            key=lambda feature: abs(
                feature.bearing_rad
            ),
        )


def _camera_xy_to_base(
    *,
    x_m: float,
    y_m: float,
    mount: dict,
) -> tuple[float, float]:
    mount_x_m = float(mount["x_mm"]) / 1000.0
    mount_y_m = float(mount["y_mm"]) / 1000.0

    mount_yaw_rad = math.radians(
        float(
            mount.get(
                "yaw_deg",
                0.0,
            )
        )
    )

    cos_yaw = math.cos(mount_yaw_rad)
    sin_yaw = math.sin(mount_yaw_rad)

    base_x_m = (
        mount_x_m
        + cos_yaw * x_m
        - sin_yaw * y_m
    )

    base_y_m = (
        mount_y_m
        + sin_yaw * x_m
        + cos_yaw * y_m
    )

    return base_x_m, base_y_m

def _feature_from_xy(
    *,
    x_m: float,
    y_m: float,
    mount: dict,
) -> HorizontalTagFeature:
    if x_m <= 0.0:
        raise ValueError(
            "projected AprilTag feature is behind the camera"
        )

    base_x_m, base_y_m = _camera_xy_to_base(
        x_m=x_m,
        y_m=y_m,
        mount=mount,
    )

    return HorizontalTagFeature(
        x_m=x_m,
        y_m=y_m,
        base_x_m=base_x_m,
        base_y_m=base_y_m,
        bearing_rad=-math.atan2(
            y_m,
            x_m,
        ),
        range_m=math.hypot(
            x_m,
            y_m,
        ),
    )

def horizontal_tag_visibility(
    *,
    tag_id: int,
    observation: dict,
    config,
) -> HorizontalTagVisibility:
    """
    Project the horizontal extent of an AprilTag.

    Tag size is resolved from arena configuration.

    The current horizontal model uses:
        centre distance
        horizontal bearing
        vertical angle
        tag yaw
        configured tag size

    For the currently selected SR object faces, pitch and roll are
    near zero, so the two vertical tag edges define the horizontal
    extrema of all four tag corners.

    This module performs geometry only. It does not know anything
    about PBVS, Smooth Control, FOV limits or controller policy.
    """

    tag_size_m = float(
        resolve_tag_size_m(
            int(tag_id)
        )
    )

    if tag_size_m <= 0.0:
        raise ValueError(
            "configured AprilTag size must be > 0"
        )

    camera_name = observation.get("camera")

    if not camera_name:
        raise ValueError(
            "AprilTag observation has no source camera"
        )

    try:
        mount = config.camera_mounts[
            str(camera_name)
        ]
    except KeyError as exc:
        raise ValueError(
            f"no camera mount configured for {camera_name!r}"
        ) from exc

    distance_m = (
        float(observation["distance"])
        / 1000.0
    )

    if distance_m <= 0.0:
        raise ValueError(
            "AprilTag distance must be > 0"
        )

    bearing_rad = math.radians(
        float(observation["bearing"])
    )

    vertical_rad = math.radians(
        float(
            observation.get(
                "vertical_angle_deg",
                0.0,
            )
        )
    )

    yaw_rad = math.radians(
        float(observation["yaw_deg"])
    )

    # Slant range -> horizontal camera-plane range.
    horizontal_range_m = (
        distance_m
        * math.cos(vertical_rad)
    )

    centre_x_m = (
        horizontal_range_m
        * math.cos(bearing_rad)
    )

    centre_y_m = (
        -horizontal_range_m
        * math.sin(bearing_rad)
    )

    centre = _feature_from_xy(
        x_m=centre_x_m,
        y_m=centre_y_m,
        mount=mount,
    )

    half_tag_m = (
        tag_size_m
        / 2.0
    )

    # Direction of the tag's horizontal edge in the
    # camera horizontal plane.
    #
    # yaw = 0:
    #     horizontal tag edge lies along camera y.
    edge_x = math.sin(yaw_rad)
    edge_y = math.cos(yaw_rad)

    edge_a = _feature_from_xy(
        x_m=(
                centre_x_m
                - half_tag_m * edge_x
        ),
        y_m=(
                centre_y_m
                - half_tag_m * edge_y
        ),
        mount=mount,
    )

    edge_b = _feature_from_xy(
        x_m=(
                centre_x_m
                + half_tag_m * edge_x
        ),
        y_m=(
                centre_y_m
                + half_tag_m * edge_y
        ),
        mount=mount,
    )

    # +bearing = image right.
    #
    # Therefore the smaller bearing is the image-left
    # feature and the larger bearing is image-right.
    left = min(
        (edge_a, edge_b),
        key=lambda feature: (
            feature.bearing_rad
        ),
    )

    right = max(
        (edge_a, edge_b),
        key=lambda feature: (
            feature.bearing_rad
        ),
    )

    return HorizontalTagVisibility(
        centre=centre,
        left=left,
        right=right,
    )