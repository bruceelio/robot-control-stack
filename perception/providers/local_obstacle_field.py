# perception/providers/local_obstacle_field.py

from __future__ import annotations

import math

from collections.abc import Iterable

from navigation.local_planning.models import (
    LocalObstacleField,
    PolarRangeSample,
)

from perception.robot_geometry import (
    relative_target_from_base_link,
)


def local_obstacle_field_from_objects(
    *,
    config,
    visible_objects: Iterable[dict],
    field_fov_rad: float,
    scan_sectors: int,
    clear_range_mm: float,
    timestamp_s: float,
    exclude_ids: Iterable[int] = (),
    default_obstacle_radius_mm: float = 0.0,
    max_obstacle_distance_mm: float | None = None,
) -> LocalObstacleField:
    """
    Build a robot-frame LocalObstacleField from current object observations.

    This is the current camera-object perception adapter used while the
    Stage-2 obstacle field is still derived from tagged objects.

    The returned field is sensor-independent. Local planning therefore
    does not need to know that the current source is camera object
    perception rather than ToF, depth or LiDAR.

    Coordinate convention:
        +bearing = left / counter-clockwise

    Units:
        range = millimetres
        bearing = radians

    `exclude_ids` allows a selected target to be omitted from the obstacle
    field without introducing target-selection policy into local planning.

    `max_obstacle_distance_mm` is an optional range-of-interest limit.
    """

    field_fov_rad = float(
        field_fov_rad
    )

    scan_sectors = int(
        scan_sectors
    )

    clear_range_mm = float(
        clear_range_mm
    )

    timestamp_s = float(
        timestamp_s
    )

    default_obstacle_radius_mm = max(
        0.0,
        float(
            default_obstacle_radius_mm
        ),
    )

    if field_fov_rad <= 0.0:
        raise ValueError(
            "field_fov_rad must be > 0"
        )

    if (
        scan_sectors < 3
        or scan_sectors % 2 == 0
    ):
        raise ValueError(
            "scan_sectors must be odd and >= 3"
        )

    if (
        not math.isfinite(clear_range_mm)
        or clear_range_mm <= 0.0
    ):
        raise ValueError(
            "clear_range_mm must be finite and > 0"
        )

    excluded_ids = {
        int(object_id)
        for object_id in exclude_ids
    }

    if max_obstacle_distance_mm is not None:
        max_obstacle_distance_mm = float(
            max_obstacle_distance_mm
        )

        if (
            not math.isfinite(
                max_obstacle_distance_mm
            )
            or max_obstacle_distance_mm <= 0.0
        ):
            max_obstacle_distance_mm = None

    sector_width_rad = (
        field_fov_rad
        / float(
            scan_sectors
        )
    )

    half_fov_rad = (
        field_fov_rad
        / 2.0
    )

    bearings = [
        -half_fov_rad
        + sector_width_rad
        * (index + 0.5)
        for index
        in range(
            scan_sectors
        )
    ]

    ranges = [
        clear_range_mm
        for _ in bearings
    ]

    for observation in visible_objects:
        try:
            object_id = int(
                observation["id"]
            )

            target = (
                relative_target_from_base_link(
                    observation=observation,
                    config=config,
                )
            )

            distance_mm = (
                float(
                    target.distance_m
                )
                * 1000.0
            )

            bearing_rad = float(
                target.bearing_rad
            )

        except (
            KeyError,
            TypeError,
            ValueError,
        ):
            continue

        if object_id in excluded_ids:
            continue

        if (
            distance_mm <= 0.0
            or not math.isfinite(
                distance_mm
            )
            or not math.isfinite(
                bearing_rad
            )
        ):
            continue

        if (
            max_obstacle_distance_mm
            is not None
            and distance_mm
            >= max_obstacle_distance_mm
        ):
            continue

        if (
            bearing_rad < -half_fov_rad
            or bearing_rad > half_fov_rad
        ):
            continue

        obstacle_half_width_rad = (
            math.atan2(
                default_obstacle_radius_mm,
                distance_mm,
            )
        )

        for (
            index,
            sample_bearing_rad,
        ) in enumerate(
            bearings
        ):
            if abs(
                sample_bearing_rad
                - bearing_rad
            ) <= (
                obstacle_half_width_rad
                + sector_width_rad / 2.0
            ):
                ranges[index] = min(
                    ranges[index],
                    distance_mm,
                )

    samples = tuple(
        PolarRangeSample(
            bearing_rad=bearing_rad,
            range_mm=range_mm,
            angular_width_rad=(
                sector_width_rad
            ),
            confidence=1.0,
        )
        for (
            bearing_rad,
            range_mm,
        ) in zip(
            bearings,
            ranges,
        )
    )

    return LocalObstacleField(
        samples=samples,
        timestamp_s=timestamp_s,
    )


__all__ = [
    "local_obstacle_field_from_objects",
]