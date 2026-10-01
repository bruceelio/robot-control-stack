# perception/providers/acquisition.py

from __future__ import annotations

import math

from navigation.wall_geometry.models import WallGeometry
from navigation.wall_geometry.resolver import WallGeometryResolver


def _read_side_ultrasonic_mm(
    *,
    config,
    io,
    wall_side: str,
) -> float | None:
    """
    Read the ultrasonic associated with the requested wall side.

    Returns None when:

        - the ultrasonic is not fitted;
        - the current sample is unavailable;
        - the current sample is invalid.
    """

    side = str(wall_side).lower()

    if side not in ("left", "right"):
        raise ValueError(
            f"wall_side must be 'left' or 'right', got {wall_side!r}"
        )

    if not config.has_io(
        "ultrasonic",
        side,
    ):
        return None

    try:
        value = io.ultrasonic[side]
    except Exception:
        return None

    if value is None:
        return None

    try:
        distance_mm = float(value)
    except (TypeError, ValueError):
        return None

    if (
        not math.isfinite(distance_mm)
        or distance_mm <= 0.0
    ):
        return None

    return distance_mm


def acquire_wall_geometry(
    *,
    config,
    io,
    wall_side: str,
    estimated_geometry: WallGeometry | None = None,
) -> WallGeometry:
    """
    Return the strongest usable geometry for one requested wall side.

    Current behaviour:

        side ultrasonic available
            -> use it as the wall-distance channel

        estimated geometry has heading
            -> preserve that heading

        no side ultrasonic but estimated geometry has distance
            -> use the estimated distance

        nothing usable
            -> empty WallGeometry()

    estimated_geometry is the extension point for richer geometry
    sources such as:

        - ultrasonic pair;
        - ultrasonic scan;
        - ToF pair;
        - ToF scan.

    This keeps consumers independent of the physical ranging method.
    """

    def _wall_side_name(wall_side) -> str:
        side = str(
            getattr(
                wall_side,
                "value",
                wall_side,
            )
        ).lower()

        if side not in ("left", "right"):
            raise ValueError(
                f"wall_side must be 'left' or 'right', "
                f"got {wall_side!r}"
            )

        return side

    side = _wall_side_name(wall_side)

    if side not in ("left", "right"):
        raise ValueError(
            f"wall_side must be 'left' or 'right', got {wall_side!r}"
        )

    side_distance_mm = _read_side_ultrasonic_mm(
        config=config,
        io=io,
        wall_side=side,
    )

    heading_rad = None
    estimated_distance_mm = None

    if estimated_geometry is not None:
        if estimated_geometry.has_heading:
            heading_rad = float(
                estimated_geometry.heading_rad
            )

        if estimated_geometry.has_distance:
            estimated_distance_mm = float(
                estimated_geometry.distance_mm
            )

    # Preserve the existing side-ultrasonic distance channel whenever
    # it is available. This also preserves the latched wall-follow
    # distance setpoint if a richer heading source becomes available.
    distance_mm = (
        side_distance_mm
        if side_distance_mm is not None
        else estimated_distance_mm
    )

    return WallGeometryResolver().update(
        heading_rad=heading_rad,
        distance_mm=distance_mm,
    )


__all__ = [
    "acquire_wall_geometry",
]