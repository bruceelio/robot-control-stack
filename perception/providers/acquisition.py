# perception/providers/acquisition.py

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from navigation.wall_geometry.models import WallGeometry
from navigation.wall_geometry.resolver import WallGeometryResolver


class WallGeometryState(Enum):
    """
    Availability of wall geometry for the current request.
    """

    AVAILABLE = "available"
    ACQUIRABLE = "acquirable"
    UNAVAILABLE = "unavailable"


class WallAcquisitionAction(Enum):
    """
    Additional perception acquisition required before usable wall
    geometry can be produced.
    """

    NONE = "none"
    SAMPLE_EXISTING = "sample_existing"
    SCAN_SINGLE_SENSOR = "scan_single_sensor"
    GATHER_MORE = "gather_more"


@dataclass(frozen=True)
class WallGeometryStatus:
    """
    Perception status for a wall-geometry request.

    AVAILABLE
        The supplied geometry already satisfies the requested
        geometric requirements.

    ACQUIRABLE
        The required geometry is not currently available, but the
        configured ranging capability can obtain more evidence.

    UNAVAILABLE
        The required geometry is not available and no geometrically
        configured ranging source can currently provide it.

    geometry may contain partial geometry even when the state is
    ACQUIRABLE or UNAVAILABLE.
    """

    state: WallGeometryState
    geometry: WallGeometry | None = None

    action: WallAcquisitionAction = (
        WallAcquisitionAction.NONE
    )

    reason: str | None = None

    @property
    def available(self) -> bool:
        return self.state == WallGeometryState.AVAILABLE

    @property
    def acquisition_required(self) -> bool:
        return self.state == WallGeometryState.ACQUIRABLE

    @property
    def unavailable(self) -> bool:
        return self.state == WallGeometryState.UNAVAILABLE

def _geometry_satisfies_request(
    geometry: WallGeometry | None,
    *,
    require_heading: bool,
    require_distance: bool,
) -> bool:
    """
    Return whether the supplied WallGeometry contains every component
    requested by the consumer.
    """

    if geometry is None:
        return False

    if require_heading and not geometry.has_heading:
        return False

    if require_distance and not geometry.has_distance:
        return False

    return True


def _configured_geometric_range_sensors(
    *,
    config,
) -> tuple[str, ...]:
    """
    Return configured ranging IO sources whose mounting geometry is
    known well enough to construct directed range observations.

    Detection is intentionally based on the generic configuration
    relationship:

        config.io
        +
        config.range_sensor_mounts

    rather than on specific technologies such as ultrasonic or ToF.

    A future ranging source therefore only needs:

        - an enabled IO key;
        - a matching mount entry;
        - x/y origin geometry;
        - ray heading geometry.

    Both current source-config units (mm / deg) and possible future
    normalized runtime units (m / rad) are accepted here.
    """

    io_map = getattr(
        config,
        "io",
        {},
    ) or {}

    mounts = getattr(
        config,
        "range_sensor_mounts",
        {},
    ) or {}

    usable: list[str] = []

    for sensor_key, mount in mounts.items():

        sensor_key = str(sensor_key)

        if io_map.get(sensor_key) is None:
            continue

        if not isinstance(mount, dict):
            continue

        has_position = (
            (
                "x_mm" in mount
                and "y_mm" in mount
            )
            or (
                "x_m" in mount
                and "y_m" in mount
            )
        )

        has_heading = (
            "yaw_deg" in mount
            or "yaw_rad" in mount
        )

        if not (
            has_position
            and has_heading
        ):
            continue

        usable.append(sensor_key)

    return tuple(usable)


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

def get_wall_geometry_status(
    *,
    config,
    estimated_geometry: WallGeometry | None = None,
    require_heading: bool = True,
    require_distance: bool = True,
) -> WallGeometryStatus:
    """
    Report whether the requested wall geometry is already available,
    can be acquired, or is unavailable.

    This function performs capability/status reasoning only.

    It does not:

        - read the sensors;
        - move the robot;
        - manufacture RangeRay2D observations;
        - decide whether multiple observations belong to one wall;
        - execute a scan.

    The returned acquisition action tells the caller what additional
    perception work is required.
    """

    if not (
        require_heading
        or require_distance
    ):
        raise ValueError(
            "wall geometry request must require "
            "heading, distance, or both"
        )

    if _geometry_satisfies_request(
        estimated_geometry,
        require_heading=require_heading,
        require_distance=require_distance,
    ):
        return WallGeometryStatus(
            state=WallGeometryState.AVAILABLE,
            geometry=estimated_geometry,
            action=WallAcquisitionAction.NONE,
            reason="geometry_available",
        )

    sensors = (
        _configured_geometric_range_sensors(
            config=config,
        )
    )

    if len(sensors) >= 2:
        return WallGeometryStatus(
            state=WallGeometryState.ACQUIRABLE,
            geometry=estimated_geometry,
            action=(
                WallAcquisitionAction
                .SAMPLE_EXISTING
            ),
            reason=(
                f"{len(sensors)}_configured_"
                "range_sensors"
            ),
        )

    if len(sensors) == 1:
        return WallGeometryStatus(
            state=WallGeometryState.ACQUIRABLE,
            geometry=estimated_geometry,
            action=(
                WallAcquisitionAction
                .SCAN_SINGLE_SENSOR
            ),
            reason=(
                "single_configured_range_sensor:"
                f"{sensors[0]}"
            ),
        )

    return WallGeometryStatus(
        state=WallGeometryState.UNAVAILABLE,
        geometry=estimated_geometry,
        action=WallAcquisitionAction.NONE,
        reason=(
            "no_geometrically_configured_"
            "range_sensor"
        ),
    )


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
    "WallAcquisitionAction",
    "WallGeometryState",
    "WallGeometryStatus",
    "acquire_wall_geometry",
    "get_wall_geometry_status",
]