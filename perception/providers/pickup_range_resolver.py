# perception/providers/pickup_range_resolver.py

from __future__ import annotations

import math

from perception.pickup_range import (
    PickupRangeObservation,
    PickupRangeReading,
)


_CHANNELS = {
    "left": ("tof", "front_left"),
    "centre": ("ultrasonic", "front"),
    "right": ("tof", "front_right"),
}


def _usable_distance_mm(value) -> float | None:
    if value is None:
        return None

    try:
        distance_mm = float(value)
    except (TypeError, ValueError):
        return None

    if not math.isfinite(distance_mm):
        return None

    if distance_mm <= 0.0:
        return None

    return distance_mm


def _reading_from_sensor(
    *,
    config,
    io,
    category: str,
    name: str,
) -> PickupRangeReading | None:
    """
    Resolve and read one semantic pickup-range sensor.

    None means the sensor capability is not present on this robot.

    A PickupRangeReading with distance_mm=None means the sensor is
    present but this sample did not produce a usable range.
    """

    if not config.has_io(category, name):
        return None

    source = f"{category}.{name}"

    try:
        mount = config.range_sensor_mounts[source]
    except KeyError as exc:
        raise ValueError(
            f"Resolved pickup sensor {source!r} has no "
            "RANGE_SENSOR_MOUNTS geometry"
        ) from exc

    sensor_x_mm = float(mount.get("x_mm", 0.0))
    sensor_y_mm = float(mount.get("y_mm", 0.0))
    sensor_z_mm = float(mount.get("z_mm", 0.0))
    sensor_yaw_deg = float(mount.get("yaw_deg", 0.0))

    collection = getattr(io, category)

    try:
        raw_value = collection[name]
    except Exception:
        raw_value = None

    distance_mm = _usable_distance_mm(raw_value)

    if distance_mm is None:
        return PickupRangeReading(
            source=source,
            distance_mm=None,
            sensor_x_mm=sensor_x_mm,
            sensor_y_mm=sensor_y_mm,
            sensor_z_mm=sensor_z_mm,
            sensor_yaw_deg=sensor_yaw_deg,
            hit_x_base_mm=None,
            hit_y_base_mm=None,
            hit_z_base_mm=None,
            gripper_forward_mm=None,
            gripper_lateral_mm=None,
            gripper_distance_mm=None,
        )

    yaw_rad = math.radians(sensor_yaw_deg)

    hit_x_base_mm = (
        sensor_x_mm
        + distance_mm * math.cos(yaw_rad)
    )

    hit_y_base_mm = (
        sensor_y_mm
        + distance_mm * math.sin(yaw_rad)
    )

    # Pickup sensors are currently modelled as horizontal beams.
    # z is retained as geometry metadata so sensor height is not lost.
    hit_z_base_mm = sensor_z_mm

    gripper = config.gripper_mount

    gripper_x_mm = (
            float(gripper["x_m"])
            * 1000.0
    )

    gripper_y_mm = (
            float(gripper["y_m"])
            * 1000.0
    )

    gripper_yaw_rad = float(
        gripper.get(
            "yaw_rad",
            0.0,
        )
    )

    delta_x_mm = hit_x_base_mm - gripper_x_mm
    delta_y_mm = hit_y_base_mm - gripper_y_mm

    cos_gripper = math.cos(gripper_yaw_rad)
    sin_gripper = math.sin(gripper_yaw_rad)

    # base_link -> gripper plan-view coordinates.
    gripper_forward_mm = (
        cos_gripper * delta_x_mm
        + sin_gripper * delta_y_mm
    )

    gripper_lateral_mm = (
        -sin_gripper * delta_x_mm
        + cos_gripper * delta_y_mm
    )

    gripper_distance_mm = math.hypot(
        gripper_forward_mm,
        gripper_lateral_mm,
    )

    return PickupRangeReading(
        source=source,
        distance_mm=distance_mm,
        sensor_x_mm=sensor_x_mm,
        sensor_y_mm=sensor_y_mm,
        sensor_z_mm=sensor_z_mm,
        sensor_yaw_deg=sensor_yaw_deg,
        hit_x_base_mm=hit_x_base_mm,
        hit_y_base_mm=hit_y_base_mm,
        hit_z_base_mm=hit_z_base_mm,
        gripper_forward_mm=gripper_forward_mm,
        gripper_lateral_mm=gripper_lateral_mm,
        gripper_distance_mm=gripper_distance_mm,
    )


def resolve_pickup_range(
    *,
    config,
    io,
) -> PickupRangeObservation:
    """
    Read all pickup-range capabilities which are actually resolved
    for this robot and return one semantic observation.

    Sensor selection is automatic through config.has_io(...).
    """

    readings = {}

    for channel, (category, name) in _CHANNELS.items():
        readings[channel] = _reading_from_sensor(
            config=config,
            io=io,
            category=category,
            name=name,
        )

    return PickupRangeObservation(
        left=readings["left"],
        centre=readings["centre"],
        right=readings["right"],
    )


__all__ = [
    "resolve_pickup_range",
]
