# perception/pickup_range.py

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PickupRangeReading:
    """
    One physical pickup-range sensor reading.

    distance_mm is the raw range measured from the sensor origin
    along that sensor's beam.

    hit_*_base_mm is the measured beam-hit point in base_link:
        +x = forward
        +y = left
        +z = up

    gripper_forward_mm / gripper_lateral_mm are that same hit point
    expressed relative to the configured gripper origin in plan view.

    gripper_distance_mm is the plan-view Euclidean distance from the
    configured gripper origin to the measured hit point.

    A reading object may exist with distance_mm=None. That means the
    sensor exists on this robot but did not produce a usable reading.
    """

    source: str

    distance_mm: float | None

    sensor_x_mm: float
    sensor_y_mm: float
    sensor_z_mm: float
    sensor_yaw_deg: float

    hit_x_base_mm: float | None
    hit_y_base_mm: float | None
    hit_z_base_mm: float | None

    gripper_forward_mm: float | None
    gripper_lateral_mm: float | None
    gripper_distance_mm: float | None

    @property
    def valid(self) -> bool:
        return self.distance_mm is not None


@dataclass(frozen=True)
class PickupRangeObservation:
    """
    Semantic near-field pickup observation.

    Channel meaning:
        left   -> tof.front_left
        centre -> ultrasonic.front
        right  -> tof.front_right

    A channel of None means that semantic sensor capability is not
    resolved on this robot.

    A non-None PickupRangeReading with distance_mm=None means the
    hardware exists but there is no usable return for this sample.
    """

    left: PickupRangeReading | None = None
    centre: PickupRangeReading | None = None
    right: PickupRangeReading | None = None

    @property
    def any_sensor_available(self) -> bool:
        return any(
            reading is not None
            for reading in (
                self.left,
                self.centre,
                self.right,
            )
        )

    @property
    def any_valid_reading(self) -> bool:
        return any(
            reading is not None and reading.valid
            for reading in (
                self.left,
                self.centre,
                self.right,
            )
        )

    @property
    def tof_baseline_mm(self) -> float | None:
        """
        Physical lateral baseline between the two ToF sensor origins.

        None when either ToF is not present on this robot.
        """

        if self.left is None or self.right is None:
            return None

        return abs(
            self.left.sensor_y_mm
            - self.right.sensor_y_mm
        )

    @property
    def available_sources(self) -> tuple[str, ...]:
        return tuple(
            reading.source
            for reading in (
                self.left,
                self.centre,
                self.right,
            )
            if reading is not None
        )

    @property
    def valid_sources(self) -> tuple[str, ...]:
        return tuple(
            reading.source
            for reading in (
                self.left,
                self.centre,
                self.right,
            )
            if reading is not None and reading.valid
        )
