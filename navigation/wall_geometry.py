from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class WallGeometry:
    """
    Robot-relative geometry of a wall.

    Coordinate convention:

        +x       forward
        +y       left
        +heading counter-clockwise / left

    heading_rad:
        Direction of the wall normal from base_link.

         0 deg = robot faces square toward wall
        +90 deg = wall is on robot's left
        -90 deg = wall is on robot's right

    distance_mm:
        Perpendicular distance from base_link to the wall.

    Either value may be unavailable independently.
    """

    heading_rad: float | None = None
    distance_mm: float | None = None

    @property
    def has_heading(self) -> bool:
        return (
            self.heading_rad is not None
            and math.isfinite(self.heading_rad)
        )

    @property
    def has_distance(self) -> bool:
        return (
            self.distance_mm is not None
            and math.isfinite(self.distance_mm)
            and self.distance_mm >= 0.0
        )


@dataclass(frozen=True)
class RangeObservation2D:
    """
    One range measurement expressed in base_link geometry.

    No assumption is made about the physical sensor which
    produced the measurement.
    """

    distance_mm: float

    sensor_x_mm: float = 0.0
    sensor_y_mm: float = 0.0
    sensor_yaw_rad: float = 0.0