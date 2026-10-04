# navigation/models.py

"""
Shared models for localisation-independent local planning.

Design rules
------------
- Units are explicit in field names.
- Distances and linear speeds use millimetres / millimetres per second.
- Angles and angular speeds use radians / radians per second.
- Bearings are robot-relative: 0 rad = forward, positive = left/CCW.
- These models contain no sensor-specific types and no global localisation state.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite
from typing import Optional, Tuple


class LocalPlanningStatus(str, Enum):
    """High-level outcome from a local-planning update."""

    OK = "ok"
    NO_GAP = "no_gap"
    NO_SAFE_VELOCITY = "no_safe_velocity"
    INVALID_INPUT = "invalid_input"


@dataclass(frozen=True)
class PolarRangeSample:
    """One perception-derived robot-frame range observation.

    ``range_mm=None`` means the sector is unknown / unavailable.
    Unknown must not be interpreted as clear space.

    ``angular_width_rad`` represents angular support / uncertainty rather
    than a particular physical sensor beam.
    """

    bearing_rad: float
    range_mm: Optional[float]
    angular_width_rad: float = 0.0
    confidence: float = 1.0

    @property
    def is_valid(self) -> bool:
        return (
            self.range_mm is not None
            and isfinite(self.range_mm)
            and self.range_mm >= 0.0
            and 0.0 <= self.confidence <= 1.0
        )


@dataclass(frozen=True)
class LocalObstacleField:
    """Ordered local obstacle / free-space observations in the robot frame."""

    samples: Tuple[PolarRangeSample, ...]
    timestamp_s: Optional[float] = None


@dataclass(frozen=True)
class PreferredDirection:
    """Robot-relative direction in which local navigation should progress."""

    bearing_rad: float
    weight: float = 1.0


@dataclass(frozen=True)
class FreeSpaceGap:
    """A traversable angular interval selected from the obstacle field."""

    start_bearing_rad: float
    end_bearing_rad: float
    selected_bearing_rad: float
    min_clearance_mm: Optional[float] = None

    @property
    def angular_width_rad(self) -> float:
        return self.end_bearing_rad - self.start_bearing_rad


@dataclass(frozen=True)
class RobotMotionState:
    """Current robot-relative planar motion."""

    linear_mm_s: float
    angular_rad_s: float


@dataclass(frozen=True)
class RobotDynamicLimits:
    """Robot-profile motion limits consumed by DWA / local planning."""

    linear_min_mm_s: float
    linear_max_mm_s: float
    angular_max_rad_s: float
    linear_accel_mm_s2: float
    linear_decel_mm_s2: float
    angular_accel_rad_s2: float


@dataclass(frozen=True)
class RobotCollisionGeometry:
    """Conservative circular collision geometry for local planning."""

    collision_radius_mm: float
    safety_margin_mm: float = 0.0

    @property
    def inflated_radius_mm(self) -> float:
        return self.collision_radius_mm + self.safety_margin_mm


@dataclass(frozen=True)
class TrackedObstacle:
    """Perception-derived moving obstacle in the robot frame.

    Relative position and velocity are sufficient for VO/TTC-style
    prediction; no arena/global pose is required.
    """

    x_mm: float
    y_mm: float
    vx_mm_s: float
    vy_mm_s: float
    radius_mm: float
    confidence: float = 1.0
    track_id: Optional[str] = None
    timestamp_s: Optional[float] = None


@dataclass(frozen=True)
class LocalPlanningRequest:
    """Common input to the local-planning coordinator."""

    obstacle_field: LocalObstacleField
    preferred_direction: PreferredDirection
    motion: RobotMotionState
    dynamic_limits: RobotDynamicLimits
    collision_geometry: RobotCollisionGeometry
    tracked_obstacles: Tuple[TrackedObstacle, ...] = ()


@dataclass(frozen=True)
class LocalPlanningResult:
    """Local-planning output before the normal velocity-command path."""

    status: LocalPlanningStatus
    linear_mm_s: float
    angular_rad_s: float
    selected_bearing_rad: Optional[float] = None
    selected_gap: Optional[FreeSpaceGap] = None
    reason: str = ""
