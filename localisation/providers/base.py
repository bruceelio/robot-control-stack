from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, Dict, Any


@dataclass(frozen=True)
class PoseObservation:
    """
    A provider's proposed pose estimate.

    This is intentionally more flexible than Pose:
    - may contain only position
    - may contain only heading
    - may be low-confidence fallback
    - may represent absolute or relative estimates
    """

    # --- pose values ---
    x: Optional[float] = None
    y: Optional[float] = None
    heading: Optional[float] = None

    # --- validity flags ---
    position_valid: bool = False
    heading_valid: bool = False

    # --- arbitration inputs ---
    confidence: float = 0.5
    source: str = "unknown"
    timestamp: float = 0.0

    # --- classification ---
    # True = absolute reference (e.g. vision, startup)
    # False = relative/integrated (e.g. odometry, motion, OTOS)
    is_absolute: bool = False
    quality: str = "poor"  # "good" | "poor" | "bad"

    # --- diagnostics ---
    diagnostics: Dict[str, Any] = field(default_factory=dict)

    def can_reseed_position(self) -> bool:
        """
        True if this observation can safely reset x/y.
        """
        return (
            self.position_valid
            and self.x is not None
            and self.y is not None
        )

    def can_reseed_heading(self) -> bool:
        """
        True if this observation can safely reset heading.
        """
        return (
            self.heading_valid
            and self.heading is not None
        )

    def has_full_pose(self) -> bool:
        """
        True if both position and heading are valid.
        """
        return self.position_valid and self.heading_valid

    def is_usable(self) -> bool:
        """
        True if this observation contributes anything useful.
        """
        return (
                (self.position_valid or self.heading_valid)
                and self.quality != "bad"
        )

    def age(self, now_s: float) -> float:
        """
        Age of the observation in seconds.
        """
        return max(0.0, now_s - self.timestamp)

    def __repr__(self) -> str:
        """
        Compact debug-friendly representation.
        """
        parts = [
            f"src={self.source}",
            f"q={self.quality}",
            f"conf={self.confidence:.2f}",
        ]

        if self.position_valid:
            parts.append(f"x={self.x:.1f}")
            parts.append(f"y={self.y:.1f}")

        if self.heading_valid:
            parts.append(f"h={self.heading:.2f}")

        if self.is_absolute:
            parts.append("ABS")

        return f"<PoseObs {' '.join(parts)}>"


# Lightweight alias without importing pose_types here
PoseLike = Any


class PoseProvider(ABC):
    """
    Base interface for localisation providers.

    Providers may be absolute (vision, startup, OTOS with map alignment)
    or relative (deadwheel odometry, commanded motion, IMU-integrated heading).
    """

    def __init__(self, name: str, *, base_weight: float = 1.0):
        self.name = name
        self.base_weight = float(base_weight)

    @abstractmethod
    def get_observation(self, now_s: float) -> Optional[PoseObservation]:
        """
        Return the provider's latest pose observation, or None if no update is available.
        """
        raise NotImplementedError

    def reseed(self, pose: PoseLike) -> None:
        """
        Reset/re-anchor the provider from an externally accepted pose.

        Absolute providers will often ignore this.
        Relative/integrating providers should usually reset internal state here.
        """
        return None

    def invalidate(self) -> None:
        """
        Optional hook to mark internal state stale/bad.
        """
        return None