# localisation/localisation.py

from __future__ import annotations

import math
from typing import List, Optional, Sequence

from localisation.pose_types import Pose, PoseObservation
from localisation.providers.base import PoseProvider


class Localisation:
    """
    Owns the robot's current pose.

    - update_from_vision(...) queries providers and selects best observation
    - apply_motion(...) updates pose by dead-reckoning when you drive/rotate
      (used as a temporary estimate between vision updates)
    """

    def __init__(self, providers: Optional[List[PoseProvider]] = None):
        if providers is None:
            # Import here to avoid circular imports at module import time
            from localisation.providers import default_providers
            providers = default_providers()

        self.providers = list(providers)
        self.pose: Optional[Pose] = None

    def has_position(self) -> bool:
        return self.pose is not None and self.pose.position_valid

    def has_heading(self) -> bool:
        return self.pose is not None and self.pose.heading_valid and self.pose.heading is not None

    # -------------------------
    # Compatibility shims (legacy behaviours)
    # -------------------------

    def has_pose(self) -> bool:
        """
        Legacy name used by some behaviours.
        Equivalent to 'has a valid position'.
        """
        return self.has_position()

    def get_pose(self):
        """
        Legacy getter used by some behaviours.
        Returns (position_tuple, heading) or (None, None) if invalid.
        """
        if self.pose is None or not self.pose.position_valid:
            return None, None
        return (self.pose.x, self.pose.y), self.pose.heading

    def set_pose(self, position, heading=None, *, source: str = "manual", timestamp: float = 0.0) -> None:
        """
        Legacy setter used by older code paths.
        """
        x, y = position
        self.pose = Pose(
            x=float(x),
            y=float(y),
            heading=heading,
            position_valid=True,
            heading_valid=(heading is not None),
            source=source,
            timestamp=float(timestamp),
        )

    def update_from_vision(self, *, io, arena_detections: Sequence[dict], now_s: float) -> bool:
        obs = self.estimate(io=io, arena_observations=arena_detections, now_s=now_s)
        if obs is None:
            return False
        self.accept(obs)
        return True

    def estimate(
            self,
            *,
            io,
            arena_observations: Sequence[dict] | None = None,
            now_s: float,
    ) -> PoseObservation | None:
        """
        Controller-facing: pick the best PoseObservation from providers.
        """
        best: Optional[PoseObservation] = None
        for p in self.providers:
            obs = p.estimate(
                io=io,
                now_s=now_s,
                current_pose=self.pose,
                arena_detections=arena_observations,
            )
            if obs is None:
                continue
            if best is None or obs.confidence > best.confidence:
                best = obs
        return best

    def accept(self, obs: PoseObservation) -> None:
        """
        Controller-facing: accept an observation and set pose.
        """
        self.pose = Pose(
            x=obs.x,
            y=obs.y,
            heading=obs.heading,
            position_valid=True,
            heading_valid=(obs.heading is not None),
            source=obs.source,
            timestamp=obs.timestamp,
        )

    def invalidate(self) -> None:
        """
        Controller-facing: mark pose invalid.
        """
        if self.pose is None:
            self.pose = Pose(
                x=0.0,
                y=0.0,
                heading=None,
                position_valid=False,
                heading_valid=False,
                source="none",
                timestamp=0.0,
            )
            return

        self.pose = Pose(
            x=self.pose.x,
            y=self.pose.y,
            heading=self.pose.heading,
            position_valid=False,
            heading_valid=False,
            source=self.pose.source,
            timestamp=self.pose.timestamp,
        )

    def apply_motion(self, *, drive_mm: float = 0.0, rotate_deg: float = 0.0) -> None:
        """
        Update pose by applying a commanded motion.

        - Requires a pose position.
        - Heading must be known to update x/y from drive.
          If heading is unknown, we only update heading if rotate is provided AND heading exists.
        """
        if self.pose is None or not self.pose.position_valid:
            return

        x = self.pose.x
        y = self.pose.y
        heading = self.pose.heading

        # Update heading if we have one
        if heading is not None:
            heading = self._wrap_rad(heading + math.radians(rotate_deg))

            # Update position if we have heading and a drive command
            if abs(drive_mm) > 0.0:
                x += float(drive_mm) * math.cos(heading)
                y += float(drive_mm) * math.sin(heading)

            heading_valid = True
        else:
            # Heading unknown: cannot reliably integrate forward motion
            heading_valid = False

        self.pose = Pose(
            x=x,
            y=y,
            heading=heading,
            position_valid=True,
            heading_valid=heading_valid,
            source=self.pose.source,
            timestamp=self.pose.timestamp,
        )

    @staticmethod
    def _wrap_rad(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi
