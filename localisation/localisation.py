# localisation/localisation.py

from __future__ import annotations

import math
from typing import List, Optional, Sequence

from localisation.arbitration import Arbitrator
from localisation.pose_types import Pose
from localisation.providers.base import PoseProvider, PoseObservation


class Localisation:
    """
    Owns the robot's current pose.

    - update_from_vision(...) feeds detections into providers, asks the arbitrator
      for the best observation, and accepts it if one is available
    - estimate(...) remains as a compatibility wrapper
    - apply_motion(...) updates pose by dead-reckoning when you drive/rotate
      (used as a temporary estimate between vision updates)
    """

    def __init__(self, providers: Optional[List[PoseProvider]] = None):
        if providers is None:
            # Import here to avoid circular imports at module import time
            from localisation.providers import default_providers
            providers = default_providers()

        self.providers = providers
        self.arbitrator = Arbitrator(providers)
        self.pose: Optional[Pose] = None

    def has_position(self) -> bool:
        return self.pose is not None and self.pose.position_valid

    def has_heading(self) -> bool:
        return (
            self.pose is not None
            and self.pose.heading_valid
            and self.pose.heading is not None
        )

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

    def set_pose(
        self,
        position,
        heading=None,
        *,
        source: str = "manual",
        timestamp: float = 0.0,
    ) -> None:
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

    def estimate(
        self,
        *,
        now_s: float,
        io=None,
        arena_detections: Sequence[dict] | None = None,
        arena_observations: Sequence[dict] | None = None,
    ) -> PoseObservation | None:
        """
        Compatibility wrapper around provider-fed arbitration.

        Supports both:
        - arena_detections   (preferred)
        - arena_observations (legacy)

        Also supports older callers which do not pass io.
        """
        del io  # currently unused in the provider-fed path

        if arena_detections is None:
            arena_detections = arena_observations

        arena_detections = list(arena_detections or [])

        # Feed fresh detections into providers that support it.
        for provider in self.providers:
            if hasattr(provider, "set_detections"):
                provider.set_detections(arena_detections)

        return self.arbitrator.estimate(now_s=now_s)

    def update_from_vision(
        self,
        *,
        io,
        arena_detections: Sequence[dict],
        now_s: float,
    ) -> bool:
        """
        Ask the arbitrator for the best observation from configured providers.
        Accept it if present.
        """
        obs = self.estimate(
            io=io,
            now_s=now_s,
            arena_detections=arena_detections,
        )
        if obs is None:
            return False

        self.accept(obs)
        return True

    def accept(self, obs: PoseObservation) -> None:
        """
        Controller-facing: accept an observation and update pose.

        Position always comes from the observation.
        Heading is only replaced if the observation provides one.
        Otherwise, preserve the current heading if available.

        Also reseeds all providers after accepting a new pose.
        """
        prev_heading = self.pose.heading if self.pose is not None else None
        prev_heading_valid = self.pose.heading_valid if self.pose is not None else False

        if obs.heading is not None:
            heading = obs.heading
            heading_valid = True
        else:
            heading = prev_heading
            heading_valid = prev_heading_valid and (heading is not None)

        self.pose = Pose(
            x=obs.x if obs.x is not None else 0.0,
            y=obs.y if obs.y is not None else 0.0,
            heading=heading,
            position_valid=obs.position_valid,
            heading_valid=heading_valid,
            source=obs.source,
            timestamp=obs.timestamp,
        )

        for provider in self.providers:
            provider.reseed(self.pose)

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

        for provider in self.providers:
            provider.invalidate()

    def begin_commanded_drive(
        self,
        *,
        distance_mm: float,
        duration_s: float,
        now_s: float,
    ) -> None:
        for provider in self.providers:
            if hasattr(provider, "begin_drive"):
                provider.begin_drive(
                    distance_mm=float(distance_mm),
                    duration_s=float(duration_s),
                    now_s=float(now_s),
                )

    def begin_commanded_rotate(
        self,
        *,
        angle_deg: float,
        duration_s: float,
        now_s: float,
    ) -> None:
        for provider in self.providers:
            if hasattr(provider, "begin_rotate"):
                provider.begin_rotate(
                    angle_deg=float(angle_deg),
                    duration_s=float(duration_s),
                    now_s=float(now_s),
                )

    def apply_motion(self, *, drive_mm: float = 0.0, rotate_deg: float = 0.0) -> None:
        """
        Update pose by applying a commanded motion.

        - Requires a valid position.
        - Heading must be known to update x/y from drive.
        - If heading is unknown, forward motion is not integrated.
        """
        if self.pose is None or not self.pose.position_valid:
            return

        x = self.pose.x
        y = self.pose.y
        heading = self.pose.heading

        if heading is not None:
            heading = self._wrap_rad(heading + math.radians(rotate_deg))

            if abs(drive_mm) > 0.0:
                x += float(drive_mm) * math.cos(heading)
                y += float(drive_mm) * math.sin(heading)

            heading_valid = True
        else:
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