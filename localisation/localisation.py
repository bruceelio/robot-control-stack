# localisation/localisation.py

from __future__ import annotations

from dataclasses import replace

import math
from typing import List, Optional, Sequence


from localisation.pose_types import Pose
from localisation.providers.base import PoseProvider, PoseObservation
from vision.apriltag.observations import AprilTagObservation

from localisation.fusion import (
    PoseEstimator,
    create_default_estimator,
)

class Localisation:
    """
    Owns the robot's current pose.

    - update_from_vision(...) feeds detections into providers, asks the arbitrator
      for the best observation, and accepts it if one is available
    - estimate(...) remains as a compatibility wrapper
    - apply_motion(...) updates pose by dead-reckoning when you drive/rotate
      (used as a temporary estimate between vision updates)
    """

    def __init__(
            self,
            providers: Optional[List[PoseProvider]] = None,
            *,
            estimator: PoseEstimator | None = None,
    ):
        if providers is None:
            # Import here to avoid circular imports at module import time
            from localisation.providers import default_providers
            providers = default_providers()

        self.providers = providers

        print("[LOC][PROVIDERS]")

        for provider in self.providers:
            print(
                f"  {type(provider).__name__}"
                f" -> {provider.name}"
            )

            for child in getattr(provider, "providers", []):
                print(
                    f"    {type(child).__name__}"
                    f" -> {child.name}"
                )

        if estimator is None:
            estimator = create_default_estimator(providers)

        self.estimator: PoseEstimator = estimator

        # Compatibility alias for existing callers.
        self.arbitrator = self.estimator

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
            covariance=None,
    ) -> None:
        x, y = position
        self.pose = Pose(
            x=float(x),
            y=float(y),
            heading=heading,
            position_valid=True,
            heading_valid=(heading is not None),
            source=source,
            timestamp=float(timestamp),
            covariance=covariance,
        )

        for provider in self.providers:
            provider.reseed(self.pose)

    def estimate(
            self,
            *,
            now_s: float,
            io=None,
            arena_detections: Sequence[dict] | None = None,
            arena_observations: Sequence[dict] | None = None,
            vision_message: dict | None = None,
            apriltag_source_id: str | None = None,
            apriltag_observations: Sequence[AprilTagObservation] | None = None,
    ) -> PoseObservation | None:
        """
        Compatibility wrapper around provider-fed arbitration.

        Supports both:
        - arena_detections   (preferred)
        - arena_observations (legacy)

        Also supports older callers which do not pass io.
        """
        # Supply semantic IO to providers which need it.
        for provider in self.providers:
            if hasattr(provider, "set_io"):
                provider.set_io(io)

        # --------------------------------------------------
        # Canonical Vision path
        # --------------------------------------------------

        for provider in self.providers:
            if hasattr(provider, "set_vision_message"):
                provider.set_vision_message(
                    vision_message
                )

        # --------------------------------------------------
        # Legacy direct-input compatibility
        # --------------------------------------------------

        if vision_message is None:

            if arena_detections is None:
                arena_detections = arena_observations

            arena_detections = list(
                arena_detections or []
            )

            apriltag_observations = list(
                apriltag_observations or []
            )

            for provider in self.providers:

                if hasattr(provider, "set_detections"):
                    provider.set_detections(
                        arena_detections
                    )

                if hasattr(
                        provider,
                        "set_apriltag_observations",
                ):
                    provider.set_apriltag_observations(
                        source_id=apriltag_source_id,
                        observations=apriltag_observations,
                    )

        return self.estimator.estimate(now_s=now_s)

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

        Absolute observations reseed providers so propagated pose estimates
        continue from the corrected global pose.
        """
        prev_heading = self.pose.heading if self.pose is not None else None
        prev_heading_valid = self.pose.heading_valid if self.pose is not None else False

        if obs.heading is not None:
            heading = obs.heading
            heading_valid = True
        else:
            heading = prev_heading
            heading_valid = prev_heading_valid and (heading is not None)

        # The observation covariance can describe the stored pose directly
        # only when the observation supplies the complete pose used here.
        #
        # If heading is being preserved from the previous pose, combining
        # the old heading uncertainty with the new position uncertainty
        # requires a proper fusion/update step. Do not invent that here.
        if obs.heading_valid and obs.heading is not None:
            covariance = obs.covariance
        else:
            covariance = None

        self.pose = Pose(
            x=obs.x if obs.x is not None else 0.0,
            y=obs.y if obs.y is not None else 0.0,
            heading=heading,
            position_valid=obs.position_valid,
            heading_valid=heading_valid,
            source=obs.source,
            timestamp=obs.timestamp,
            covariance=covariance,
        )

        # Absolute observations correct/reseed propagated pose sources.
        # Propagated observations must be allowed to continue accumulating dead_reckoning.
        if obs.is_absolute:
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

        self.pose = replace(
            self.pose,
            position_valid=False,
            heading_valid=False,
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
        Update pose by applying a commanded dead_reckoning.

        - Requires a valid position.
        - Heading must be known to update x/y from drive.
        - If heading is unknown, forward dead_reckoning is not integrated.
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

        self.pose = replace(
            self.pose,
            x=x,
            y=y,
            heading=heading,
            position_valid=True,
            heading_valid=heading_valid,

            # Proper propagation would require a motion-model Jacobian
            # and process noise. Until that exists, do not carry forward
            # covariance which no longer describes the updated pose.
            covariance=None,
        )

    @staticmethod
    def _wrap_rad(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi