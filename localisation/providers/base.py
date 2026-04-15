# localisation/providers/base.py

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Sequence

from localisation.pose_types import Pose, PoseObservation


class PoseProvider(ABC):
    name: str = "provider"

    @abstractmethod
    def estimate(
        self,
        *,
        io,
        now_s: float,
        current_pose: Pose | None,
        arena_detections: Sequence[dict] | None = None,
    ) -> PoseObservation | None:
        """
        Return a PoseObservation if possible, else None.

        Parameters
        ----------
        io:
            Resolved robot io interface.
        now_s:
            Current timestamp.
        current_pose:
            Last accepted pose, or None if no pose has been seeded yet.
        arena_detections:
            Optional arena detections for vision-based providers.
        """
        raise NotImplementedError