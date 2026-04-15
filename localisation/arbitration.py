# localisation/arbitration.py

from __future__ import annotations

from typing import Iterable, Optional, Sequence

from localisation.pose_types import Pose, PoseObservation
from localisation.providers.base import PoseProvider


class Arbitrator:
    """
    Owns provider execution and simple arbitration policy.

    Responsibilities:
    - call configured providers
    - collect candidate observations
    - reject invalid observations
    - choose the best remaining observation

    Initial policy is intentionally simple:
    highest confidence wins.
    """

    def __init__(self, providers: Iterable[PoseProvider]):
        self.providers = list(providers)

    def estimate(
        self,
        *,
        io,
        now_s: float,
        current_pose: Pose | None,
        arena_detections: Sequence[dict] | None = None,
    ) -> PoseObservation | None:
        """
        Return the best PoseObservation from configured providers,
        or None if no usable observation is available.
        """
        best: Optional[PoseObservation] = None

        for provider in self.providers:
            obs = provider.estimate(
                io=io,
                now_s=now_s,
                current_pose=current_pose,
                arena_detections=arena_detections,
            )
            if obs is None:
                continue

            if not self._is_valid_observation(obs, now_s=now_s):
                continue

            if best is None or obs.confidence > best.confidence:
                best = obs

        return best

    @staticmethod
    def _is_valid_observation(obs: PoseObservation, *, now_s: float) -> bool:
        """
        Basic sanity checks for candidate observations.

        Keep this intentionally minimal for now so the refactor is structural,
        not behavioural. More policy can be added later.
        """
        if not (0.0 <= obs.confidence <= 1.0):
            return False

        if obs.timestamp > now_s:
            return False

        return True