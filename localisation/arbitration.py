# localisation/arbitration.py

from __future__ import annotations

from typing import Iterable

from localisation.providers.base import PoseProvider, PoseObservation

MAX_CANDIDATE_AGE_S = 0.20


class Arbitrator:
    """
    Owns provider execution and simple arbitration policy.

    Responsibilities:
    - call configured providers
    - collect candidate observations
    - reject invalid or stale observations
    - prefer absolute pose observations over propagated estimates
    - choose the highest-confidence observation within that class
    """

    def __init__(self, providers: Iterable[PoseProvider]):
        self.providers = list(providers)


    def estimate(
            self,
            *,
            now_s: float,
    ) -> PoseObservation | None:

        absolute_candidates: list[PoseObservation] = []
        propagated_candidates: list[PoseObservation] = []

        for provider in self.providers:
            obs = provider.get_observation(now_s)

            if obs is None:
                continue

            if not self._is_valid_observation(
                    obs,
                    now_s=now_s,
            ):
                continue

            if obs.is_absolute:
                absolute_candidates.append(obs)
            else:
                propagated_candidates.append(obs)

        # --------------------------------------------------
        # Absolute pose always supersedes propagated pose.
        # --------------------------------------------------

        if absolute_candidates:
            candidates = absolute_candidates
        else:
            candidates = propagated_candidates

        if not candidates:
            return None

        return max(
            candidates,
            key=lambda obs: (
                float(obs.confidence),
                -obs.age(now_s),
            ),
        )

    @staticmethod
    def _is_valid_observation(obs: PoseObservation, *, now_s: float) -> bool:
        """
        Basic sanity checks for candidate observations.

        Keep this intentionally minimal for now so the refactor is structural,
        not behavioural. More policy can be added later.
        """
        if not obs.is_usable():
            return False

        if not (0.0 <= obs.confidence <= 1.0):
            return False

        if obs.timestamp > now_s:
            return False

        if obs.age(now_s) > MAX_CANDIDATE_AGE_S:
            return False

        return True