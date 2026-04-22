# localisation/arbitration.py

from __future__ import annotations

from typing import Iterable, Optional

from localisation.providers.base import PoseProvider, PoseObservation


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

    def _score(self, provider: PoseProvider, obs: PoseObservation, now_s: float) -> float:
        score = provider.base_weight * obs.confidence

        # Prefer absolute vision over timed/dead-reckoned motion.
        # A 2-tag visual reseed is usually more globally correct than motion drift.
        if obs.source == "cam1_markers2":
            candidate_count = 0
            if obs.diagnostics:
                candidate_count = int(obs.diagnostics.get("candidate_count", 0))

            # Strong bias for any usable vision pose
            score += 1.0

            # Extra reward for stronger visual geometry
            if candidate_count >= 2:
                score += 0.2

        return score

    def estimate(self, *, now_s: float) -> PoseObservation | None:
        """
        Return the best PoseObservation from configured providers,
        or None if no usable observation is available.
        """
        best: Optional[PoseObservation] = None
        best_score: float = float("-inf")

        for provider in self.providers:
            obs = provider.get_observation(now_s)
            if obs is None:
                continue

            if not self._is_valid_observation(obs, now_s=now_s):
                continue

            score = self._score(provider, obs, now_s)

            if best is None or score > best_score:
                best = obs
                best_score = score

        return best

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

        return True