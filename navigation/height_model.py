# navigation/height_model.py

from __future__ import annotations
from dataclasses import dataclass


@dataclass
class HeightDecision:
    committed: bool
    is_high: bool | None
    reason: str = ""


class HeightModel:
    """
    Belief model for marker height (high vs low).

    IMPORTANT: pitch values are radians.
    """

    def __init__(
        self,
        *,
        promote_score: float = 2.0,
        demote_score: float = 5.0,
        step: float = 1.0,
    ):
        self.promote_score = float(promote_score)
        self.demote_score = float(demote_score)
        self.step = float(step)
        self.reset()

    def reset(self):
        self._committed = False
        self._is_high: bool | None = None
        self.score = 0.0
        self.max_pitch = float("-inf")
        self.samples = 0

    def is_committed(self) -> bool:
        return self._committed

    def is_high(self) -> bool:
        return bool(self._is_high)

    @staticmethod
    def _weights(distance_mm: float) -> tuple[float, float]:
        d = float(distance_mm)
        if d >= 2500:
            return (0.20, 0.01)
        if d >= 2000:
            return (0.35, 0.02)
        if d >= 1600:
            return (0.55, 0.05)
        if d >= 1200:
            return (0.80, 0.20)
        return (1.00, 0.50)

    def update(
        self,
        *,
        pitch_deg: float,      # radians
        distance_mm: float,
        high_thresh: float,
        low_thresh: float,
    ) -> None:
        if self._committed:
            return

        pitch = float(pitch_deg)
        d = float(distance_mm)

        self.samples += 1
        self.max_pitch = max(self.max_pitch, pitch)

        margin = 0.02
        ev = 0.0
        if pitch >= (high_thresh + margin):
            ev = +1.0
        elif pitch <= (low_thresh - margin):
            ev = -1.0

        if ev == 0.0:
            return

        w_pos, w_neg = self._weights(d)
        w = w_pos if ev > 0 else w_neg
        self.score += ev * self.step * w

    def try_commit(
        self,
        *,
        distance_mm: float,
        high_thresh: float,
        low_thresh: float,
        decision_deadline_mm: float,
        low_confirm_max_mm: float = 1600.0,
    ) -> HeightDecision:

        if self._committed:
            return HeightDecision(True, self._is_high, "already_committed")

        d = float(distance_mm)

        if self.max_pitch >= (high_thresh + 0.02):
            self._commit(True)
            return HeightDecision(True, True, "peak_high_latch")

        if self.score >= self.promote_score:
            self._commit(True)
            return HeightDecision(True, True, "score_promote")

        if d <= low_confirm_max_mm and self.score <= -self.demote_score:
            self._commit(False)
            return HeightDecision(True, False, "score_demote_close")

        if d <= decision_deadline_mm:
            # If we've ever seen a strong "high" pitch, latch high.
            if self.max_pitch >= (high_thresh + 0.02):
                self._commit(True)
                return HeightDecision(True, True, "deadline_peak_high")

            # Otherwise decide from score (negative -> LOW, non-negative -> HIGH).
            is_high = (self.score >= 0.0)
            self._commit(is_high)
            return HeightDecision(True, is_high, "deadline_score_sign")

        return HeightDecision(False, None, "no_commit")

    def _commit(self, is_high: bool):
        self._committed = True
        self._is_high = bool(is_high)
