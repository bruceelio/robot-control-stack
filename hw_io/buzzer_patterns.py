# hw_io/buzzer_patterns.py
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterable, List, Tuple, Any, Optional


class BuzzerCue(str, Enum):
    SUCCESS = "success"
    ERROR = "error"
    START = "start"
    END = "end"


@dataclass(frozen=True)
class ToneStep:
    tone: Any                 # int Hz or platform-specific note enum
    duration_s: float
    gap_s: float = 0.05       # silence after the tone


class BuzzerPatterns:
    """
    Semantic buzzer layer.

    - Robot agnostic: no SR imports, no hardware assumptions.
    - Depends only on Level2.BUZZ(...) + Level2.SLEEP(...).
    """

    def __init__(self, lvl2):
        self.l2 = lvl2

    # -----------------------------
    # Low-level building blocks
    # -----------------------------

    def play(self, steps: Iterable[ToneStep], *, blocking: bool = True) -> None:
        for step in steps:
            # Always log: useful in sims with no audio output
            print(f"[BUZZ] tone={step.tone} dur={step.duration_s:.3f}s gap={step.gap_s:.3f}s blocking={blocking}")

            try:
                self.l2.BUZZ(step.tone, step.duration_s, blocking=blocking)
            except Exception as e:
                # If the platform doesn't support buzz, we still keep timing + logs.
                print(f"[BUZZ] (ignored) {e}")

            if step.gap_s > 0:
                self.l2.SLEEP(step.gap_s)

    def beep(self, tone: Any = 440, duration_s: float = 0.2, gap_s: float = 0.05) -> None:
        self.play([ToneStep(tone, duration_s, gap_s)])

    # -----------------------------
    # Canonical cues
    # -----------------------------

    def cue(self, cue: BuzzerCue) -> None:
        self.play(self._cue_steps(cue), blocking=True)

    def _cue_steps(self, cue: BuzzerCue) -> List[ToneStep]:
        # These are intentionally simple + distinctive.
        # You can tune them later per robot (or normalise in adapters).
        if cue == BuzzerCue.START:
            return [
                ToneStep(880, 0.10, 0.05),
                ToneStep(1320, 0.12, 0.00),
            ]
        if cue == BuzzerCue.END:
            return [
                ToneStep(1320, 0.12, 0.05),
                ToneStep(880, 0.10, 0.00),
            ]
        if cue == BuzzerCue.SUCCESS:
            return [
                ToneStep(988, 0.12, 0.05),
                ToneStep(1319, 0.18, 0.00),
            ]
        if cue == BuzzerCue.ERROR:
            return [
                ToneStep(220, 0.25, 0.08),
                ToneStep(220, 0.25, 0.00),
            ]
        # Defensive default
        return [ToneStep(440, 0.15, 0.00)]

    # -----------------------------
    # Fun / diagnostics
    # -----------------------------

    def rickroll_intro(self) -> None:
        """
        Short recognisable intro (your sample style), not the full song.
        Keeps it lightweight + testable.
        """
        seq: List[Tuple[int, float]] = [
            (262, 0.15), (294, 0.15), (349, 0.15), (294, 0.15),
            (440, 0.55), (440, 0.55), (392, 1.10),
        ]
        self.play([ToneStep(freq, dur, 0.05) for freq, dur in seq], blocking=True)
