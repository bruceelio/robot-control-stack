# skills/perception/select_target.py

"""
SelectTarget skill (control-loop program).

Stateful wrapper around target selection utilities:
- RUNNING until a target is available
- SUCCEEDED once selected_target is set
- Optional seed_target support
- Throttled "no target" logging

This is the "main program" for target selection in your architecture.
"""

from __future__ import annotations

import time

from primitives.base import PrimitiveStatus
from skills.perception.select_target_utils import get_closest_target


class SelectTarget:
    def __init__(
        self,
        *,
        kind: str,
        max_age_s: float,
        log_every_s: float = 1.0,
        label: str = "SELECT_TARGET",
    ):
        self.kind = kind
        self.max_age_s = float(max_age_s)
        self.log_every_s = float(log_every_s)
        self.label = label

        self.selected_target = None
        self._last_no_target_log = None
        self._seed_used = False

    def start(self, *, seed_target=None, **_):
        self.selected_target = None
        self._last_no_target_log = None

        # If we get a seed target, we can succeed immediately on first update.
        self._seed_used = False
        self._seed_target = seed_target

        return PrimitiveStatus.RUNNING

    def update(self, *, perception=None, now=None, **_):
        if now is None:
            now = time.time()

        # Seed target handoff (optional)
        if self._seed_target is not None and not self._seed_used:
            self._seed_used = True
            self.selected_target = self._seed_target
            tid = self.selected_target.get("id", "REL")
            dist = float(self.selected_target.get("distance", 0.0))
            bearing = float(self.selected_target.get("bearing", 0.0))
            print(f"[{self.label}] seeded id={tid} dist={dist:.0f} bearing={bearing:.1f}")
            return PrimitiveStatus.SUCCEEDED

        t = get_closest_target(
            perception,
            self.kind,
            now=now,
            max_age_s=self.max_age_s,
        )

        if t is None:
            if self._last_no_target_log is None or (now - self._last_no_target_log) >= self.log_every_s:
                print(f"[{self.label}] no target visible — waiting")
                self._last_no_target_log = now
            return PrimitiveStatus.RUNNING

        self.selected_target = t
        tid = t.get("id", "REL")
        print(f"[{self.label}] chose id={tid} dist={t['distance']:.0f} bearing={t['bearing']:.1f}")
        return PrimitiveStatus.SUCCEEDED
