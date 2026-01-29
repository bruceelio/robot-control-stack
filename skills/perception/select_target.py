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
from typing import Iterable, Optional

from primitives.base import PrimitiveStatus
from skills.perception.select_target_utils import get_closest_target



def _safe_int(v) -> Optional[int]:
    if v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


class SelectTarget:
    def __init__(
        self,
        *,
        kind: str,
        max_age_s: float,
        log_every_s: float = 1.0,
        label: str = "SELECT_TARGET",
        exclude_ids: Iterable[int] | None = None,
    ):
        self.kind = kind
        self.max_age_s = float(max_age_s)
        self.log_every_s = float(log_every_s)
        self.label = label

        # NOTE: callers can pass a shared/mutable set here.
        self.exclude_ids = exclude_ids

        self.selected_target = None
        self._last_no_target_log = None
        self._seed_used = False
        self._seed_target = None
        self._seed_rejected_logged = False

    def start(self, *, seed_target=None, exclude_ids=None, **_):
        self._exclude_ids = set(exclude_ids) if exclude_ids else set()

        self.selected_target = None
        self._last_no_target_log = None

        # If we get a seed target, we can succeed immediately on first update
        # (unless it is excluded).
        self._seed_used = False
        self._seed_target = seed_target
        self._seed_rejected_logged = False

        return PrimitiveStatus.RUNNING

    def update(self, *, perception=None, now=None, **_):

        if now is None:
            now = time.time()

        # Make an exclusion set snapshot (but keep reference-friendly behaviour)
        exclude = set(int(x) for x in self.exclude_ids) if self.exclude_ids is not None else self._exclude_ids

        t = get_closest_target(
            perception,
            self.kind,
            now=now,
            max_age_s=self.max_age_s,
            exclude_ids=exclude,
        )

        # Seed target handoff (optional)
        if self._seed_target is not None and not self._seed_used:
            tid = _safe_int(self._seed_target.get("id"))

            if exclude is not None and tid is not None and tid in exclude:
                # Ignore seed if it's already delivered/blocked
                if not self._seed_rejected_logged:
                    print(f"[{self.label}] seed id={tid} is excluded — ignoring seed")
                    self._seed_rejected_logged = True
                self._seed_used = True
                self._seed_target = None
            else:
                self._seed_used = True
                self.selected_target = self._seed_target
                tid_disp = self.selected_target.get("id", "REL")
                dist = float(self.selected_target.get("distance", 0.0))
                bearing = float(self.selected_target.get("bearing", 0.0))
                print(f"[{self.label}] seeded id={tid_disp} dist={dist:.0f} bearing={bearing:.1f}")
                return PrimitiveStatus.SUCCEEDED

        t = get_closest_target(
            perception,
            self.kind,
            now=now,
            max_age_s=self.max_age_s,
            exclude_ids=self._exclude_ids,
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
