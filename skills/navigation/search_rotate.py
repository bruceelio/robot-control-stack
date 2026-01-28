# skills/navigation/search_rotate.py

from __future__ import annotations

import time
from typing import Iterable, Optional

from primitives.base import Primitive, PrimitiveStatus
from primitives.motion import Rotate
from skills.perception.select_target_utils import get_closest_target


class SearchRotate(Primitive):
    """
    Skill: SearchRotate
    Responsibility:
      - Actively rotate in place to find ANY valid target (early exit when found)
      - Bounded by max rotation and/or timeout

    Inputs:
      - perception
      - kinds: str or list[str] (which target kinds to accept)
      - step_deg, max_deg, timeout_s
      - max_age_s (freshness window for perception memory)
      - settle_s (optional post-rotate settle time)

    Outputs:
      - SUCCEEDED if a target is found (found_target is set)
      - FAILED if scan completes without finding a target
      - RUNNING while scanning
    """

    def __init__(
        self,
        *,
        kinds: str | Iterable[str],
        step_deg: float,
        max_deg: float,
        timeout_s: float,
        max_age_s: float,
        label: str = "SEARCH_ROTATE",
        settle_s: float = 0.0,   # <-- NEW
    ):
        super().__init__()
        if isinstance(kinds, str):
            self.kinds = [kinds]
        else:
            self.kinds = list(kinds)

        self.step_deg = float(step_deg)
        self.max_deg = float(max_deg)
        self.timeout_s = float(timeout_s)
        self.max_age_s = float(max_age_s)
        self.label = label

        self.settle_s = float(settle_s)  # <-- NEW

        self._rotated = 0.0
        self._child: Optional[Rotate] = None
        self._t0: Optional[float] = None
        self.found_target = None

        self._settle_until: Optional[float] = None  # <-- NEW

    def start(self, *, motion_backend, **_):
        self._rotated = 0.0
        self._child = None
        self._t0 = time.time()
        self.found_target = None
        self._settle_until = None  # <-- NEW
        return PrimitiveStatus.RUNNING

    def _find_any_target(self, perception, now: float):
        best = None
        for k in self.kinds:
            t = get_closest_target(perception, k, now=now, max_age_s=self.max_age_s)
            if t is None:
                continue
            if best is None or t["distance"] < best["distance"]:
                best = t
        return best

    def update(self, *, motion_backend, perception=None, **_):
        now = time.time()

        # 0) If settling, wait (but still allow early exit if target appears)
        if self._settle_until is not None:
            if perception is not None:
                t = self._find_any_target(perception, now)
                if t is not None:
                    self.found_target = t
                    self._settle_until = None
                    return PrimitiveStatus.SUCCEEDED

            if now < self._settle_until:
                return PrimitiveStatus.RUNNING

            # settle complete
            self._settle_until = None

        # 1) Early exit if target appears
        if perception is not None:
            t = self._find_any_target(perception, now)
            if t is not None:
                self.found_target = t
                return PrimitiveStatus.SUCCEEDED

        # 2) Timeout bound
        if self._t0 is not None and (now - self._t0) >= self.timeout_s:
            return PrimitiveStatus.FAILED

        # 3) If rotating child active, keep going
        if self._child is not None:
            st = self._child.update(motion_backend=motion_backend)
            if st == PrimitiveStatus.SUCCEEDED:
                self._child = None
                # NEW: camera settle after rotation completes
                if self.settle_s > 0.0:
                    self._settle_until = time.time() + self.settle_s
                return PrimitiveStatus.RUNNING
            if st == PrimitiveStatus.FAILED:
                self._child = None
                return PrimitiveStatus.FAILED
            return PrimitiveStatus.RUNNING

        # 4) Rotation bound
        if self._rotated >= self.max_deg:
            return PrimitiveStatus.FAILED

        # 5) Start next rotate step
        angle = self.step_deg
        self._rotated += abs(angle)

        self._child = Rotate(angle_deg=angle)
        self._child.start(motion_backend=motion_backend)
        return PrimitiveStatus.RUNNING

    def stop(self):
        self._settle_until = None  # <-- NEW
        if self._child is not None:
            self._child.stop()
            self._child = None
