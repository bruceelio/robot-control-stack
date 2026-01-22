# skills/perception/reacquire_target.py

import time

from primitives.base import Primitive, PrimitiveStatus
from primitives.motion import Rotate
from skills.perception.select_target import get_closest_target


class ReacquireTarget(Primitive):
    """
    Skill: ReacquireTarget
    Responsibility:
      - Short-horizon attempt to regain visual contact with a lost target
      - Rotates in-place in steps until target appears or sweep limit hit
    Inputs:
      - perception
      - kind
      - step_deg, max_sweep_deg
      - max_age_s
    Outputs:
      - SUCCEEDED if target reacquired
      - FAILED if sweep completes without seeing target
      - RUNNING while sweeping
    """

    def __init__(self, *, kind: str, step_deg: float, max_sweep_deg: float, max_age_s: float):
        super().__init__()
        self.kind = kind
        self.step_deg = float(step_deg)
        self.max_sweep_deg = float(max_sweep_deg)
        self.max_age_s = float(max_age_s)

        self._swept = 0.0
        self._dir = 1
        self._child = None
        self.found_target = None

    def start(self, *, motion_backend, **_):
        # Start with no child; update() will either reacquire or rotate-step.
        self._swept = 0.0
        self._dir = 1
        self._child = None
        self.found_target = None

    def update(self, *, motion_backend, perception=None, **_):
        now = time.time()

        # 1) check if target already visible
        if perception is not None:
            t = get_closest_target(perception, self.kind, now=now, max_age_s=self.max_age_s)
            if t is not None:
                self.found_target = t
                return PrimitiveStatus.SUCCEEDED

        # 2) if we are currently rotating, continue that child
        if self._child is not None:
            st = self._child.update(motion_backend=motion_backend)
            if st == PrimitiveStatus.SUCCEEDED:
                self._child = None
                return PrimitiveStatus.RUNNING
            if st == PrimitiveStatus.FAILED:
                self._child = None
                return PrimitiveStatus.FAILED
            return PrimitiveStatus.RUNNING

        # 3) choose next rotate step
        if self._swept >= self.max_sweep_deg:
            return PrimitiveStatus.FAILED

        angle = self._dir * self.step_deg
        self._swept += abs(self.step_deg)

        # reverse direction at end of a sweep half if you want; keeping simple:
        if self._swept >= self.max_sweep_deg:
            # next call will fail unless target found
            pass

        self._child = Rotate(angle_deg=angle)
        self._child.start(motion_backend=motion_backend)
        return PrimitiveStatus.RUNNING

    def stop(self):
        if self._child is not None:
            self._child.stop()
