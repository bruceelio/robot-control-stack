# primitives/manipulation/liftup.py

import time
from primitives.base import Primitive, PrimitiveStatus

class LiftUp(Primitive):
    """
    Raise the lift to the upper position.
    """

    def __init__(self, settle_time=1.0):
        super().__init__()
        self.settle_time = settle_time
        self._start_time = None

    def start(self, *, lvl2, **_):
        print("[LiftUp] start")

        try:
            if hasattr(lvl2, "LIFT_UP"):
                lvl2.LIFT_UP()
            else:
                print("[LiftUp] No lift available on this robot")
        except Exception as e:
            print(f"[LiftUp] ignored ({e})")

        self._start_time = time.time()

    def update(self, **_):
        if self._start_time is None:
            return PrimitiveStatus.FAILED

        if time.time() - self._start_time < self.settle_time:
            return PrimitiveStatus.RUNNING

        print("[LiftUp] succeeded")
        return PrimitiveStatus.SUCCEEDED