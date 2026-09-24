# primitives/manipulation/liftcarry.py

import time

from primitives.base import Primitive, PrimitiveStatus


class LiftCarry(Primitive):
    """
    Raise the lift slightly above the lower position
    for carrying a low object.
    """

    def __init__(self, settle_time=1.0):
        super().__init__()
        self.settle_time = settle_time
        self._start_time = None

    def start(self, *, lvl2, **_):
        print("[LiftCarry] start")

        try:
            if hasattr(lvl2, "LIFT_CARRY"):
                lvl2.LIFT_CARRY()
            else:
                print(
                    "[LiftCarry] No carry lift position "
                    "available on this robot"
                )
        except Exception as e:
            print(f"[LiftCarry] ignored ({e})")

        self._start_time = time.time()

    def update(self, **_):
        if self._start_time is None:
            return PrimitiveStatus.FAILED

        if (
            time.time() - self._start_time
            < self.settle_time
        ):
            return PrimitiveStatus.RUNNING

        print("[LiftCarry] succeeded")
        return PrimitiveStatus.SUCCEEDED