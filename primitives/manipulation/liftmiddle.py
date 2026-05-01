# primitives/manipulation/liftmiddle.py

import time
from primitives.base import Primitive, PrimitiveStatus

class LiftMiddle(Primitive):
    """
    Raise the lift to the middle position.
    """

    def __init__(self, settle_time=1.0):
        super().__init__()
        self.settle_time = settle_time
        self._start_time = None

    def start(self, *, lvl2, **_):
        print("[LiftMiddle] start")
        print(f"[LiftMiddle][DEBUG] lvl2={lvl2}")
        print(f"[LiftMiddle][DEBUG] type={type(lvl2)} module={type(lvl2).__module__}")
        print(f"[LiftMiddle][DEBUG] has LIFT_DOWN={hasattr(lvl2, 'LIFT_DOWN')}")
        print(f"[LiftMiddle][DEBUG] has LIFT_MIDDLE={hasattr(lvl2, 'LIFT_MIDDLE')}")
        print(f"[LiftMiddle][DEBUG] has LIFT_UP={hasattr(lvl2, 'LIFT_UP')}")

        try:
            if hasattr(lvl2, "LIFT_MIDDLE"):
                lvl2.LIFT_MIDDLE()
            else:
                print("[LiftMiddle] No lift available on this robot")
        except Exception as e:
            print(f"[LiftMiddle] ignored ({e})")

        self._start_time = time.time()

    def update(self, **_):
        if self._start_time is None:
            return PrimitiveStatus.FAILED

        if time.time() - self._start_time < self.settle_time:
            return PrimitiveStatus.RUNNING

        print("[LiftMiddle] succeeded")
        return PrimitiveStatus.SUCCEEDED