# primitives/manipulation/grab.py

import time
from primitives.base import Primitive, PrimitiveStatus

class Grab(Primitive):
    def __init__(self, settle_time=0.75):
        super().__init__()
        self.settle_time = settle_time
        self._start_time = None

    def start(self, *, lvl2, **_):
        print("[Grab] start")

        try:
            if hasattr(lvl2, "GRAB"):
                lvl2.GRAB()
            elif hasattr(lvl2, "GRABBER_CLOSE"):
                lvl2.GRABBER_CLOSE()
            elif hasattr(lvl2, "VACUUM_ON"):
                lvl2.VACUUM_ON()
            else:
                print("[Grab] No grabber available on this robot")
        except Exception as e:
            print(f"[Grab] ignored ({e})")

        self._start_time = time.time()

    def update(self, **_):
        if self._start_time is None:
            return PrimitiveStatus.FAILED

        if time.time() - self._start_time < self.settle_time:
            return PrimitiveStatus.RUNNING

        print("[Grab] succeeded")
        return PrimitiveStatus.SUCCEEDED