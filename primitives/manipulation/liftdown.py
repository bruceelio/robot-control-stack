# primitives/manipulation/liftdown.py


from primitives.base import Primitive, PrimitiveStatus

class LiftDown(Primitive):
    """
    Lower the lift to the lower position.
    """

    def __init__(self, settle_time=1.0):
        super().__init__()
        self.settle_time = settle_time
        self._start_time = None
        self._io = None

    def start(self, *, lvl2, **_):
        print("[LiftDown] start")

        try:
            if hasattr(lvl2, "LIFT_DOWN"):
                lvl2.LIFT_DOWN()
            else:
                print("[LiftDown] No lift available on this robot")
        except Exception as e:
            print(f"[LiftDown] ignored ({e})")

        self._io = lvl2.io
        self._start_time = float(self._io.time())

    def update(self, **_):
        if self._start_time is None:
            return PrimitiveStatus.FAILED

        if float(self._io.time()) - self._start_time < self.settle_time:
            return PrimitiveStatus.RUNNING

        print("[LiftDown] succeeded")
        return PrimitiveStatus.SUCCEEDED
