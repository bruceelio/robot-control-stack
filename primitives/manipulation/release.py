# primitives/manipulation/release.py


from primitives.base import Primitive, PrimitiveStatus

class Release(Primitive):
    def __init__(self, settle_time=1.0):
        super().__init__()
        self.settle_time = settle_time
        self._start_time = None
        self._io = None

    def start(self, *, lvl2, **_):
        print("[Release] start")
        try:
            if hasattr(lvl2, "RELEASE"):
                lvl2.RELEASE()
            elif hasattr(lvl2, "GRABBER_OPEN"):
                lvl2.GRABBER_OPEN()
            elif hasattr(lvl2, "VACUUM_OFF"):
                lvl2.VACUUM_OFF()
            else:
                print("[Release] No release available on this robot")
        except Exception as e:
            print(f"[Release] ignored ({e})")

        self._io = lvl2.io
        self._start_time = float(self._io.time())

    def update(self, **_):
        if self._start_time is None:
            return PrimitiveStatus.FAILED

        if float(self._io.time()) - self._start_time < self.settle_time:
            return PrimitiveStatus.RUNNING

        print("[Release] succeeded")
        return PrimitiveStatus.SUCCEEDED


