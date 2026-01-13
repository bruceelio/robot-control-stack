# primitives/manipulation.py

"""
Manipulation primitives.

Atomic physical interactions.
"""

import time
from primitives.base import Primitive, PrimitiveStatus


class Grab(Primitive):
    """
    Grab an object.

    This primitive issues a GRAB command to Level2 and
    waits for a fixed settle time before succeeding.
    """

    def __init__(self, settle_time=0.75):
        self.settle_time = settle_time
        self._start_time = None

    def start(self, *, lvl2, **_):
        print("[Grab] start")
        lvl2.GRAB()
        self._start_time = time.time()

    def update(self, **_):
        if self._start_time is None:
            return PrimitiveStatus.FAILED

        if time.time() - self._start_time < self.settle_time:
            return PrimitiveStatus.RUNNING

        print("[Grab] succeeded")
        return PrimitiveStatus.SUCCEEDED

    def stop(self, *, lvl2, **_):
        # Optional safety: stop actuation if aborted
        pass


class Release(Primitive):
    """
    Release a held object.
    """

    def __init__(self, settle_time=0.5):
        self.settle_time = settle_time
        self._start_time = None

    def start(self, *, lvl2, **_):
        print("[Release] start")
        lvl2.RELEASE()
        self._start_time = time.time()

    def update(self, **_):
        if self._start_time is None:
            return PrimitiveStatus.FAILED

        if time.time() - self._start_time < self.settle_time:
            return PrimitiveStatus.RUNNING

        print("[Release] succeeded")
        return PrimitiveStatus.SUCCEEDED


class StubGrabPrimitive(Primitive):
    """
    Temporary grab primitive used to allow SeekAndCollect
    to complete in simulation before the real grabber is validated.
    """

    def __init__(self, duration=0.5):
        super().__init__()
        self.duration = duration
        self.start_time = None

    def start(self, *, lvl2, **_):
        print("[StubGrab] start")
        self.start_time = time.time()

        # Optional: exercise actuator path
        try:
            if hasattr(lvl2, "GRAB"):
                lvl2.GRAB()
        except Exception as e:
            print(f"[StubGrab] GRAB ignored ({e})")

    def update(self, **_):
        if self.start_time is None:
            return PrimitiveStatus.FAILED

        if time.time() - self.start_time < self.duration:
            return PrimitiveStatus.RUNNING

        print("[StubGrab] complete")
        return PrimitiveStatus.SUCCEEDED

