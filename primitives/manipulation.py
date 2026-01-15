# primitives/manipulation.py

"""
Manipulation primitives.

Atomic physical interactions.
"""

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


class LiftUp(Primitive):
    """
    Raise the lift to the upper position.
    """

    def __init__(self, settle_time=0.5):
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


class LiftDown(Primitive):
    """
    Lower the lift to the lower position.
    """

    def __init__(self, settle_time=0.5):
        super().__init__()
        self.settle_time = settle_time
        self._start_time = None

    def start(self, *, lvl2, **_):
        print("[LiftDown] start")

        try:
            if hasattr(lvl2, "LIFT_DOWN"):
                lvl2.LIFT_DOWN()
            else:
                print("[LiftDown] No lift available on this robot")
        except Exception as e:
            print(f"[LiftDown] ignored ({e})")

        self._start_time = time.time()

    def update(self, **_):
        if self._start_time is None:
            return PrimitiveStatus.FAILED

        if time.time() - self._start_time < self.settle_time:
            return PrimitiveStatus.RUNNING

        print("[LiftDown] succeeded")
        return PrimitiveStatus.SUCCEEDED
