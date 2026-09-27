# skills/manipulation/prepare_search.py

from __future__ import annotations

from primitives.base import Primitive, PrimitiveStatus
from primitives.manipulation import Release, LiftDown


class PrepareSearch(Primitive):
    """
    Prepare the robot for object search.

    Sequence:
        - open gripper
        - lower lift

    Success:
        - manipulator is configured for searching

    Failure:
        - release command failed
        - lift-down command failed
    """

    def __init__(self):
        super().__init__()
        self._status = PrimitiveStatus.RUNNING

    def start(self, *, lvl2, **_):
        self._status = PrimitiveStatus.RUNNING

        try:
            release = Release(settle_time=0.0)
            release.start(lvl2=lvl2)
            print("[PREPARE_SEARCH] RELEASE")
        except Exception as e:
            print(f"[PREPARE_SEARCH] RELEASE failed: {e}")
            self._status = PrimitiveStatus.FAILED
            return self._status

        try:
            lift_down = LiftDown(settle_time=0.0)
            lift_down.start(lvl2=lvl2)
            print("[PREPARE_SEARCH] LIFT DOWN")
        except Exception as e:
            print(f"[PREPARE_SEARCH] LIFT DOWN failed: {e}")
            self._status = PrimitiveStatus.FAILED
            return self._status

        print("[PREPARE_SEARCH] complete")

        self._status = PrimitiveStatus.SUCCEEDED
        return self._status

    def update(self, **_):
        return self._status

    def stop(self, **_):
        pass