# skills/manipulation/prepare_pickup.py

from __future__ import annotations

from primitives.base import Primitive, PrimitiveStatus
from primitives.manipulation import Release, LiftDown


class PreparePickup(Primitive):
    """
    Prepare the manipulator before approaching a pickup target.

    Current sequence:
        - open gripper
        - lower lift
    """

    def __init__(self):
        super().__init__()
        self._status = PrimitiveStatus.RUNNING

    def start(self, *, lvl2, **_):
        self._status = PrimitiveStatus.RUNNING

        try:
            prep_release = Release(settle_time=0.0)
            prep_release.start(lvl2=lvl2)
            print("[PREPARE_PICKUP] RELEASE")
        except Exception as e:
            print(f"[PREPARE_PICKUP] RELEASE failed: {e}")
            self._status = PrimitiveStatus.FAILED
            return self._status

        try:
            prep_liftdown = LiftDown(settle_time=0.0)
            prep_liftdown.start(lvl2=lvl2)
            print("[PREPARE_PICKUP] LIFT DOWN")
        except Exception as e:
            print(f"[PREPARE_PICKUP] LIFT DOWN failed: {e}")
            self._status = PrimitiveStatus.FAILED
            return self._status

        print("[PREPARE_PICKUP] complete")

        self._status = PrimitiveStatus.SUCCEEDED
        return self._status

    def update(self, **_):
        return self._status

    def stop(self, **_):
        pass