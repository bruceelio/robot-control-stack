# skills/manipulation/verify_grip.py

import time
from primitives.base import PrimitiveStatus


class VerifyGrip:
    """
    Placeholder verification step implemented as a Primitive so it plugs into the
    existing AcquireObject._grab() state machine without structural changes.

    For now: wait a short settle time and declare success.
    """

    def __init__(self):
        self._done_at = None

    def start(self, *, settle_s=0.2, **_):
        self._done_at = time.time() + float(settle_s)
        print("[VERIFY_GRIP] start")
        return PrimitiveStatus.RUNNING

    def update(self, *, lvl2=None, **_):
        if self._done_at is None:
            return PrimitiveStatus.FAILED

        if time.time() >= self._done_at:
            print("[VERIFY_GRIP] ok (stub)")
            return PrimitiveStatus.SUCCEEDED

        return PrimitiveStatus.RUNNING
