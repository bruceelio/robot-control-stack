# behaviors/recover_localisation.py

import time
from behaviors.base import Behavior, BehaviorStatus
from primitives.motion import Rotate
from primitives.base import PrimitiveStatus


class RecoverLocalisation(Behavior):
    """
    Rotate in place in fixed increments until localisation is recovered.

    Success:
        - >=2 arena markers visible OR localisation pose valid

    Failure:
        - Full 360° rotation without recovery
    """

    STEP_DEG = 30
    MAX_SWEEP = 360
    SETTLE_TIME = 0.30  # seconds to let vision stabilise after rotation

    def __init__(self):
        super().__init__()
        self.total_rotated = 0
        self.active_primitive = None
        self.settle_until = None

    def start(self, *, motion_backend, **_):
        print("[RECOVER_LOCALISATION] start")
        self.total_rotated = 0
        self.active_primitive = None
        self.settle_until = None
        self._start_next_rotation(motion_backend)

    def _start_next_rotation(self, motion_backend):
        print(f"[RECOVER_LOCALISATION] rotating {self.STEP_DEG}° "
              f"(total={self.total_rotated}°)")
        self.active_primitive = Rotate(angle_deg=self.STEP_DEG)
        self.active_primitive.start(
            motion_backend=motion_backend
        )

    def update(
        self,
        *,
        motion_backend,
        perception,
        localisation,
        **_
    ):

        # ---------- SETTLE PHASE ----------
        if self.settle_until is not None:
            if time.monotonic() < self.settle_until:
                return BehaviorStatus.RUNNING

            # settle complete — check for recovery
            self.settle_until = None

            if localisation.has_pose():
                print("[RECOVER_LOCALISATION] pose recovered")
                return BehaviorStatus.SUCCEEDED

            if self.total_rotated >= self.MAX_SWEEP:
                print("[RECOVER_LOCALISATION] full sweep complete — failed")
                return BehaviorStatus.FAILED

            self._start_next_rotation(motion_backend)
            return BehaviorStatus.RUNNING

        # ---------- SUCCESS CHECK ----------
        if localisation.has_pose():
            print("[RECOVER_LOCALISATION] pose recovered")
            return BehaviorStatus.SUCCEEDED

        # ---------- ACTIVE ROTATION ----------
        if self.active_primitive is None:
            return BehaviorStatus.FAILED

        status = self.active_primitive.update(
            motion_backend=motion_backend
        )

        if status == PrimitiveStatus.RUNNING:
            return BehaviorStatus.RUNNING

        if status == PrimitiveStatus.FAILED:
            print("[RECOVER_LOCALISATION] rotate failed")
            return BehaviorStatus.FAILED

        # ---------- STEP COMPLETE ----------
        self.total_rotated += self.STEP_DEG

        # begin settle phase
        self.settle_until = time.monotonic() + self.SETTLE_TIME
        self.active_primitive = None

        return BehaviorStatus.RUNNING

