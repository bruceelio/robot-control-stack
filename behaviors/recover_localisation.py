# behaviors/recover_localisation.py

import time
from behaviors.base import Behavior, BehaviorStatus
from primitives.motion import Rotate
from primitives.base import PrimitiveStatus


class RecoverLocalisation(Behavior):
    """
    Rotate in place in fixed increments until localisation is recovered.

    Success:
        - localisation pose valid

    Failure:
        - Full sweep without recovery
    """

    def __init__(self):
        super().__init__()
        self.config = None
        self.total_rotated = 0
        self.active_primitive = None
        self.settle_until = None

    def start(self, *, config, motion_backend, **_):
        print("[RECOVER_LOCALISATION] start")

        self.config = config
        self.total_rotated = 0
        self.active_primitive = None
        self.settle_until = None
        self.status = BehaviorStatus.RUNNING

        self._start_next_rotation(motion_backend)

    def _start_next_rotation(self, motion_backend):
        print(
            f"[RECOVER_LOCALISATION] rotating {self.config.recover_step_deg}° "
            f"(total={self.total_rotated}°)"
        )

        self.active_primitive = Rotate(
            angle_deg=self.config.recover_step_deg
        )
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
                return self.status

            # settle complete — check for recovery
            self.settle_until = None

            if localisation.has_pose():
                print("[RECOVER_LOCALISATION] pose recovered")
                self.status = BehaviorStatus.SUCCEEDED
                return self.status

            if self.total_rotated >= self.config.recover_max_sweep_deg:
                print("[RECOVER_LOCALISATION] full sweep complete — failed")
                self.status = BehaviorStatus.FAILED
                return self.status

            self._start_next_rotation(motion_backend)
            return self.status

        # ---------- SUCCESS CHECK ----------
        if localisation.has_pose():
            print("[RECOVER_LOCALISATION] pose recovered")
            self.status = BehaviorStatus.SUCCEEDED
            return self.status

        # ---------- ACTIVE ROTATION ----------
        if self.active_primitive is None:
            self.status = BehaviorStatus.FAILED
            return self.status

        prim_status = self.active_primitive.update(
            motion_backend=motion_backend
        )

        if prim_status == PrimitiveStatus.RUNNING:
            return self.status

        if prim_status == PrimitiveStatus.FAILED:
            print("[RECOVER_LOCALISATION] rotate failed")
            self.status = BehaviorStatus.FAILED
            return self.status

        # ---------- STEP COMPLETE ----------
        self.total_rotated += self.config.recover_step_deg

        # begin settle phase
        self.settle_until = (
            time.monotonic() + self.config.recover_settle_time
        )
        self.active_primitive = None

        return self.status
