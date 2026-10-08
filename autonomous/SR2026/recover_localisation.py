# autonomous/SR2026/recover_localisation.py

import math

from autonomous.SR2026.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus
from skills.navigation.search_rotate import SearchRotate


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
        self.active_primitive: SearchRotate | None = None

    def start(self, *, config, motion_backend, **_):
        print("[RECOVER_LOCALISATION] start")

        self.config = config
        self.active_primitive = None
        self.status = BehaviorStatus.RUNNING

        target_angle_deg = float(
            self.config.recover_max_sweep_deg
        )

        angular_speed_rad_s = abs(
            float(
                self.config.servoing_angular_max_rad_s
            )
        )

        timeout_s = 0.0

        if angular_speed_rad_s > 0.0:
            timeout_s = (
                    math.radians(
                        abs(target_angle_deg)
                    )
                    / angular_speed_rad_s
                    + 1.0
            )

        self.active_primitive = SearchRotate(
            target_angle_deg=target_angle_deg,
            timeout_s=timeout_s,
            config=self.config,
            label="RECOVER_LOCALISATION",
        )

        st = self.active_primitive.start(
            motion_backend=motion_backend,
        )

        if st == PrimitiveStatus.FAILED:
            print(
                "[RECOVER_LOCALISATION] "
                "search failed to start"
            )
            self.status = BehaviorStatus.FAILED

        return self.status

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
            io,
            **_,
    ):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        if self.active_primitive is None:
            self.status = BehaviorStatus.FAILED
            return self.status

        found_pose = (
            True
            if localisation.has_pose()
            else None
        )

        st = self.active_primitive.update(
            motion_backend=motion_backend,
            found_item=found_pose,
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.SUCCEEDED:
            print(
                "[RECOVER_LOCALISATION] "
                "pose recovered"
            )
            self.status = BehaviorStatus.SUCCEEDED
            return self.status

        print(
            "[RECOVER_LOCALISATION] "
            "search sweep complete — failed"
        )

        self.status = BehaviorStatus.FAILED
        return self.status

    def stop(
        self,
        *,
        motion_backend=None,
        **_,
    ):
        if self.active_primitive is not None:
            self.active_primitive.stop(
                motion_backend=motion_backend,
            )

        self.active_primitive = None

        if self.status == BehaviorStatus.RUNNING:
            self.status = BehaviorStatus.FAILED

        return self.status
