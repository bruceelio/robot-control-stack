# behaviors/post_pickup_realign.py

from behaviors.base import Behavior, BehaviorStatus
from primitives.motion import Drive, Rotate
from primitives.base import PrimitiveStatus


class PostPickupRealign(Behavior):
    """
    Post-pickup cleanup behavior:
    - reverse to clear the marker / centre cluster
    - rotate to re-establish a useful heading
    """

    def __init__(self):
        super().__init__()
        self.phase = None
        self.active_primitive = None
        self.config = None

    def start(self, *, config, motion_backend, **_):
        print("[POST_PICKUP_REALIGN] start")

        self.config = config
        self.phase = "REVERSE"

        self.active_primitive = Drive(
            distance_mm=-self.config.post_pickup_reverse_mm
        )
        self.active_primitive.start(
            motion_backend=motion_backend
        )

        self.status = BehaviorStatus.RUNNING

    def update(self, *, motion_backend, **_):
        if self.active_primitive is None:
            print("[POST_PICKUP_REALIGN] no active primitive")
            self.status = BehaviorStatus.FAILED
            return self.status

        prim_status = self.active_primitive.update(
            motion_backend=motion_backend
        )

        if prim_status == PrimitiveStatus.RUNNING:
            return self.status

        if prim_status == PrimitiveStatus.FAILED:
            print(f"[POST_PICKUP_REALIGN] {self.phase} failed")
            self.status = BehaviorStatus.FAILED
            return self.status

        # ---------- REVERSE COMPLETE ----------
        if self.phase == "REVERSE":
            print("[POST_PICKUP_REALIGN] reverse complete")

            self.phase = "ROTATE"
            self.active_primitive = Rotate(
                angle_deg=self.config.post_pickup_rotate_deg
            )
            self.active_primitive.start(
                motion_backend=motion_backend
            )
            return self.status

        # ---------- ROTATE COMPLETE ----------
        if self.phase == "ROTATE":
            print("[POST_PICKUP_REALIGN] rotate complete")
            self.status = BehaviorStatus.SUCCEEDED
            return self.status
