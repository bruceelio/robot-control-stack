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

    def start(self, *, motion_backend, **_):
        print("[POST_PICKUP_REALIGN] start")

        self.phase = "REVERSE"
        self.active_primitive = Drive(distance_mm=-200)

        self.active_primitive.start(
            motion_backend=motion_backend
        )

    def update(self, *, motion_backend, **_):
        if self.active_primitive is None:
            print("[POST_PICKUP_REALIGN] no active primitive")
            return BehaviorStatus.FAILED

        status = self.active_primitive.update(
            motion_backend=motion_backend
        )

        if status == PrimitiveStatus.RUNNING:
            return BehaviorStatus.RUNNING

        if status == PrimitiveStatus.FAILED:
            print(f"[POST_PICKUP_REALIGN] {self.phase} failed")
            return BehaviorStatus.FAILED

        # ---------- REVERSE COMPLETE ----------
        if self.phase == "REVERSE":
            print("[POST_PICKUP_REALIGN] reverse complete")

            self.phase = "ROTATE"
            self.active_primitive = Rotate(angle_deg=90)

            self.active_primitive.start(
                motion_backend=motion_backend
            )
            return BehaviorStatus.RUNNING

        # ---------- ROTATE COMPLETE ----------
        if self.phase == "ROTATE":
            print("[POST_PICKUP_REALIGN] rotate complete")
            return BehaviorStatus.SUCCEEDED
