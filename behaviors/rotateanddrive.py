# behaviors/rotateanddrive.py

from behaviors.base import Behavior, BehaviorStatus
from primitives.motion import Rotate, Drive
from primitives.base import PrimitiveStatus


class RotateAndDrive(Behavior):
    """
    Validation behavior:
    - rotate 180 degrees
    - drive forward 1000 mm
    """

    def __init__(self):
        super().__init__()
        self.phase = None
        self.active_primitive = None

    def start(self, *, motion_backend, **_):
        print("[ROTATE_AND_DRIVE] start")
        self.phase = "ROTATE"
        self.active_primitive = Rotate(angle_deg=180)
        self.active_primitive.start(
            motion_backend=motion_backend
        )

    def update(self, *, motion_backend, **_):
        if self.active_primitive is None:
            return BehaviorStatus.FAILED

        status = self.active_primitive.update(
            motion_backend=motion_backend
        )

        if status == PrimitiveStatus.RUNNING:
            return BehaviorStatus.RUNNING

        if status == PrimitiveStatus.FAILED:
            print("[ROTATE_AND_DRIVE] primitive failed")
            return BehaviorStatus.FAILED

        # ---------- ROTATE COMPLETE ----------
        if self.phase == "ROTATE":
            print("[ROTATE_AND_DRIVE] rotate complete")
            self.phase = "DRIVE"
            self.active_primitive = Drive(distance_mm=1000)
            self.active_primitive.start(
                motion_backend=motion_backend
            )
            return BehaviorStatus.RUNNING

        # ---------- DRIVE COMPLETE ----------
        if self.phase == "DRIVE":
            print("[ROTATE_AND_DRIVE] drive complete")
            return BehaviorStatus.SUCCEEDED
