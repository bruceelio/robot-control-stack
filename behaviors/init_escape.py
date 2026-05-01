# behaviors/init_escape.py

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus
from primitives.composites.drive_then_rotate import DriveThenRotate
from primitives.manipulation.liftup import LiftUp


class InitEscape(Behavior):
    """
    Move away from wall and face arena.
    """

    def __init__(self):
        super().__init__()
        self.config = None
        self.primitive = None
        self.step = None

    def start(self, *, config, motion_backend=None, **_):
        print("[INIT_ESCAPE] start")
        self.config = config
        self.primitive = None
        self.step = "LIFT_UP"
        self.status = BehaviorStatus.RUNNING

    def update(self, *, motion_backend, lvl2=None, **_):
        if self.step == "LIFT_UP":
            print("[INIT_ESCAPE] LIFT_UP fire-and-forget")
            try:
                lift = LiftUp()
                lift.start(lvl2=lvl2)
                lift.update(lvl2=lvl2)
            except Exception as e:
                print(f"[INIT_ESCAPE] LIFT_UP ignored failure: {e}")

            self.primitive = None
            self.step = "DRIVE_THEN_ROTATE"
            return self.status

        if self.step == "DRIVE_THEN_ROTATE":
            if self.primitive is None:
                self.primitive = DriveThenRotate(
                    distance_mm=self.config.init_escape_drive_mm,
                    angle_deg=self.config.init_escape_rotate_deg,
                )
                self.primitive.start(motion_backend=motion_backend)

            prim_status = self.primitive.update(motion_backend=motion_backend)

            if prim_status == PrimitiveStatus.SUCCEEDED:
                self.status = BehaviorStatus.SUCCEEDED
            elif prim_status == PrimitiveStatus.FAILED:
                self.status = BehaviorStatus.FAILED

            return self.status

        return self.status