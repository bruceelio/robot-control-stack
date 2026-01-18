# behaviors/init_escape.py

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus
from primitives.drive_then_rotate import DriveThenRotate


class InitEscape(Behavior):
    """
    Move away from wall and face arena.
    """

    def __init__(self):
        super().__init__()
        self.config = None
        self.primitive = None

    def start(self, *, config, motion_backend=None, **_):
        print("[INIT_ESCAPE] start")
        self.config = config
        self.primitive = None
        self.status = BehaviorStatus.RUNNING

    def update(self, *, motion_backend, **_):
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
