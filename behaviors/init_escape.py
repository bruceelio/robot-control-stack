# behaviors/init_escape.py

from behaviors.base import Behavior, BehaviorStatus
from primitives.motion import Drive, Rotate
from primitives.base import PrimitiveStatus


class InitEscape(Behavior):
    """
    Move away from wall and face arena.
    """

    def __init__(self):
        super().__init__()
        self.config = None
        self.phase = None
        self.primitive = None

    def start(self, *, config, motion_backend=None, **_):
        print("[INIT_ESCAPE] start")

        self.config = config
        self.phase = "DRIVE"
        self.primitive = None
        self.status = BehaviorStatus.RUNNING

    def update(self, *, motion_backend, **_):
        """
        lvl2 and localisation are intentionally ignored.
        Motion primitives do not use them.
        """

        # -----------------
        # Phase: DRIVE
        # -----------------
        if self.phase == "DRIVE":
            if self.primitive is None:
                self.primitive = Drive(
                    distance_mm=self.config.init_escape_drive_mm
                )
                self.primitive.start(
                    motion_backend=motion_backend
                )

            prim_status = self.primitive.update(
                motion_backend=motion_backend
            )

            if prim_status == PrimitiveStatus.SUCCEEDED:
                self.primitive = None
                self.phase = "ROTATE"

        # -----------------
        # Phase: ROTATE
        # -----------------
        elif self.phase == "ROTATE":
            if self.primitive is None:
                self.primitive = Rotate(
                    angle_deg=self.config.init_escape_rotate_deg
                )
                self.primitive.start(
                    motion_backend=motion_backend
                )

            prim_status = self.primitive.update(
                motion_backend=motion_backend
            )

            if prim_status == PrimitiveStatus.SUCCEEDED:
                self.status = BehaviorStatus.SUCCEEDED

        return self.status
