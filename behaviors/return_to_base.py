# behaviors/return_to_base.py

from behaviors.base import Behavior, BehaviorStatus
from primitives.motion import Drive
from primitives.manipulation import LiftDown, Release, LiftUp
from primitives.base import PrimitiveStatus


class ReturnToBase(Behavior):
    """
    Simple ReturnToBase:
    Drop carried marker, back away, then resume SeekAndCollect.
    """

    def __init__(self):
        super().__init__()
        self.phase = None
        self.primitive = None

    def start(self, **_):
        print("[RETURN_TO_BASE] start")
        self.phase = "LIFT_DOWN"
        self.primitive = None
        self.status = BehaviorStatus.RUNNING

    def update(self, *, lvl2, motion_backend, **_):

        # -----------------
        # Phase: LIFT_DOWN
        # -----------------
        if self.phase == "LIFT_DOWN":
            if self.primitive is None:
                self.primitive = LiftDown()
                self.primitive.start(lvl2=lvl2)

            status = self.primitive.update()
            if status == PrimitiveStatus.SUCCEEDED:
                self.primitive = None
                self.phase = "RELEASE"

        # -----------------
        # Phase: RELEASE
        # -----------------
        elif self.phase == "RELEASE":
            if self.primitive is None:
                self.primitive = Release()
                self.primitive.start(lvl2=lvl2)

            status = self.primitive.update()
            if status == PrimitiveStatus.SUCCEEDED:
                self.primitive = None
                self.phase = "BACK_AWAY"

        # -----------------
        # Phase: BACK_AWAY
        # -----------------
        elif self.phase == "BACK_AWAY":
            if self.primitive is None:
                self.primitive = Drive(distance_mm=-2500)
                self.primitive.start(motion_backend=motion_backend)

            status = self.primitive.update(motion_backend=motion_backend)
            if status == PrimitiveStatus.SUCCEEDED:
                self.primitive = None
                self.phase = "LIFT_UP"

        # -----------------
        # Phase: LIFT_UP
        # -----------------
        elif self.phase == "LIFT_UP":
            if self.primitive is None:
                self.primitive = LiftUp()
                self.primitive.start(lvl2=lvl2)

            status = self.primitive.update()
            if status == PrimitiveStatus.SUCCEEDED:
                self.status = BehaviorStatus.SUCCEEDED

        return self.status
