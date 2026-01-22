# behaviors/deliver_object.py

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus
from primitives.manipulation import Release, LiftDown, LiftUp
from primitives.motion import Drive


class DeliverObject(Behavior):
    """
    Partitioned version of the delivery half:
      (positioning handled elsewhere for now)
      -> LIFT_DOWN -> RELEASE -> REVERSE -> LIFT_UP -> SUCCEEDED
    """

    def __init__(self):
        super().__init__()
        self.active_primitive = None
        self.step = None
        self.config = None

    def start(self, *, config, **_):
        print("[DELIVER_OBJECT] start")
        self.config = config
        self.active_primitive = None
        self.step = "LIFT_DOWN"
        self.status = BehaviorStatus.RUNNING
        return self.status

    def update(self, *, lvl2, motion_backend, **_):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        if self.active_primitive is None:
            if self.step == "LIFT_DOWN":
                self.active_primitive = LiftDown()
                self.active_primitive.start(lvl2=lvl2)

            elif self.step == "RELEASE":
                self.active_primitive = Release()
                self.active_primitive.start(lvl2=lvl2)

            elif self.step == "REVERSE":
                self.active_primitive = Drive(distance_mm=-250.0)
                self.active_primitive.start(motion_backend=motion_backend)

            elif self.step == "LIFT_UP":
                self.active_primitive = LiftUp()
                self.active_primitive.start(lvl2=lvl2)

            else:
                self.status = BehaviorStatus.SUCCEEDED
                return self.status

        # Dispatch: motion primitives need motion_backend
        if isinstance(self.active_primitive, Drive):
            st = self.active_primitive.update(motion_backend=motion_backend)
        else:
            st = self.active_primitive.update()

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            self.active_primitive = None
            self.status = BehaviorStatus.FAILED
            return self.status

        # SUCCEEDED -> advance
        self.active_primitive = None
        if self.step == "LIFT_DOWN":
            self.step = "RELEASE"
        elif self.step == "RELEASE":
            self.step = "REVERSE"
        elif self.step == "REVERSE":
            self.step = "LIFT_UP"
        elif self.step == "LIFT_UP":
            self.step = "DONE"

        return self.status
