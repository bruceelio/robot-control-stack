# behaviors/deliver_object.py

from __future__ import annotations

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus
from primitives.manipulation import Release, LiftDown, LiftUp
from primitives.motion import Drive


class DeliverObject(Behavior):
    """
    Partitioned version of the delivery half:
      (positioning handled elsewhere for now)
      -> LIFT_DOWN -> RELEASE -> REVERSE -> LIFT_UP -> SUCCEEDED

    NOTE:
      This behavior does not own "preferred/delivered list" policy.
      It only reports which marker id should be considered delivered upon success.
    """

    def __init__(self):
        super().__init__()
        self.active_primitive = None
        self.step = None
        self.config = None

        # Marker id that should be marked delivered once this behavior SUCCEEDS
        self.delivered_target_id = None

    @property
    def delivered_id(self):
        return self.delivered_target_id

    def start(self, *, config, delivered_target_id=None, **_):
        print("[DELIVER_OBJECT] start")
        self.config = config
        self.active_primitive = None
        self.step = "FORWARD"
        self.status = BehaviorStatus.RUNNING

        self.delivered_target_id = delivered_target_id
        if self.delivered_target_id is not None:
            print(f"[DELIVER_OBJECT] will mark delivered id={self.delivered_target_id}")

        return self.status

    def update(self, *, lvl2, motion_backend, **_):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        if self.active_primitive is None:
            if self.step == "FORWARD":
                # TODO: temp for ease until final progam
                self.active_primitive = Drive(distance_mm=750.0)
                self.active_primitive.start(motion_backend=motion_backend)

            elif self.step == "LIFT_DOWN":
                self.active_primitive = LiftDown()
                self.active_primitive.start(lvl2=lvl2)

            elif self.step == "RELEASE":
                self.active_primitive = Release()
                self.active_primitive.start(lvl2=lvl2)

            elif self.step == "REVERSE":
                # TODO: make this distance configurable if you want (config.final_dropoff_reverse_mm etc.)
                self.active_primitive = Drive(distance_mm=-250.0)
                self.active_primitive.start(motion_backend=motion_backend)

            elif self.step == "LIFT_UP":
                self.active_primitive = LiftUp()
                self.active_primitive.start(lvl2=lvl2)

            else:
                # DONE
                self.status = BehaviorStatus.SUCCEEDED
                if self.delivered_target_id is not None:
                    print(f"[DELIVER_OBJECT] delivered id={self.delivered_target_id}")
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
        if self.step == "FORWARD":
            self.step = "LIFT_DOWN"
        elif self.step == "LIFT_DOWN":
            self.step = "RELEASE"
        elif self.step == "RELEASE":
            self.step = "REVERSE"
        elif self.step == "REVERSE":
            self.step = "LIFT_UP"
        elif self.step == "LIFT_UP":
            self.step = "DONE"

        return self.status
