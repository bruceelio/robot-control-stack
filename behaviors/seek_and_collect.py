# behaviors/seek_and_collect.py

from behaviors.base import Behavior, BehaviorStatus
from primitives.motion import Rotate
from primitives.manipulation import Grab
from primitives.base import PrimitiveStatus

from navigation import get_closest_target, drive_to_target


class SeekAndCollect(Behavior):
    def __init__(self, tolerance_mm=50, max_drive_mm=500):
        super().__init__()
        self.tolerance_mm = tolerance_mm
        self.max_drive_mm = max_drive_mm

        self.state = None
        self.target = None
        self.active_primitive = None

    def start(self, *, kind="acidic"):
        self.state = "SEARCHING"
        self.kind = kind
        self.target = None
        self.active_primitive = None
        self.status = BehaviorStatus.RUNNING

    def update(self, *, lvl2, perception, localisation, motion_backend):
        if self.state == "SEARCHING":
            return self._search(perception, motion_backend)

        if self.state == "APPROACHING":
            return self._approach(lvl2, localisation)

        if self.state == "GRABBING":
            return self._grab(lvl2)

        return self.status

    # -------------------------
    # Behavior phases
    # -------------------------

    def _search(self, perception, motion_backend):
        self.target = get_closest_target(perception, self.kind)
        if self.target is None:
            self.status = BehaviorStatus.FAILED
            return self.status

        if self.active_primitive is None:
            self.active_primitive = Rotate(
                angle_deg=self.target["bearing"]
            )
            self.active_primitive.start(
                motion_backend=motion_backend
            )

        prim_status = self.active_primitive.update(
            motion_backend=motion_backend
        )

        if prim_status == PrimitiveStatus.SUCCEEDED:
            self.active_primitive = None
            self.state = "APPROACHING"

        return self.status

    def _approach(self, lvl2, localisation):
        distance = self.target["distance"]

        if distance <= self.tolerance_mm:
            self.state = "GRABBING"
            return self.status

        # Legacy drive (unchanged for now)
        position, heading = drive_to_target(
            lvl2,
            self.target,
            localisation.position,
            localisation.heading,
            max_drive_mm=self.max_drive_mm,
            tolerance_mm=self.tolerance_mm,
        )

        localisation.position = position
        localisation.heading = heading

        return self.status

    def _grab(self, lvl2):
        if self.active_primitive is None:
            self.active_primitive = Grab()
            self.active_primitive.start(lvl2=lvl2)

        prim_status = self.active_primitive.update()

        if prim_status == PrimitiveStatus.SUCCEEDED:
            self.active_primitive = None
            self.status = BehaviorStatus.SUCCEEDED

        if prim_status == PrimitiveStatus.FAILED:
            self.active_primitive = None
            self.status = BehaviorStatus.FAILED

        return self.status
