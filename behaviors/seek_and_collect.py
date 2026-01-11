import time

from behaviors.base import Behavior, BehaviorStatus
from primitives.motion import Rotate, Drive
from primitives.manipulation import Grab
from primitives.base import PrimitiveStatus
from navigation import get_closest_target

from config.base import (
    ALIGN_THRESHOLD_DEG,
    MAX_ROTATE_DEG,
    MIN_DRIVE_MM,
    MAX_DRIVE_MM,
    GRAB_DISTANCE_MM,
    CAMERA_SETTLE_TIME,
)


class SeekAndCollect(Behavior):
    def __init__(self, tolerance_mm=50, max_drive_mm=500):
        super().__init__()
        self.tolerance_mm = tolerance_mm
        self.max_drive_mm = max_drive_mm

        self.state = None
        self.target = None
        self.active_primitive = None
        self.last_action = None  # "rotate" or "drive"
        self.settle_until = None

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
            return self._approach(perception, motion_backend)

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

    def _approach(self, perception, motion_backend):
        # --- Settling after drive ---
        if self.settle_until is not None:
            if time.time() < self.settle_until:
                return self.status
            self.settle_until = None

        # If no active primitive, decide next action
        if self.active_primitive is None:
            self.target = get_closest_target(perception, self.kind)

            if self.target is None:
                # Temporary vision loss → wait and retry
                return self.status

            distance = self.target["distance"]
            bearing = self.target["bearing"]

            # --- Stop condition ---
            if distance <= GRAB_DISTANCE_MM:
                self.state = "GRABBING"
                return self.status

            # --- ROTATE phase ---
            if abs(bearing) > ALIGN_THRESHOLD_DEG:
                angle = max(
                    -MAX_ROTATE_DEG,
                    min(MAX_ROTATE_DEG, bearing)
                )

                self.active_primitive = Rotate(angle_deg=angle)
                self.last_action = "rotate"
                self.active_primitive.start(
                    motion_backend=motion_backend
                )
                return self.status

            # --- DRIVE phase ---
            drive_mm = max(
                MIN_DRIVE_MM,
                min(MAX_DRIVE_MM, distance - GRAB_DISTANCE_MM)
            )

            self.active_primitive = Drive(distance_mm=drive_mm)
            self.last_action = "drive"
            self.active_primitive.start(
                motion_backend=motion_backend
            )
            return self.status

        # --- Primitive running ---
        prim_status = self.active_primitive.update(
            motion_backend=motion_backend
        )

        if prim_status == PrimitiveStatus.SUCCEEDED:
            self.active_primitive = None

            if self.last_action == "drive":
                self.settle_until = time.time() + CAMERA_SETTLE_TIME

        if prim_status == PrimitiveStatus.FAILED:
            self.active_primitive = None
            self.status = BehaviorStatus.FAILED

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
