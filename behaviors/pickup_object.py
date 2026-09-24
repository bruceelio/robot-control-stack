# behaviors/pickup_object.py

from __future__ import annotations

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus
from primitives.motion import Drive
from primitives.manipulation import LiftMiddle

from skills.navigation.align_to_target import AlignToTarget
from skills.manipulation.grasp_object import GraspObject
from skills.manipulation.verify_grip import VerifyGrip


class PickupObject(Behavior):
    """
    Final pickup behavior after ApproachObject reaches its handoff distance.

    Sequence:
        1. HIGH target: move lift to middle
        2. Final alignment
        3. Blind final drive
        4. Grasp
        5. Verify grip

    distance_mm and bearing_deg are expected to already be
    gripper-relative target geometry.
    """

    def __init__(self):
        super().__init__()

        self.config = None

        self.distance_mm = 0.0
        self.bearing_deg = 0.0
        self.target_is_high = False
        self.final_drive_mm = 0.0

        self._step = None

        self._align = None
        self._drive = None
        self._grasp = None
        self._verify = None

    def start(
        self,
        *,
        config,
        lvl2,
        motion_backend,
        distance_mm: float,
        bearing_deg: float,
        target_is_high: bool,
        **_,
    ):
        self.config = config
        self.status = BehaviorStatus.RUNNING

        self.distance_mm = float(distance_mm)
        self.bearing_deg = float(bearing_deg)
        self.target_is_high = bool(target_is_high)

        marker_push_mm = float(
            getattr(config, "final_approach_marker_push", 50.0)
        )

        self.final_drive_mm = max(
            0.0,
            self.distance_mm + marker_push_mm,
        )

        self._align = None
        self._drive = None
        self._grasp = None
        self._verify = None

        # Preserve current behaviour:
        # HIGH target moves lift before final alignment/drive.
        if self.target_is_high:
            try:
                prep_lift = LiftMiddle(settle_time=0.0)
                prep_lift.start(lvl2=lvl2)
                print("[[PICKUP_OBJECT]] LIFT MIDDLE")
            except Exception as e:
                print(f"[[PICKUP_OBJECT]] LIFT MIDDLE failed: {e}")

        print(
            f"[[PICKUP_OBJECT]] start "
            f"high={self.target_is_high} "
            f"dist={self.distance_mm:.0f}mm "
            f"bearing={self.bearing_deg:.1f}deg "
            f"final_drive={self.final_drive_mm:.0f}mm"
        )

        self._align = AlignToTarget(
            bearing_deg=self.bearing_deg,
            tolerance_deg=0.0,
            max_rotate_deg=float(config.max_rotate_deg),
        )
        self._align.start(motion_backend=motion_backend)

        self._step = "ALIGN"

        return self.status

    def update(self, *, lvl2, motion_backend, **_):

        # --------------------------------------------------
        # Final alignment
        # --------------------------------------------------

        if self._step == "ALIGN":
            st = self._align.update(
                motion_backend=motion_backend
            )

            if st == PrimitiveStatus.RUNNING:
                return st

            if st == PrimitiveStatus.FAILED:
                print("[[PICKUP_OBJECT]] alignment FAILED")
                self.status = BehaviorStatus.FAILED
                return self.status

            print("[[PICKUP_OBJECT]] alignment complete")

            self._drive = Drive(
                distance_mm=self.final_drive_mm
            )
            self._drive.start(
                motion_backend=motion_backend
            )

            self._step = "DRIVE"
            return PrimitiveStatus.RUNNING

        # --------------------------------------------------
        # Blind final drive
        # --------------------------------------------------

        if self._step == "DRIVE":
            st = self._drive.update(
                motion_backend=motion_backend
            )

            if st == PrimitiveStatus.RUNNING:
                return st

            if st == PrimitiveStatus.FAILED:
                print("[[PICKUP_OBJECT]] final drive FAILED")
                self.status = BehaviorStatus.FAILED
                return self.status

            print("[[PICKUP_OBJECT]] final drive complete")

            self._grasp = GraspObject()
            self._grasp.start(
                lvl2=lvl2,
                config=self.config,
            )

            self._step = "GRASP"
            return PrimitiveStatus.RUNNING

        # --------------------------------------------------
        # Grasp
        # --------------------------------------------------

        if self._step == "GRASP":
            st = self._grasp.update(lvl2=lvl2)

            if st == PrimitiveStatus.RUNNING:
                return st

            if st == PrimitiveStatus.FAILED:
                print("[[PICKUP_OBJECT]] GraspObject FAILED")
                self.status = BehaviorStatus.FAILED
                return self.status

            print("[[PICKUP_OBJECT]] GraspObject complete")

            self._verify = VerifyGrip()
            self._verify.start(
                lvl2=lvl2,
                config=self.config,
            )

            self._step = "VERIFY"
            return PrimitiveStatus.RUNNING

        # --------------------------------------------------
        # Verify
        # --------------------------------------------------

        if self._step == "VERIFY":
            st = self._verify.update(lvl2=lvl2)

            if st == PrimitiveStatus.RUNNING:
                return st

            if st == PrimitiveStatus.FAILED:
                print("[[PICKUP_OBJECT]] VerifyGrip FAILED")
                self.status = BehaviorStatus.FAILED
                return self.status

            print("[PICKUP_OBJECT] complete")
            self.status = BehaviorStatus.SUCCEEDED
            return self.status

        self.status = BehaviorStatus.FAILED
        return self.status

    def stop(self, *, motion_backend=None):
        for child in (
            self._align,
            self._drive,
            self._grasp,
            self._verify,
        ):
            if child is None:
                continue

            try:
                if motion_backend is not None:
                    child.stop(motion_backend=motion_backend)
                else:
                    child.stop()
            except TypeError:
                try:
                    child.stop()
                except Exception:
                    pass
            except Exception:
                pass