# behaviors/acquire_object.py

import time

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus

from skills.navigation.align_to_target import AlignToTarget
from skills.navigation.approach_target import ApproachTarget
from skills.perception.select_target import SelectTarget

from skills.manipulation.grasp_object import GraspObject
from skills.manipulation.verify_grip import VerifyGrip

from navigation.height_model import HeightModel


class AcquireObject(Behavior):
    """
    Pickup-only pipeline:

      SELECT -> ALIGN -> APPROACHING -> GRABBING -> SUCCEEDED
    """

    def __init__(self):
        super().__init__()

        self.config = None
        self.kind = None

        self.phase = "SELECT"
        self.target = None

        # SELECT skill
        self._select_skill = None

        # ALIGN skill
        self._align_skill = None

        # APPROACH skill (new home for the big loop)
        self._approach_skill = None

        # Grab state (via skills)
        self._grab_step = None  # "GRASP" -> "VERIFY"
        self._grasp_skill = None
        self._verify_skill = None

        self.height_model = HeightModel()

    def start(self, *, config, kind=None, seed_target=None, **_):
        print("[ACQUIRE_OBJECT] start")
        self.config = config
        self.kind = kind or config.default_target_kind

        self.phase = "SELECT"
        self.target = None

        # Reset height model for each acquire run (matches previous behavior)
        self.height_model = HeightModel()

        # --- SELECT skill boot ---
        self._select_skill = SelectTarget(
            kind=self.kind,
            max_age_s=self.config.vision_loss_timeout_s,
            log_every_s=1.0,
            label="ACQUIRE_OBJECT][SELECT",
        )
        self._select_skill.start(seed_target=seed_target)

        self._align_skill = None
        self._approach_skill = None

        # reset grab state
        self._grab_step = None
        self._grasp_skill = None
        self._verify_skill = None

        self.status = BehaviorStatus.RUNNING
        return self.status

    def update(self, *, lvl2, perception, localisation, motion_backend, **_):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        if self.phase == "SELECT":
            return self._select(perception)

        if self.phase == "ALIGN":
            return self._align(motion_backend)

        if self.phase == "APPROACHING":
            return self._approach(perception, motion_backend)

        if self.phase == "GRABBING":
            return self._grab(lvl2)

        return self.status

    # -------------------------
    # Phase: SELECT
    # -------------------------

    def _select(self, perception):
        if self._select_skill is None:
            self._select_skill = SelectTarget(
                kind=self.kind,
                max_age_s=self.config.vision_loss_timeout_s,
                log_every_s=1.0,
                label="ACQUIRE_OBJECT][SELECT",
            )
            self._select_skill.start(seed_target=None)

        st = self._select_skill.update(perception=perception)

        if st in (PrimitiveStatus.RUNNING, PrimitiveStatus.FAILED):
            return self.status

        # SUCCEEDED
        self.target = self._select_skill.selected_target
        if self.target is None:
            return self.status

        print(
            f"[ACQUIRE_OBJECT][SELECT] target found "
            f"id={self.target.get('id', 'REL')} "
            f"dist={self.target['distance']:.0f} "
            f"bearing={self.target['bearing']:.1f}"
        )

        self.phase = "ALIGN"
        self._align_skill = None
        return self.status

    # -------------------------
    # Phase: ALIGN
    # -------------------------

    def _align(self, motion_backend):
        bearing = float(self.target["bearing"])

        if self._align_skill is None:
            self._align_skill = AlignToTarget(
                bearing_deg=bearing,
                tolerance_deg=self.config.min_rotate_deg,
                max_rotate_deg=self.config.max_rotate_deg,
            )
            self._align_skill.start(motion_backend=motion_backend)

        st = self._align_skill.update(motion_backend=motion_backend)
        if st == PrimitiveStatus.RUNNING:
            return self.status
        if st == PrimitiveStatus.FAILED:
            self.status = BehaviorStatus.FAILED
            return self.status

        # hand off to approach skill
        self.phase = "APPROACHING"
        self._approach_skill = ApproachTarget(
            config=self.config,
            kind=self.kind,
            height_model=self.height_model,
        )
        # seed_target so it can optionally do an initial align + initialise last_seen
        self._approach_skill.start(motion_backend=motion_backend, seed_target=self.target)
        return self.status

    # -------------------------
    # Phase: APPROACHING (delegated)
    # -------------------------

    def _approach(self, perception, motion_backend):
        if self._approach_skill is None:
            self._approach_skill = ApproachTarget(
                config=self.config,
                kind=self.kind,
                height_model=self.height_model,
            )
            self._approach_skill.start(motion_backend=motion_backend, seed_target=self.target)

        st = self._approach_skill.update(perception=perception, motion_backend=motion_backend)

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            self.status = BehaviorStatus.FAILED
            return self.status

        # SUCCEEDED => ready to grab
        self._grab_step = "GRASP"
        self._grasp_skill = None
        self._verify_skill = None
        self.phase = "GRABBING"
        return self.status

    # -------------------------
    # Phase: GRABBING
    # -------------------------

    def _grab(self, lvl2):
        if self._grab_step is None:
            self._grab_step = "GRASP"

        if self._grab_step == "GRASP":
            if self._grasp_skill is None:
                self._grasp_skill = GraspObject()
                print("[GRAB] starting GraspObject")
                self._grasp_skill.start(lvl2=lvl2, config=self.config)

            st = self._grasp_skill.update(lvl2=lvl2)
            if st == PrimitiveStatus.RUNNING:
                return self.status
            if st == PrimitiveStatus.FAILED:
                print("[GRAB] GraspObject FAILED")
                self.status = BehaviorStatus.FAILED
                return self.status

            print("[GRAB] GraspObject complete")
            self._grab_step = "VERIFY"

        if self._grab_step == "VERIFY":
            if self._verify_skill is None:
                self._verify_skill = VerifyGrip()
                print("[GRAB] starting VerifyGrip")
                self._verify_skill.start(lvl2=lvl2, config=self.config)

            st = self._verify_skill.update(lvl2=lvl2)
            if st == PrimitiveStatus.RUNNING:
                return self.status
            if st == PrimitiveStatus.FAILED:
                print("[GRAB] VerifyGrip FAILED")
                self.status = BehaviorStatus.FAILED
                return self.status

            print("[GRAB] VerifyGrip complete")
            self.status = BehaviorStatus.SUCCEEDED
            return self.status

        return self.status

    def stop(self):
        if self._select_skill is not None:
            self._select_skill.stop()
        if self._align_skill is not None:
            self._align_skill.stop()
        if self._approach_skill is not None:
            self._approach_skill.stop()
        if self._grasp_skill is not None:
            self._grasp_skill.stop()
        if self._verify_skill is not None:
            self._verify_skill.stop()
