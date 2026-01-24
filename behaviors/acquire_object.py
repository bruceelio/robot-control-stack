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
from skills.navigation.search_rotate import SearchRotate


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

        self._search_rotate_skill = None

        # The concrete marker id we actually approached (if any)
        self.acquired_target_id = None

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
        self.exclude_ids: set[int] = set()

        self.locked_target_id = None


    @property
    def acquired_id(self):
        """
        Read this after SUCCEEDED (or after APPROACHING completes) so the caller can:
          - remove from preferred list,
          - add to delivered/blacklist,
          - etc.
        """
        return self.acquired_target_id

    def start(self, *, config, kind=None, seed_target=None, exclude_ids=None, **_):
        print("[ACQUIRE_OBJECT] start")
        self.config = config
        self.kind = kind or config.default_target_kind
        self.locked_target_id = None

        self.phase = "SELECT"
        self.target = None
        self.acquired_target_id = None
        self.exclude_ids = set(exclude_ids) if exclude_ids else set()


        # Reset height model for each acquire run (matches previous behavior)
        self.height_model = HeightModel()

        # --- SELECT skill boot ---
        self._select_skill = SelectTarget(
            kind=self.kind,
            max_age_s=self.config.vision_loss_timeout_s,
            log_every_s=1.0,
            label="ACQUIRE_OBJECT][SELECT",
        )
        self._select_skill.start(seed_target=seed_target, exclude_ids=self.exclude_ids)

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
            return self._select(perception, motion_backend)

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

    def _select(self, perception, motion_backend):

        if self._select_skill is None:
            self._select_skill = SelectTarget(
                kind=self.kind,
                max_age_s=self.config.vision_loss_timeout_s,
                log_every_s=1.0,
                label="ACQUIRE_OBJECT][SELECT",
            )
            self._select_skill.start(seed_target=None, exclude_ids=self.exclude_ids)

        st = self._select_skill.update(perception=perception)

        if st == PrimitiveStatus.RUNNING:
            # No target yet → actively scan
            if self._search_rotate_skill is None:
                self._search_rotate_skill = SearchRotate(
                    kinds=[self.kind],
                    step_deg=15.0,
                    max_deg=360.0,
                    timeout_s=3.0,
                    max_age_s=self.config.vision_loss_timeout_s,
                    label="ACQUIRE_OBJECT][SEARCH",
                )
                self._search_rotate_skill.start(motion_backend=motion_backend)

            sr = self._search_rotate_skill.update(
                motion_backend=motion_backend,
                perception=perception,
            )

            # If a scan finishes without finding anything, restart it
            if sr == PrimitiveStatus.FAILED:
                self._search_rotate_skill.start(motion_backend=motion_backend)

            return self.status

        if st == PrimitiveStatus.FAILED:
            return self.status

        # SUCCEEDED
        self.target = self._select_skill.selected_target
        if self.target is None:
            return self.status

        self.height_model.reset()

        # --- LOCK ---
        try:
            self.locked_target_id = int(self.target.get("id"))
        except Exception:
            self.locked_target_id = None

        print(f"[ACQUIRE_OBJECT][LOCK] locked_target_id={self.locked_target_id}")


        print(
            f"[ACQUIRE_OBJECT][SELECT] target found "
            f"id={self.target.get('id', 'REL')} "
            f"dist={self.target['distance']:.0f} "
            f"bearing={self.target['bearing']:.1f}"
        )

        self._search_rotate_skill = None

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
            locked_target_id=self.locked_target_id,
        )
        # seed_target so it can initialise last_seen
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
                locked_target_id=self.locked_target_id,
            )
            self._approach_skill.start(motion_backend=motion_backend, seed_target=self.target)

        st = self._approach_skill.update(perception=perception, motion_backend=motion_backend)

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            self.status = BehaviorStatus.FAILED
            return self.status

        # SUCCEEDED => ready to grab
        # Capture the *actual* marker id we approached (used later for blacklist/preferred removal)
        tid = None
        if self._approach_skill is not None:
            tid = self._approach_skill.approached_target_id
        if tid is None and self.target is not None:
            try:
                tid = int(self.target.get("id"))
            except Exception:
                tid = None

        self.acquired_target_id = tid
        if tid is not None:
            print(f"[ACQUIRE_OBJECT] approached_target_id={tid}")

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
