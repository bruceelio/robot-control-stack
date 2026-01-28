# behaviors/acquire_object.py

import time
import inspect

from behaviors.base import Behavior, BehaviorStatus
from behaviors.global_search import GlobalSearchStub
from behaviors.recover_lost_target import RecoverLostTarget

from navigation.height_model import HeightModel

from policies.vision_grace_period import VisionGracePeriod

from primitives.base import PrimitiveStatus

from skills.manipulation.grasp_object import GraspObject
from skills.manipulation.verify_grip import VerifyGrip
from skills.navigation.align_to_target import AlignToTarget
from skills.navigation.approach_target import ApproachTarget
from skills.navigation.backoff_scan import BackoffScan
from skills.navigation.search_rotate import SearchRotate
from skills.perception.select_target import SelectTarget
from skills.perception.track_object import TrackObject


class AcquireObject(Behavior):
    """
    Pickup-only pipeline:

      SELECT -> ALIGN -> APPROACHING -> GRABBING -> SUCCEEDED

    Failure-loop fix:
      - When vision loss persists beyond VisionGracePeriod, delegate recovery to RecoverLostTarget:
          ReacquireTarget (locked) -> BackoffScan -> SearchRotate (unlock + 360)
      - Any success funnels back through TrackObject before re-entering ALIGN/APPROACH.
    """

    def __init__(self):
        super().__init__()

        self.config = None
        self.kind = None

        self.phase = "SELECT"
        self.target = None

        # The concrete marker id we actually approached (if any)
        self.acquired_target_id = None

        # SELECT skill + scan helper
        self._select_skill = None
        self._search_rotate_skill = None

        # ALIGN + APPROACH
        self._align_skill = None
        self._approach_skill = None

        # Grab state
        self._grab_step = None  # "GRASP" -> "VERIFY"
        self._grasp_skill = None
        self._verify_skill = None

        self.height_model = HeightModel()
        self.exclude_ids: set[int] = set()

        # Locked target id (single source of “what we are trying to chase”)
        self.locked_target_id = None

        # Locked-target tracking (single source of truth for visibility/loss)
        self._tracker: TrackObject | None = None
        self.track = None

        # Vision loss policy
        self._vision_grace: VisionGracePeriod | None = None

        # Failure ladder for SELECT stall only (kept)
        self._backoff_scan_skill = None
        self._global_search_behavior: GlobalSearchStub | None = None

        # New: lost-target recovery behavior
        self._recover_lost_target_behavior: RecoverLostTarget | None = None

        # SELECT stall watchdog
        self._select_started_s = None
        self._select_stall_count = 0

    @property
    def acquired_id(self):
        return self.acquired_target_id

    def start(self, *, config, kind=None, seed_target=None, exclude_ids=None, **_):
        print("[ACQUIRE_OBJECT] start")
        self.config = config
        self.kind = kind or config.default_target_kind

        self.exclude_ids = set(exclude_ids) if exclude_ids else set()
        self.locked_target_id = None
        self.target = None
        self.acquired_target_id = None

        # Reset height model for each acquire run
        self.height_model = HeightModel()

        # Tracker + policy
        self._tracker = TrackObject(kind=self.kind)
        self._tracker.reset(locked_target_id=None, kind=self.kind)
        self.track = None

        self._vision_grace = VisionGracePeriod(vision_grace_s=self.config.vision_grace_period_s)

        # SELECT boot
        self.phase = "SELECT"
        self._select_skill = SelectTarget(
            kind=self.kind,
            max_age_s=self.config.vision_loss_timeout_s,
            log_every_s=1.0,
            label="ACQUIRE_OBJECT][SELECT",
        )
        self._select_skill.start(seed_target=seed_target, exclude_ids=self.exclude_ids)

        self._search_rotate_skill = None
        self._align_skill = None
        self._approach_skill = None

        # Recovery state reset
        self._recover_lost_target_behavior = None

        # SELECT ladder state reset
        self._backoff_scan_skill = None
        self._global_search_behavior = None

        # grab reset
        self._grab_step = None
        self._grasp_skill = None
        self._verify_skill = None

        # watchdog
        self._select_started_s = time.time()
        self._select_stall_count = 0

        self.status = BehaviorStatus.RUNNING
        return self.status

    def update(self, *, lvl2, perception, localisation, motion_backend, **_):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        # Defensive: ensure tracker & policy exist
        if self._tracker is None:
            self._tracker = TrackObject(kind=self.kind)
            self._tracker.reset(locked_target_id=self.locked_target_id, kind=self.kind)
        if self._vision_grace is None:
            self._vision_grace = VisionGracePeriod(vision_grace_s=self.config.vision_grace_period_s)

        # Update tracker once per tick so all phases read the same truth.
        self.track = self._tracker.update(
            perception_objects=getattr(perception, "objects", perception),
            now_s=time.time(),
            locked_target_id=self.locked_target_id,
            kind=self.kind,
        )

        if self.phase == "SELECT":
            return self._select(perception, motion_backend)

        if self.phase == "ALIGN":
            return self._align(motion_backend)

        if self.phase == "APPROACHING":
            return self._approach(perception, localisation, motion_backend)

        if self.phase == "RECOVER_LOST_TARGET":
            return self._recover_lost_target(perception, localisation, motion_backend)

        if self.phase == "TRACK_AFTER_RECOVER":
            return self._track_after_recover(motion_backend)

        if self.phase == "BACKOFF_SCAN":
            return self._backoff_scan(perception, motion_backend)

        if self.phase == "GLOBAL_SEARCH":
            return self._global_search(perception, motion_backend)

        if self.phase == "GRABBING":
            return self._grab(lvl2)

        return self.status

    # -------------------------
    # Helpers: phase transitions
    # -------------------------

    def _enter_select(self, *, motion_backend=None, reason: str = ""):
        if reason:
            print(f"[ACQUIRE_OBJECT] -> SELECT ({reason})")

        self._safe_stop(self._search_rotate_skill, motion_backend=motion_backend)
        self._search_rotate_skill = None

        self._safe_stop(self._select_skill, motion_backend=motion_backend)
        self._select_skill = None

        self.target = None
        self.locked_target_id = None

        if self._tracker is not None:
            self._tracker.reset(locked_target_id=None, kind=self.kind)
        self.track = None

        self._select_started_s = time.time()
        self._select_stall_count = 0

        self.phase = "SELECT"

    def _enter_backoff_scan(self, *, motion_backend=None, reason: str = ""):
        if reason:
            print(f"[ACQUIRE_OBJECT] -> BACKOFF_SCAN ({reason})")

        self._safe_stop(self._search_rotate_skill, motion_backend=motion_backend)
        self._search_rotate_skill = None

        self._safe_stop(self._select_skill, motion_backend=motion_backend)
        self._select_skill = None

        self.target = None
        self.locked_target_id = None

        if self._tracker is not None:
            self._tracker.reset(locked_target_id=None, kind=self.kind)
        self.track = None

        self.phase = "BACKOFF_SCAN"
        self._backoff_scan_skill = None

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

            self._select_started_s = time.time()
            self._select_stall_count = 0

        st = self._select_skill.update(perception=perception)

        if st == PrimitiveStatus.RUNNING:
            now = time.time()

            # Keep current behavior: scan while selecting (unchanged here)
            if self._search_rotate_skill is None:
                self._search_rotate_skill = SearchRotate(
                    kinds=[self.kind],
                    step_deg=15.0,
                    max_deg=360.0,
                    timeout_s=float(getattr(self.config, "select_search_timeout_s", 3.0)),
                    max_age_s=self.config.vision_loss_timeout_s,
                    label="ACQUIRE_OBJECT][SEARCH",
                )
                self._search_rotate_skill.start(motion_backend=motion_backend)

            sr = self._search_rotate_skill.update(
                motion_backend=motion_backend,
                perception=perception,
            )

            if sr in (PrimitiveStatus.SUCCEEDED, PrimitiveStatus.FAILED):
                self._select_stall_count += 1
                print(
                    f"[ACQUIRE_OBJECT][SELECT] scan finished (sr={sr.name}) "
                    f"stalls={self._select_stall_count}"
                )
                self._search_rotate_skill.start(motion_backend=motion_backend)

            # Watchdog
            select_timeout_s = float(getattr(self.config, "select_timeout_s", 6.0))
            max_stalls = int(getattr(self.config, "select_max_stalls_before_escalate", 2))

            if self._select_started_s is None:
                self._select_started_s = now
            elapsed = now - self._select_started_s

            if elapsed > select_timeout_s or self._select_stall_count >= max_stalls:
                print(
                    f"[ACQUIRE_OBJECT][SELECT] STALLED elapsed={elapsed:.2f}s "
                    f"timeout={select_timeout_s:.2f}s stalls={self._select_stall_count}/{max_stalls} "
                    f"-> BACKOFF_SCAN"
                )
                self._enter_backoff_scan(motion_backend=motion_backend, reason="select_stalled")
                return self.status

            return self.status

        if st == PrimitiveStatus.FAILED:
            print("[ACQUIRE_OBJECT][SELECT] SelectTarget FAILED -> BACKOFF_SCAN")
            self._enter_backoff_scan(motion_backend=motion_backend, reason="select_failed")
            return self.status

        # SUCCEEDED
        self.target = self._select_skill.selected_target
        if self.target is None:
            print("[ACQUIRE_OBJECT][SELECT] SUCCEEDED but selected_target is None -> SELECT")
            self._enter_select(motion_backend=motion_backend, reason="no_selected_target")
            return self.status

        self.height_model.reset()

        try:
            self.locked_target_id = int(self.target.get("id"))
        except Exception:
            self.locked_target_id = None

        print(f"[ACQUIRE_OBJECT][LOCK] locked_target_id={self.locked_target_id}")

        if self._tracker is not None:
            self._tracker.reset(locked_target_id=self.locked_target_id, kind=self.kind)

        print(
            f"[ACQUIRE_OBJECT][SELECT] target found "
            f"id={self.target.get('id', 'REL')} "
            f"dist={self.target['distance']:.0f} "
            f"bearing={self.target['bearing']:.1f}"
        )

        self._safe_stop(self._search_rotate_skill, motion_backend=motion_backend)
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

        self.phase = "APPROACHING"
        self._approach_skill = ApproachTarget(
            config=self.config,
            kind=self.kind,
            height_model=self.height_model,
            locked_target_id=self.locked_target_id,
        )
        self._approach_skill.start(motion_backend=motion_backend, seed_target=self.target)
        return self.status

    # -------------------------
    # Phase: APPROACHING
    # -------------------------

    def _approach(self, perception, localisation, motion_backend):
        if self._approach_skill is None:
            self._approach_skill = ApproachTarget(
                config=self.config,
                kind=self.kind,
                height_model=self.height_model,
                locked_target_id=self.locked_target_id,
            )
            self._approach_skill.start(motion_backend=motion_backend, seed_target=self.target)

        snap = self.track

        st = self._approach_skill.update(perception=perception, motion_backend=motion_backend)

        if st == PrimitiveStatus.RUNNING:
            if not snap.visible_now:
                grace = self._vision_grace.evaluate(visible_now=snap.visible_now, age_s=snap.age_s)
                if not grace.lost_long_enough:
                    return self.status

                print(
                    f"[ACQUIRE_OBJECT][VISION_LOSS] age_s={snap.age_s:.2f} "
                    f"> grace_s={grace.grace_s:.2f} -> RECOVER_LOST_TARGET"
                )

                self._safe_stop(self._approach_skill, motion_backend=motion_backend)
                self._approach_skill = None

                self.phase = "RECOVER_LOST_TARGET"
                self._recover_lost_target_behavior = None
                return self.status

            return self.status

        if st == PrimitiveStatus.FAILED:
            # If it failed while vision is lost long enough, treat as recovery entry.
            if not snap.visible_now:
                grace = self._vision_grace.evaluate(visible_now=snap.visible_now, age_s=snap.age_s)
                if grace.lost_long_enough:
                    print(
                        f"[ACQUIRE_OBJECT][APPROACH_FAILED_VISION] age_s={snap.age_s:.2f} "
                        f"> grace_s={grace.grace_s:.2f} -> RECOVER_LOST_TARGET"
                    )

                    self._safe_stop(self._approach_skill, motion_backend=motion_backend)
                    self._approach_skill = None

                    self.phase = "RECOVER_LOST_TARGET"
                    self._recover_lost_target_behavior = None
                    return self.status

            self.status = BehaviorStatus.FAILED
            return self.status

        # SUCCEEDED => ready to grab
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
    # Phase: RECOVER_LOST_TARGET
    # -------------------------

    def _recover_lost_target(self, perception, localisation, motion_backend):
        """
        Delegate lost-target recovery to RecoverLostTarget behavior.
        On success:
          - LOCKED_RECOVERED -> TRACK_AFTER_RECOVER (funnel through TrackObject)
          - NEW_TARGET_FOUND -> SELECT (choose+lock cleanly)
          - POSE_OBTAINED_NO_TARGET -> SELECT for now (future: pose-directed reposition)
        On failure:
          - GLOBAL_SEARCH (kept as your existing rung #3)
        """
        if self._recover_lost_target_behavior is None:
            last_bearing = 0.0
            last_distance = None
            if self.track is not None:
                if self.track.last_seen_bearing_deg is not None:
                    last_bearing = float(self.track.last_seen_bearing_deg)
                if self.track.last_seen_distance_mm is not None:
                    last_distance = float(self.track.last_seen_distance_mm)

            self._recover_lost_target_behavior = RecoverLostTarget()
            self._recover_lost_target_behavior.start(
                config=self.config,
                kind=self.kind,
                locked_target_id=self.locked_target_id,
                last_bearing_deg=last_bearing,
                last_distance_mm=last_distance,
            )

        st = self._recover_lost_target_behavior.update(
            perception=perception,
            localisation=localisation,
            motion_backend=motion_backend,
        )

        if st == BehaviorStatus.RUNNING:
            return self.status

        res = getattr(self._recover_lost_target_behavior, "result", None)
        outcome = getattr(res, "outcome", "FAILED")

        if st == BehaviorStatus.SUCCEEDED and outcome == "LOCKED_RECOVERED":
            self.locked_target_id = getattr(res, "found_target_id", self.locked_target_id)
            self.target = None
            self.phase = "TRACK_AFTER_RECOVER"
            self._recover_lost_target_behavior = None
            return self.status

        if st == BehaviorStatus.SUCCEEDED and outcome == "NEW_TARGET_FOUND":
            self._recover_lost_target_behavior = None
            self._enter_select(motion_backend=motion_backend, reason="recover_lost_target_new_target")
            return self.status

        if st == BehaviorStatus.SUCCEEDED and outcome == "POSE_OBTAINED_NO_TARGET":
            self._recover_lost_target_behavior = None
            self._enter_select(motion_backend=motion_backend, reason="recover_lost_target_pose_only")
            return self.status

        # FAILED
        self._recover_lost_target_behavior = None
        self.phase = "GLOBAL_SEARCH"
        self._global_search_behavior = None
        return self.status

    # -------------------------
    # Phase: TRACK_AFTER_RECOVER
    # -------------------------

    def _track_after_recover(self, motion_backend):
        """
        Hard invariant:
          Any recovery success must funnel back through TrackObject before ALIGN/APPROACH.
        We wait until the tracker has a fresh observation for our locked id, then proceed to ALIGN.
        """
        if self.locked_target_id is None:
            self._enter_select(motion_backend=motion_backend, reason="track_after_recover_no_lock")
            return self.status

        if self.track is None or (not self.track.visible_now) or self.track.last_obs is None:
            return self.status

        try:
            obs_id = int(self.track.last_obs.get("id"))
        except Exception:
            obs_id = None

        if obs_id != self.locked_target_id:
            self._enter_select(motion_backend=motion_backend, reason="track_after_recover_id_mismatch")
            return self.status

        self.target = self.track.last_obs
        self.phase = "ALIGN"
        self._align_skill = None
        return self.status

    # -------------------------
    # Phase: BACKOFF_SCAN (SELECT-stall ladder)
    # -------------------------

    def _backoff_scan(self, perception, motion_backend):
        if self._backoff_scan_skill is None:
            print("[ACQUIRE_OBJECT][BACKOFF_SCAN] start")
            self._backoff_scan_skill = BackoffScan(kind=self.kind, label="ACQUIRE_OBJECT][BACKOFF_SCAN")
            self._backoff_scan_skill.start(motion_backend=motion_backend)

        st = self._backoff_scan_skill.update(motion_backend=motion_backend, perception=perception)

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.SUCCEEDED:
            print("[ACQUIRE_OBJECT][BACKOFF_SCAN] succeeded -> SELECT")
            self._backoff_scan_skill = None
            self._enter_select(motion_backend=motion_backend, reason="backoff_succeeded")
            return self.status

        print("[ACQUIRE_OBJECT][BACKOFF_SCAN] failed -> GLOBAL_SEARCH")
        self._backoff_scan_skill = None
        self.phase = "GLOBAL_SEARCH"
        self._global_search_behavior = None
        return self.status

    # -------------------------
    # Phase: GLOBAL_SEARCH
    # -------------------------

    def _global_search(self, perception, motion_backend):
        """
        Delegate to GlobalSearchStub.
          - SUCCEEDED -> SELECT
          - FAILED -> Behavior FAILED
        """
        if self._global_search_behavior is None:
            self._global_search_behavior = GlobalSearchStub()
            self._global_search_behavior.start(
                config=self.config,
                kind=self.kind,
                exclude_ids=self.exclude_ids,
                motion_backend=motion_backend,
            )

        st = self._global_search_behavior.update(perception=perception, motion_backend=motion_backend)

        if st == BehaviorStatus.RUNNING:
            return self.status

        if st == BehaviorStatus.SUCCEEDED:
            print("[ACQUIRE_OBJECT][GLOBAL_SEARCH] succeeded -> SELECT")
            self._global_search_behavior = None
            self._enter_select(motion_backend=motion_backend, reason="global_search_succeeded")
            return self.status

        print("[ACQUIRE_OBJECT][GLOBAL_SEARCH] failed -> Behavior FAILED")
        self.status = BehaviorStatus.FAILED
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

    # -------------------------
    # Stop / util
    # -------------------------

    def stop(self, *, motion_backend=None, **_):
        self._safe_stop(self._select_skill, motion_backend=motion_backend)
        self._safe_stop(self._search_rotate_skill, motion_backend=motion_backend)
        self._safe_stop(self._align_skill, motion_backend=motion_backend)
        self._safe_stop(self._approach_skill, motion_backend=motion_backend)
        self._safe_stop(self._recover_lost_target_behavior, motion_backend=motion_backend)
        self._safe_stop(self._backoff_scan_skill, motion_backend=motion_backend)
        self._safe_stop(self._global_search_behavior, motion_backend=motion_backend)
        self._safe_stop(self._grasp_skill, motion_backend=motion_backend)
        self._safe_stop(self._verify_skill, motion_backend=motion_backend)

        self.status = BehaviorStatus.FAILED
        return self.status

    def _safe_stop(self, thing, *, motion_backend=None):
        """
        Stop helper that tolerates mixed stop() signatures and nested primitives.
        Never raises.
        """
        if thing is None:
            return

        try:
            sig = inspect.signature(thing.stop)
            if motion_backend is not None and "motion_backend" in sig.parameters:
                thing.stop(motion_backend=motion_backend)
            else:
                thing.stop()
            return
        except TypeError:
            pass
        except Exception:
            pass

        if motion_backend is not None:
            for attr in ("active_primitive", "_child"):
                child = getattr(thing, attr, None)
                if child is None:
                    continue
                try:
                    child.stop(motion_backend=motion_backend)
                except Exception:
                    pass

        try:
            thing.stop()
        except Exception:
            pass
