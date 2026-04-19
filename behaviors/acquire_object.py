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

from skills.perception.select_target import SelectTarget
from skills.perception.track_object import TrackObject

from log_trace import next_run, trace

class AcquireObject(Behavior):
    """
    Pickup-only pipeline:

      SELECT -> ALIGN -> APPROACHING -> GRABBING -> SUCCEEDED

    Adds:
      - Active scanning while SELECT is running (SearchRotate)
      - SELECT stall watchdog that escalates to GLOBAL_SEARCH instead of hanging forever
    """

    def __init__(self):
        super().__init__()

        self.config = None
        self.kind = None

        self.phase = "SELECT"
        self.target = None

        # The concrete marker id we actually approached (if any)
        self.acquired_target_id = None

        # SELECT skill
        self._select_skill = None

        # ALIGN skill
        self._align_skill = None

        # APPROACH skill
        self._approach_skill = None

        # Grab state
        self._grab_step = None  # "GRASP" -> "VERIFY"
        self._grasp_skill = None
        self._verify_skill = None

        self.height_model = HeightModel()
        self.exclude_ids: set[int] = set()

        self.locked_target_id = None

        # Locked-target tracking (single source of truth for visibility/loss)
        self._tracker: TrackObject | None = None
        self.track = None

        # Vision loss policy
        self._vision_grace: VisionGracePeriod | None = None

        self._global_search_behavior: GlobalSearchStub | None = None

        # Recover step (delegates full ladder)
        self._recover_behavior: RecoverLostTarget | None = None
        self._recover_result = None  # RecoverLostTarget.Result
        self._recover_seed_target = None
        self._recover_started_s: float | None = None
        self._recover_timeout_s: float | None = None
        self._recover_lost_target_id: int | None = None

        # Track-after-recover (locks regained -> TRACK to establish stable obs)
        self._track_after_recover_skill: TrackObject | None = None

        # --- SELECT stall watchdog ---
        self._select_started_s = None
        self._select_stall_count = 0

        # Vision Settle
        self._vision_settle_until = None
        self._require_fresh_obs_after_settle = False
        self._max_fresh_age_s = None

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
        self.run_id = next_run()

        print("[ACQUIRE_OBJECT] start")
        self.config = config
        self.kind = kind or config.default_target_kind
        self.locked_target_id = None

        # exclusions must be set before tracing
        self.exclude_ids = set(exclude_ids) if exclude_ids else set()

        trace(
            src="ACQ",
            evt="ACQ_START",
            phase="SELECT",
            run=self.run_id,
            kind=self.kind,
            exclude=len(self.exclude_ids),
        )

        # --- tracking & vision policy ---
        self._tracker = TrackObject(kind=self.kind)
        self._vision_grace = VisionGracePeriod(
            vision_grace_s=self.config.vision_grace_period_s
        )

        # tracker reset for this run
        self._tracker.reset(locked_target_id=None, kind=self.kind)
        self.track = None

        self.phase = "SELECT"
        self.target = None
        self.acquired_target_id = None

        # Reset height model for each acquire run
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
        self._search_rotate_skill = None

        # reset grab state
        self._grab_step = None
        self._grasp_skill = None
        self._verify_skill = None

        self._global_search_behavior = None

        # recover reset
        self._recover_behavior = None
        self._recover_result = None
        self._recover_seed_target = None
        self._recover_started_s = None
        self._recover_timeout_s = None
        self._recover_lost_target_id = None

        self._track_after_recover_skill = None

        # --- SELECT stall watchdog reset (IMPORTANT: must be before return) ---
        self._select_started_s = time.time()
        self._select_stall_count = 0

        # --- Vision settle / fresh observation gate reset ---
        self._vision_settle_until = None
        self._require_fresh_obs_after_settle = False
        self._max_fresh_age_s = float(
            getattr(self.config, "CAMERA_FRESH_OBS_MAX_AGE_S", 0.12)
        )

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
            self._vision_grace = VisionGracePeriod(
                vision_grace_s=self.config.vision_grace_period_s
            )

        # Update tracker once per tick so all phases read the same truth.
        self.track = self._tracker.update(
            perception_objects=getattr(perception, "objects", perception),
            now_s=time.time(),
            locked_target_id=self.locked_target_id,
            kind=self.kind,
        )

        snap = self.track
        trace(
            src="TRACK",
            evt="TRACK_UPDATE",
            phase=self.phase,
            run=self.run_id,
            kind=self.kind,
            lock=snap.locked_id or "none",
            visible=int(snap.visible_now),
            age=snap.age_s,
            dist=snap.last_seen_distance_mm,
            bear=snap.last_seen_bearing_deg,
            seen=snap.seen_count,
            lost=snap.lost_count,
        )

        if self.phase == "SELECT":
            return self._select(perception, motion_backend)

        if self.phase == "ALIGN":
            return self._align(motion_backend)

        if self.phase == "APPROACHING":
            return self._approach(perception, motion_backend)

        if self.phase == "RECOVER_LOST_TARGET":
            return self._recover_lost_target(perception, localisation, motion_backend)

        if self.phase == "TRACK_AFTER_RECOVER":
            return self._track_after_recover(perception, motion_backend)

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

        # stop any scan / select primitives
        self._safe_stop(self._select_skill, motion_backend=motion_backend)
        self._select_skill = None

        # clear targeting
        self.target = None
        self.locked_target_id = None

        # reset tracker unlocked
        if self._tracker is not None:
            self._tracker.reset(locked_target_id=None, kind=self.kind)
        self.track = None

        # reset watchdog
        self._select_started_s = time.time()
        self._select_stall_count = 0

        # enter phase
        self.phase = "SELECT"
        trace(
            src="ACQ",
            evt="PHASE_ENTER",
            phase="SELECT",
            run=self.run_id,
            lock=self.locked_target_id or "none",
        )

    def _enter_global_search(self, *, motion_backend=None, reason: str = ""):
        trace(
            src="ACQ",
            evt="PHASE_ENTER",
            phase="GLOBAL_SEARCH",
            run=self.run_id,
            lock=self.locked_target_id or "none",
        )

        if reason:
            print(f"[ACQUIRE_OBJECT] -> GLOBAL_SEARCH ({reason})")

        self._safe_stop(self._select_skill, motion_backend=motion_backend)
        self._select_skill = None

        self.target = None
        self.locked_target_id = None

        if self._tracker is not None:
            self._tracker.reset(locked_target_id=None, kind=self.kind)
        self.track = None

        self.phase = "GLOBAL_SEARCH"
        self._global_search_behavior = None

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

            # entering SELECT fresh -> reset watchdog
            self._select_started_s = time.time()
            self._select_stall_count = 0

        st = self._select_skill.update(perception=perception)

        if st == PrimitiveStatus.RUNNING:
            now = time.time()

            # Start a rotate-scan if not already running


            # Hard timeout since SELECT started -> escalate
            select_timeout_s = float(getattr(self.config, "select_timeout_s", 6.0))
            max_stalls = int(getattr(self.config, "select_max_stalls_before_escalate", 2))

            if self._select_started_s is None:
                self._select_started_s = now

            elapsed = now - self._select_started_s

            if elapsed > select_timeout_s or self._select_stall_count >= max_stalls:
                print(
                    f"[ACQUIRE_OBJECT][SELECT] STALLED elapsed={elapsed:.2f}s "
                    f"timeout={select_timeout_s:.2f}s stalls={self._select_stall_count}/{max_stalls} "
                    f"-> GLOBAL_SEARCH"
                )
                self._enter_global_search(
                    motion_backend=motion_backend,
                    reason="select_stalled",
                )
                return self.status

            return self.status

        if st == PrimitiveStatus.FAILED:
            print("[ACQUIRE_OBJECT][SELECT] SelectTarget FAILED -> GLOBAL_SEARCH")
            self._enter_global_search(
                motion_backend=motion_backend,
                reason="select_failed",
            )
            return self.status

        # SUCCEEDED
        self.target = self._select_skill.selected_target
        if self.target is None:
            # Treat as stall-ish; restart SELECT cleanly
            print("[ACQUIRE_OBJECT][SELECT] SUCCEEDED but selected_target is None -> SELECT")
            self._enter_select(motion_backend=motion_backend, reason="no_selected_target")
            return self.status

        self.height_model.reset()

        # --- LOCK ---
        try:
            self.locked_target_id = int(self.target.get("id"))
        except Exception:
            self.locked_target_id = None

        print(f"[ACQUIRE_OBJECT][LOCK] locked_target_id={self.locked_target_id}")

        # Seed tracker with the lock for consistent visibility/loss decisions downstream
        if self._tracker is not None:
            self._tracker.reset(locked_target_id=self.locked_target_id, kind=self.kind)

        print(
            f"[ACQUIRE_OBJECT][SELECT] target found "
            f"id={self.target.get('id', 'REL')} "
            f"dist={self.target['distance']:.0f} "
            f"bearing={self.target['bearing']:.1f}"
        )

        # Stop scanning now that we have a target

        trace(
            src="ACQ",
            evt="PHASE_ENTER",
            phase="ALIGN",
            run=self.run_id,
            lock=self.locked_target_id or "none",
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
        trace(
            src="ACQ",
            evt="PHASE_ENTER",
            phase="APPROACHING",
            run=self.run_id,
            lock=self.locked_target_id or "none",
        )

        self.phase = "APPROACHING"
        self._approach_skill = ApproachTarget(
            config=self.config,
            kind=self.kind,
            height_model=self.height_model,
            locked_target_id=self.locked_target_id,
        )
        self._approach_skill.start(motion_backend=motion_backend, seed_target=self.target)

        # NEW: force a post-rotate camera settle gate
        self._vision_settle_until = time.time() + float(self.config.camera_settle_time)
        self._require_fresh_obs_after_settle = True

        # Optional but recommended: drop stale tracker state so we demand a fresh frame
        if self._tracker is not None:
            self._tracker.reset(locked_target_id=self.locked_target_id, kind=self.kind)
        self.track = None

        print(f"[VISION] settle start {self.config.camera_settle_time:.2f}s after ALIGN")

        return self.status

    def _safe_stop(self, thing, *, motion_backend=None):
        """
        Stop helper that tolerates mixed stop() signatures and nested primitives.

        Handles:
          - stop()
          - stop(motion_backend=...)
          - wrappers that hold an active primitive needing motion_backend (e.g. ApproachTarget.active_primitive)
        Never raises.
        """
        if thing is None:
            return

        # 1) If stop() accepts motion_backend, prefer that when available
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

        # 2) Best-effort: stop common nested primitives that require motion_backend
        if motion_backend is not None:
            for attr in ("active_primitive",):
                child = getattr(thing, attr, None)
                if child is None:
                    continue
                try:
                    child.stop(motion_backend=motion_backend)
                except Exception:
                    pass

        # 3) Last attempt: call stop() again (no args)
        try:
            thing.stop()
        except Exception:
            pass

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

        # NEW: camera settle gate
        now = time.time()
        if self._require_fresh_obs_after_settle:
            if self._vision_settle_until is not None and now < self._vision_settle_until:
                remaining = self._vision_settle_until - now
                print(f"[VISION] waiting settle {remaining:.2f}s")
                return self.status

            snap = self.track
            fresh_enough = (
                    snap is not None
                    and snap.visible_now
                    and snap.age_s is not None
                    and snap.age_s <= self._max_fresh_age_s
                    and snap.last_obs is not None
            )

            if not fresh_enough:
                age = None if snap is None else snap.age_s
                print(f"[VISION] waiting fresh obs age={age}")
                return self.status

            print(f"[VISION] fresh observation accepted age={snap.age_s:.3f}s")
            self.target = snap.last_obs
            self._require_fresh_obs_after_settle = False
            self._vision_settle_until = None

        snap = self.track

        st = self._approach_skill.update(perception=perception, motion_backend=motion_backend)

        if st == PrimitiveStatus.RUNNING:
            if not snap.visible_now:
                # 1) Small debounce (don’t react to single-frame dropouts)
                grace = self._vision_grace.evaluate(
                    visible_now=snap.visible_now,
                    age_s=snap.age_s,
                )
                if not grace.lost_long_enough:
                    return self.status

                # 2) After grace, allow the approach/reacquire ladder to run
                # until we exceed the reacquire budget.
                # We intentionally do NOT allow ApproachTarget's internal reacquire.
                # Once grace says "lost long enough", escalate to RecoverLostTarget immediately.

                print(
                    f"[ACQUIRE_OBJECT][VISION_LOSS] age_s={snap.age_s:.2f} "
                    f"> grace_s={grace.grace_s:.2f} "
                    f"-> RECOVER_LOST_TARGET (direct escalate)"
                )

                self._safe_stop(self._approach_skill, motion_backend=motion_backend)
                self._approach_skill = None

                self._recover_seed_target = self.target
                self._recover_lost_target_id = self.locked_target_id
                self._recover_started_s = time.time()
                self._recover_timeout_s = float(getattr(self.config, "recover_total_timeout_s", 8.0))

                trace(
                    src="ACQ",
                    evt="PHASE_ENTER",
                    phase="RECOVER_LOST_TARGET",
                    run=self.run_id,
                    lock=self.locked_target_id or "none",
                )

                self.phase = "RECOVER_LOST_TARGET"
                self._recover_behavior = None
                self._recover_result = None
                return self.status

            return self.status

        if st == PrimitiveStatus.FAILED:
            if not snap.visible_now:
                grace = self._vision_grace.evaluate(
                    visible_now=snap.visible_now,
                    age_s=snap.age_s,
                )
                if grace.lost_long_enough:
                    reacquire_budget_s = float(
                        getattr(self.config, "reacquire_target_vision_loss", self.config.vision_loss_timeout_s)
                    )

                    if snap.age_s < reacquire_budget_s:
                        # Failed for some other reason while target is not visible,
                        # but loss age hasn't exceeded the budget: don't escalate yet.
                        return self.status

                    print(
                        f"[ACQUIRE_OBJECT][APPROACH_FAILED_VISION] age_s={snap.age_s:.2f} "
                        f"> grace_s={grace.grace_s:.2f} and > reacquire_budget_s={reacquire_budget_s:.2f} "
                        f"-> RECOVER_LOST_TARGET"
                    )

                    self._safe_stop(self._approach_skill, motion_backend=motion_backend)
                    self._approach_skill = None

                    self._recover_seed_target = self.target
                    self._recover_lost_target_id = self.locked_target_id
                    self._recover_started_s = time.time()
                    self._recover_timeout_s = float(getattr(self.config, "recover_total_timeout_s", 8.0))

                    trace(
                        src="ACQ",
                        evt="PHASE_ENTER",
                        phase="RECOVER_LOST_TARGET",
                        run=self.run_id,
                        lock=self.locked_target_id or "none",
                    )

                    self.phase = "RECOVER_LOST_TARGET"
                    self._recover_behavior = None
                    self._recover_result = None
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
        trace(
            src="ACQ",
            evt="PHASE_ENTER",
            phase="GRABBING",
            run=self.run_id,
            lock=self.locked_target_id or "none",
        )

        self.phase = "GRABBING"
        return self.status

    # -------------------------
    # Phase: RECOVER_LOST_TARGET
    # -------------------------

    def _recover_lost_target(self, perception, localisation, motion_backend):


        if self._recover_started_s is not None and self._recover_timeout_s is not None:
            if (time.time() - self._recover_started_s) > self._recover_timeout_s:
                print("[ACQUIRE_OBJECT][RECOVER_LOST_TARGET] overall timeout -> GLOBAL_SEARCH")
                self._recover_behavior = None
                self._recover_result = None
                self._enter_global_search(motion_backend=motion_backend, reason="recover_timeout")
                return self.status

        if self._recover_behavior is None:
            self._recover_behavior = RecoverLostTarget()
            self._recover_behavior.run_id = self.run_id
            last_bearing = 0.0
            last_distance = None
            if self.track is not None:
                if self.track.last_seen_bearing_deg is not None:
                    last_bearing = float(self.track.last_seen_bearing_deg)
                if self.track.last_seen_distance_mm is not None:
                    last_distance = float(self.track.last_seen_distance_mm)

            self._recover_behavior.start(
                config=self.config,
                kind=self.kind,
                locked_target_id=self.locked_target_id,
                last_bearing_deg=last_bearing,
                last_distance_mm=last_distance,
                motion_backend=motion_backend,  # harmless; RecoverLostTarget accepts **_
            )

        st = self._recover_behavior.update(
            perception=perception,
            localisation=localisation,
            motion_backend=motion_backend,
        )

        if st == BehaviorStatus.RUNNING:
            return self.status

        if st == BehaviorStatus.SUCCEEDED:
            res = getattr(self._recover_behavior, "result", None)
            self._recover_result = res
            print(f"[ACQUIRE_OBJECT][RECOVER_LOST_TARGET] succeeded result={res}")

            if getattr(res, "outcome", None) == "LOCKED_RECOVERED":
                trace(
                    src="ACQ",
                    evt="PHASE_ENTER",
                    phase="TRACK_AFTER_RECOVER",
                    run=self.run_id,
                    lock=self.locked_target_id or "none",
                )
                self.phase = "TRACK_AFTER_RECOVER"
                self._track_after_recover_skill = None
                return self.status

            self._recover_behavior = None
            self._recover_result = None
            self._enter_select(motion_backend=motion_backend, reason="recover_search_recovered")
            return self.status

        print("[ACQUIRE_OBJECT][RECOVER_LOST_TARGET] failed -> GLOBAL_SEARCH")
        self._recover_behavior = None
        self._recover_result = None
        self._enter_global_search(motion_backend=motion_backend, reason="recover_timeout")
        return self.status

    # -------------------------
    # Phase: TRACK_AFTER_RECOVER
    # -------------------------

    def _track_after_recover(self, perception, motion_backend):


        if self.locked_target_id is None:
            self._enter_select(motion_backend=motion_backend, reason="track_after_recover_no_lock")
            return self.status

        if self._track_after_recover_skill is None:
            self._track_after_recover_skill = TrackObject(kind=self.kind)
            self._track_after_recover_skill.reset(
                locked_target_id=self.locked_target_id,
                kind=self.kind,
            )

        snap = self._track_after_recover_skill.update(
            perception_objects=getattr(perception, "objects", perception),
            now_s=time.time(),
            locked_target_id=self.locked_target_id,
            kind=self.kind,
        )

        if snap.visible_now and snap.last_obs is not None:
            self.target = snap.last_obs
            print("[ACQUIRE_OBJECT][TRACK_AFTER_RECOVER] got fresh obs -> SELECT")
            self._track_after_recover_skill = None
            self._enter_select(motion_backend=motion_backend, reason="track_after_recover_ready")
            return self.status

        grace = self._vision_grace.evaluate(visible_now=snap.visible_now, age_s=snap.age_s)
        if grace.lost_long_enough:
            print("[ACQUIRE_OBJECT][TRACK_AFTER_RECOVER] lost again -> GLOBAL_SEARCH")
            self._track_after_recover_skill = None
            self._enter_global_search(motion_backend=motion_backend, reason="recover_timeout")
            return self.status

        return self.status

    # -------------------------
    # Phase: GLOBAL_SEARCH
    # -------------------------

    def _global_search(self, perception, motion_backend):
        """
        Delegate to GlobalSearchStub:
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

    def stop(self, *, motion_backend=None, **_):
        self._safe_stop(self._select_skill, motion_backend=motion_backend)
        self._safe_stop(self._align_skill, motion_backend=motion_backend)
        self._safe_stop(self._approach_skill, motion_backend=motion_backend)
        self._safe_stop(self._recover_behavior, motion_backend=motion_backend)
        self._safe_stop(self._track_after_recover_skill, motion_backend=motion_backend)
        self._safe_stop(self._global_search_behavior, motion_backend=motion_backend)
        self._safe_stop(self._grasp_skill, motion_backend=motion_backend)
        self._safe_stop(self._verify_skill, motion_backend=motion_backend)
