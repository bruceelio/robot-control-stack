# behaviors/approach_object.py

import time
import inspect

from behaviors.base import Behavior, BehaviorStatus
from navigation.height_model import HeightModel
from policies.vision_grace_period import VisionGracePeriod
from primitives.base import PrimitiveStatus

from skills.navigation.align_to_target import AlignToTarget
from skills.navigation.approach_target import ApproachTarget
from skills.navigation.approach_target_servo import ApproachTargetServo
from skills.perception.track_object import TrackObject

from log_trace import next_run, trace


class ApproachObject(Behavior):

    def __init__(
        self,
        *,
        pose_bearing_allowed: bool = False,
    ):
        super().__init__()

        self.config = None
        self.kind = None



        self.pose_bearing_allowed = bool(
            pose_bearing_allowed
        )

        self.phase = "SELECT"
        self.target = None
        self.servo_method = None

        # Navigation handoff geometry for PickupObject.
        self.final_distance_mm = None
        self.final_bearing_deg = None
        self.target_is_high = None

        # The concrete marker id we actually approached (if any)
        self.acquired_target_id = None

        # Failure handoff to the autonomous state machine.
        self.failure_reason = None
        self.lost_target_id = None
        self.lost_target_kind = None

        # ALIGN skill
        self._align_skill = None

        # APPROACH skill
        self._approach_skill = None

        # Active navigation stage during target approach.
        # 3 = localisation/path navigation   (future)
        # 2 = perception/servo navigation
        # 1 = dead reckoning
        self._navigation_stage = None

        self.height_model = HeightModel()
        self.locked_target_id = None

        # Locked-target tracking (single source of truth for visibility/loss)
        self._tracker: TrackObject | None = None
        self.track = None

        # Vision loss policy
        self._vision_grace: VisionGracePeriod | None = None

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

    def start(
            self,
            *,
            config,
            selected_target=None,
            selected_kind=None,
            selected_servo_method=None,
            **_,
    ):
        self.run_id = next_run()

        print("[ACQUIRE_OBJECT] start")
        self.config = config

        # SelectTarget has already made the decision.
        self.target = selected_target
        self.kind = selected_kind
        self.servo_method = selected_servo_method

        self.locked_target_id = None

        initial_phase = "START_APPROACH"


        self.final_distance_mm = None
        self.final_bearing_deg = None
        self.target_is_high = None

        trace(
            src="ACQ",
            evt="ACQ_START",
            phase=initial_phase,
            run=self.run_id,
            kind=self.kind,
            servo=(
                self.servo_method.value
                if self.servo_method is not None
                else "none"
            ),
        )

        # --- tracking & vision policy ---
        # Tracker is created after SELECT, once the actual target kind is known.
        self._tracker = None

        self._vision_grace = VisionGracePeriod(
            vision_grace_s=self.config.vision_grace_period_s
        )

        self.track = None

        self.phase = initial_phase
        self.acquired_target_id = None
        self.failure_reason = None
        self.lost_target_id = None
        self.lost_target_kind = None


        # Reset height model for each acquire run
        self.height_model = HeightModel()

        self._align_skill = None
        self._approach_skill = None
        self._navigation_stage = None

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

        if self.phase == "START_APPROACH":
            return self._start_selected_target()



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

        if self.phase == "ALIGN":
            return self._align(lvl2, motion_backend)

        if self.phase == "APPROACHING":
            return self._approach(lvl2, perception, motion_backend)

        return self.status

    # -------------------------
    # Helpers: phase transitions
    # -------------------------

    def _fail_target_lost(
            self,
            *,
            motion_backend=None,
            reason: str = "",
    ):

        self._safe_stop(
            self._approach_skill,
            motion_backend=motion_backend,
        )
        self._approach_skill = None

        self.failure_reason = "target_lost"
        self.lost_target_id = self.locked_target_id
        self.lost_target_kind = self.kind

        print(
            "[APPROACH_OBJECT][TARGET_LOST] "
            f"id={self.lost_target_id} "
            f"kind={self.lost_target_kind} "
            f"reason={reason}"
        )

        self.status = BehaviorStatus.FAILED
        return self.status

    def _start_selected_target(self):

        if (
            self.target is None
            or self.kind is None
            or self.servo_method is None
        ):
            print(
                "[APPROACH_OBJECT] "
                "missing selected target, kind, or servo method"
            )
            self.status = BehaviorStatus.FAILED
            return self.status

        try:
            self.locked_target_id = int(
                self.target.get("id")
            )
        except (TypeError, ValueError):
            self.locked_target_id = None

        self.height_model.reset()

        print(
            "[APPROACH_OBJECT] "
            f"target id={self.locked_target_id} "
            f"kind={self.kind} "
            f"dist={self.target['distance']:.0f} "
            f"bearing={self.target['bearing']:.1f}"
        )

        self._tracker = TrackObject(
            kind=self.kind
        )

        self._tracker.reset(
            locked_target_id=self.locked_target_id,
            kind=self.kind,
        )

        self.track = None

        # ------------------------------------------
        # Highest available navigation stage
        # ------------------------------------------

        if self.config.servoing_enabled:

            print(
                "[APPROACH_OBJECT][NAV] "
                "Stage 2 available -> SERVO APPROACH"
            )

            self._navigation_stage = 2

            trace(
                src="ACQ",
                evt="PHASE_ENTER",
                phase="APPROACHING",
                run=self.run_id,
                lock=self.locked_target_id or "none",
            )

            self.phase = "APPROACHING"
            self._approach_skill = None

        else:

            print(
                "[APPROACH_OBJECT][NAV] "
                "Stage 2 unavailable -> Stage 1 DEAD_RECKONING"
            )

            self._navigation_stage = 1

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

    def _align(self, lvl2, motion_backend):
        bearing = float(self.target["bearing"])
        self._navigation_stage = 1

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
        self._approach_skill.start(
            motion_backend=motion_backend,
            lvl2=lvl2,
            seed_target=self.target,
        )

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

    def _approach(self, lvl2, perception, motion_backend):

        if self._approach_skill is None:

            if self._navigation_stage == 2:
                print(
                    "[ACQUIRE_OBJECT][NAV] "
                    "starting Stage 2 ApproachTargetServo"
                )

                self._approach_skill = ApproachTargetServo(
                    config=self.config,
                    kind=self.kind,
                    height_model=self.height_model,
                    locked_target_id=self.locked_target_id,
                    servo_method=self.servo_method,
                    pose_bearing_allowed=self.pose_bearing_allowed,
                )

            else:
                # Stage 1 remains the default/fallback.
                self._navigation_stage = 1

                print(
                    "[ACQUIRE_OBJECT][NAV] "
                    "starting Stage 1 ApproachTarget"
                )

                self._approach_skill = ApproachTarget(
                    config=self.config,
                    kind=self.kind,
                    height_model=self.height_model,
                    locked_target_id=self.locked_target_id,
                )

            self._approach_skill.start(
                motion_backend=motion_backend,
                lvl2=lvl2,
                seed_target=self.target,
            )

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

                # _vision_settle_until is also the moment at which
                # the fresh-observation wait begins.
                if self._vision_settle_until is not None:
                    waited = max(0.0, now - self._vision_settle_until)
                else:
                    waited = 0.0

                print(
                    f"[VISION] waiting fresh obs "
                    f"age={age} waited={waited:.2f}s"
                )

                fresh_timeout_s = float(
                    getattr(
                        self.config,
                        "vision_loss_timeout_s",
                        0.5,
                    )
                )

                if waited >= fresh_timeout_s:
                    print(
                        "[VISION] no fresh post-align observation "
                        f"within {fresh_timeout_s:.2f}s "
                        "-> TARGET LOST"
                    )

                    self._require_fresh_obs_after_settle = False
                    self._vision_settle_until = None

                    return self._fail_target_lost(
                        motion_backend=motion_backend,
                        reason="no_fresh_post_align_observation",
                    )

                return self.status

            print(f"[VISION] fresh observation accepted age={snap.age_s:.3f}s")
            self.target = snap.last_obs
            self._require_fresh_obs_after_settle = False
            self._vision_settle_until = None

        snap = self.track

        st = self._approach_skill.update(
            perception=perception,
            motion_backend=motion_backend,
            lvl2=lvl2,
        )

        # --------------------------------------------------
        # Navigation-stage fallback
        # --------------------------------------------------
        #
        # Stage 2 depends on usable live perception.
        # If that becomes unavailable, stop continuous servoing
        # and fall back to the proven Stage 1 approach.
        #
        # Do not fall back for a height-classification safety failure.

        if (
                self._navigation_stage == 2
                and st == PrimitiveStatus.FAILED
        ):
            failure_reason = getattr(
                self._approach_skill,
                "failure_reason",
                None,
            )

            if (
                    failure_reason
                    == ApproachTargetServo.FAILURE_PERCEPTION
            ):
                print(
                    "[ACQUIRE_OBJECT][NAV] "
                    "Stage 2 perception unavailable "
                    "-> fallback to Stage 1"
                )

                # Important: stop continuous velocity output before
                # handing control to the dead-reckoning approach.
                self._safe_stop(
                    self._approach_skill,
                    motion_backend=motion_backend,
                )

                self._approach_skill = None
                self._navigation_stage = 1

                # There was no discrete ALIGN rotation here, so there
                # is no post-ALIGN camera-settle requirement.
                self._vision_settle_until = None
                self._require_fresh_obs_after_settle = False

                return self.status

            if (
                    failure_reason
                    == ApproachTargetServo.FAILURE_HEIGHT
            ):
                print(
                    "[ACQUIRE_OBJECT][NAV] "
                    "Stage 2 height unresolved at commit "
                    "-> FAILED"
                )

                self._safe_stop(
                    self._approach_skill,
                    motion_backend=motion_backend,
                )

                self.status = BehaviorStatus.FAILED
                return self.status

        # While Stage 2 is healthy, its controller owns perception
        # freshness. Do not also run the Stage 1 vision-loss policy.
        if (
                self._navigation_stage == 2
                and st == PrimitiveStatus.RUNNING
        ):
            return self.status

        if st == PrimitiveStatus.RUNNING:
            if not snap.visible_now:

                grace = self._vision_grace.evaluate(
                    visible_now=snap.visible_now,
                    age_s=snap.age_s,
                )

                if not grace.lost_long_enough:
                    return self.status

                return self._fail_target_lost(
                    motion_backend=motion_backend,
                    reason="vision_grace_expired",
                )

            return self.status

        if st == PrimitiveStatus.FAILED:

            if not snap.visible_now:

                grace = self._vision_grace.evaluate(
                    visible_now=snap.visible_now,
                    age_s=snap.age_s,
                )

                if grace.lost_long_enough:
                    return self._fail_target_lost(
                        motion_backend=motion_backend,
                        reason="approach_failed_target_not_visible",
                    )

            self.failure_reason = "approach_failed"
            self.status = BehaviorStatus.FAILED
            return self.status

        # SUCCEEDED => reached pickup commit distance

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
            print(
                f"[ACQUIRE_OBJECT] "
                f"approached_target_id={tid}"
            )

        self.final_distance_mm = self._approach_skill.final_distance_mm
        self.final_bearing_deg = self._approach_skill.final_bearing_deg
        self.target_is_high = self._approach_skill.target_is_high

        if (
                self.final_distance_mm is None
                or self.final_bearing_deg is None
        ):
            print(
                "[ACQUIRE_OBJECT] ApproachTarget SUCCEEDED "
                "without pickup handoff geometry"
            )
            self.status = BehaviorStatus.FAILED
            return self.status

        print(
            "[ACQUIRE_OBJECT] approach complete "
            f"id={self.acquired_target_id} "
            f"distance={self.final_distance_mm:.0f}mm "
            f"bearing={self.final_bearing_deg:.1f}deg "
            f"high={self.target_is_high}"
        )

        self.status = BehaviorStatus.SUCCEEDED
        return self.status

    def stop(self, *, motion_backend=None, **_):
        self._safe_stop(self._align_skill, motion_backend=motion_backend)
        self._safe_stop(self._approach_skill, motion_backend=motion_backend)
