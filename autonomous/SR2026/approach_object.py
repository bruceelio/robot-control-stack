# autonomous/SR2026/approach_object.py

import inspect
import math
import time

from autonomous.SR2026.base import Behavior, BehaviorStatus
from perception import get_visible_targets
from perception.height_model import HeightModel
from perception.providers.local_obstacle_field import (
    local_obstacle_field_from_objects,
)
from perception.robot_geometry import (
    relative_target_from_base_link,
)
from policies.vision_grace_period import VisionGracePeriod
from primitives.base import PrimitiveStatus

from skills.navigation.align_to_target import AlignToTarget
from skills.navigation.approach_target import ApproachTarget
from skills.navigation.approach_target_servo import ApproachTargetServo
from skills.navigation.local_avoidance import (
    LocalAvoidance,
    LocalObstacleObservation,
)
from skills.perception.acquire_wall_geometry import AcquireWallGeometry
from skills.perception.track_object import TrackObject
from skills.perception.reacquire_target_bearing_reverse import (
    ReacquireTargetBearingReverse,
)

from log_trace import next_run, trace

SR2026_OBJECT_RADIUS_MM = 65.0

class ApproachObject(Behavior):

    def __init__(
        self,
        *,
        pose_bearing_allowed: bool = False,
    ):
        super().__init__()

        self.config = None
        self.calibration = None
        self.kind = None
        self.elevation = None

        self.approach_feasibility = None
        self.wall_geometry_status = None

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

        # Wall active-perception skill
        self._wall_acquisition_skill = None
        self._wall_acquisition_attempted = False
        self.wall_range_rays = ()

        # APPROACH skill
        self._approach_skill = None

        # Stage-2 local-avoidance supervisor.
        self._local_avoidance = None
        self._local_avoidance_active = False
        self._local_avoidance_unavailable_logged = False

        # Post-avoidance target recovery.
        self._post_avoidance_reacquire = None

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
            calibration=None,
            selected_target=None,
            selected_kind=None,
            selected_elevation=None,
            selected_servo_method=None,
            selected_approach_feasibility=None,
            **_,
    ):
        self.run_id = next_run()

        print("[ACQUIRE_OBJECT] start")
        self.config = config
        self.calibration = calibration

        # SelectTarget has already made the decision.
        self.target = selected_target
        self.kind = selected_kind
        self.elevation = selected_elevation
        self.servo_method = selected_servo_method

        self.approach_feasibility = (
            selected_approach_feasibility
        )

        self.wall_geometry_status = getattr(
            self.approach_feasibility,
            "wall_geometry_status",
            None,
        )

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
            elevation=self.elevation,
            servo=(
                self.servo_method.value
                if self.servo_method is not None
                else "none"
            ),
            wall_state=(
                self.wall_geometry_status.state.value
                if self.wall_geometry_status is not None
                else "none"
            ),
            wall_action=(
                self.wall_geometry_status.action.value
                if self.wall_geometry_status is not None
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

        self._wall_acquisition_skill = None
        self._wall_acquisition_attempted = False
        self.wall_range_rays = ()

        self._approach_skill = None

        self._local_avoidance = None
        self._local_avoidance_active = False
        self._local_avoidance_unavailable_logged = False

        # Post-avoidance target recovery.
        self._post_avoidance_reacquire = None

        self._navigation_stage = None

        # --- Vision settle / fresh observation gate reset ---
        self._vision_settle_until = None
        self._require_fresh_obs_after_settle = False
        self._max_fresh_age_s = float(
            getattr(self.config, "CAMERA_FRESH_OBS_MAX_AGE_S", 0.12)
        )

        self.status = BehaviorStatus.RUNNING
        return self.status

    def update(
        self,
        *,
        lvl2,
        perception,
        localisation,
        motion_backend,
        io=None,
        **_,
    ):
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

        if self.phase == "ACQUIRE_WALL_GEOMETRY":
            return self._acquire_wall_geometry(
                perception=perception,
                motion_backend=motion_backend,
            )

        if self.phase == "ALIGN":
            return self._align(lvl2, motion_backend)

        if self.phase == "APPROACHING":
            return self._approach(
                lvl2,
                perception,
                motion_backend,
                io=io,
                localisation=localisation,
            )

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

        self._safe_stop(
            self._post_avoidance_reacquire,
            motion_backend=motion_backend,
        )
        self._post_avoidance_reacquire = None

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
        # HIGH wall-geometry acquisition
        # ------------------------------------------

        if (
                self.elevation == "high"
                and self.wall_geometry_status is not None
                and self.wall_geometry_status.acquisition_required
                and not self._wall_acquisition_attempted
        ):
            print(
                "[APPROACH_OBJECT][WALL] "
                "HIGH target requires wall acquisition "
                f"action="
                f"{self.wall_geometry_status.action.value}"
            )

            trace(
                src="ACQ",
                evt="PHASE_ENTER",
                phase="ACQUIRE_WALL_GEOMETRY",
                run=self.run_id,
                lock=(
                        self.locked_target_id
                        or "none"
                ),
            )

            self.phase = "ACQUIRE_WALL_GEOMETRY"
            self._wall_acquisition_skill = None

            return self.status

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
    # Phase: ACQUIRE WALL GEOMETRY
    # -------------------------

    def _acquire_wall_geometry(
        self,
        *,
        perception,
        motion_backend,
    ):
        if self._wall_acquisition_skill is None:

            self._wall_acquisition_skill = (
                AcquireWallGeometry(
                    config=self.config,
                    wall_status=(
                        self.wall_geometry_status
                    ),
                )
            )

            st = (
                self._wall_acquisition_skill.start(
                    perception=perception,
                    motion_backend=motion_backend,
                )
            )

            if st == PrimitiveStatus.FAILED:
                print(
                    "[APPROACH_OBJECT][WALL] "
                    "acquisition could not start "
                    "-> visual fallback"
                )

                self._wall_acquisition_attempted = True
                self._wall_acquisition_skill = None
                self.phase = "START_APPROACH"

                return self.status

        st = self._wall_acquisition_skill.update(
            perception=perception,
            motion_backend=motion_backend,
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            print(
                "[APPROACH_OBJECT][WALL] "
                "acquisition failed "
                f"reason="
                f"{self._wall_acquisition_skill.failure_reason} "
                "-> visual fallback"
            )

            self._wall_acquisition_attempted = True
            self._wall_acquisition_skill = None
            self.phase = "START_APPROACH"

            return self.status

        self.wall_range_rays = tuple(
            self._wall_acquisition_skill.range_rays
        )

        print(
            "[APPROACH_OBJECT][WALL] "
            f"acquired {len(self.wall_range_rays)} "
            "range rays"
        )

        # Wall association / extraction is deliberately the
        # next layer. Do not blindly interpret these observations
        # as one wall here.
        #
        # Until that layer is connected, continue through the
        # proven HIGH visual fallback after acquisition.
        self._wall_acquisition_attempted = True
        self._wall_acquisition_skill = None
        self.phase = "START_APPROACH"

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
    # Stage-2 local avoidance
    # -------------------------

    def _ensure_local_avoidance(self):
        if self._local_avoidance is not None:
            return self._local_avoidance

        if self.calibration is None:
            if not self._local_avoidance_unavailable_logged:
                print(
                    "[APPROACH_OBJECT][LOCAL_AVOIDANCE] "
                    "calibration unavailable -> disabled"
                )
                self._local_avoidance_unavailable_logged = True

            return None

        try:
            camera_fov_deg = float(
                self.calibration
                .cameras["front"]
                .meta
                .fov_deg
            )
        except Exception:
            if not self._local_avoidance_unavailable_logged:
                print(
                    "[APPROACH_OBJECT][LOCAL_AVOIDANCE] "
                    "front-camera FOV unavailable -> disabled"
                )
                self._local_avoidance_unavailable_logged = True

            return None

        if camera_fov_deg <= 0.0:
            return None

        self._local_avoidance = LocalAvoidance(
            config=self.config,
            field_fov_rad=math.radians(
                camera_fov_deg
            ),
        )

        return self._local_avoidance

    def _visible_avoidance_objects(
        self,
        perception,
    ):
        objects = []

        for kind in (
            "acidic",
            "basic",
        ):
            objects.extend(
                get_visible_targets(
                    perception,
                    kind,
                    max_age_s=float(
                        self.config.visible_max_age_s
                    ),
                )
            )

        return objects

    def _avoidance_goal(self):
        snap = self.track

        if (
            snap is None
            or not snap.visible_now
            or snap.last_obs is None
        ):
            return None, None

        target = relative_target_from_base_link(
            observation=snap.last_obs,
            config=self.config,
        )

        return (
            float(target.distance_m) * 1000.0,
            float(target.bearing_rad),
        )

    def _avoidance_observations(
        self,
        *,
        visible_objects,
    ):
        observations = []

        for obj in visible_objects:
            try:
                object_id = int(
                    obj["id"]
                )

                obstacle = (
                    relative_target_from_base_link(
                        observation=obj,
                        config=self.config,
                    )
                )

                distance_mm = (
                    float(
                        obstacle.distance_m
                    )
                    * 1000.0
                )

                bearing_rad = float(
                    obstacle.bearing_rad
                )

            except (
                KeyError,
                TypeError,
                ValueError,
            ):
                continue

            if (
                object_id
                == self.locked_target_id
            ):
                continue

            if distance_mm <= 0.0:
                continue

            observations.append(
                LocalObstacleObservation(
                    obstacle_id=object_id,
                    distance_mm=distance_mm,
                    bearing_rad=bearing_rad,
                    radius_mm=(
                        SR2026_OBJECT_RADIUS_MM
                    ),
                )
            )

        return tuple(
            observations
        )

    def _update_local_avoidance(
        self,
        *,
        avoidance,
        visible_objects,
        obstacle_observations,
        goal_distance_mm,
        goal_bearing_rad,
        now_s,
    ):
        obstacle_range_limit_mm = (
            goal_distance_mm
            if goal_distance_mm is not None
            else avoidance.goal_distance_mm
        )

        exclude_ids = (
            ()
            if self.locked_target_id is None
            else (
                self.locked_target_id,
            )
        )

        obstacle_field = (
            local_obstacle_field_from_objects(
                config=self.config,
                visible_objects=visible_objects,
                field_fov_rad=(
                    avoidance.field_fov_rad
                ),
                scan_sectors=(
                    avoidance.scan_sectors
                ),
                clear_range_mm=(
                    avoidance.lookahead_distance_mm
                    * 1.5
                ),
                timestamp_s=now_s,
                exclude_ids=exclude_ids,
                default_obstacle_radius_mm=(
                    SR2026_OBJECT_RADIUS_MM
                ),
                max_obstacle_distance_mm=(
                    obstacle_range_limit_mm
                ),
            )
        )

        return avoidance.update(
            now_s=now_s,
            goal_distance_mm=(
                goal_distance_mm
            ),
            goal_bearing_rad=(
                goal_bearing_rad
            ),
            obstacle_observations=(
                obstacle_observations
            ),
            obstacle_field=(
                obstacle_field
            ),
        )

    def _post_avoidance_reverse_limit_mm(self) -> float:
        if (
            self.height_model.is_committed()
            and self.height_model.is_high()
        ):
            return float(
                self.config.final_commit_distance_high_mm
            )

        # If avoidance caused loss before HeightModel committed,
        # preserve the selected HIGH geometry.
        if (
            not self.height_model.is_committed()
            and self.elevation == "high"
        ):
            return float(
                self.config.final_commit_distance_high_mm
            )

        return float(
            self.config.final_commit_distance_mm
        )

    def _handoff_after_local_avoidance(
        self,
        *,
        avoidance,
        lvl2,
        motion_backend,
        io,
    ):
        self._local_avoidance_active = False

        # Always restart the nominal servo fresh after avoidance.
        self._approach_skill = None

        snap = self.track

        target_fresh = (
                snap is not None
                and snap.visible_now
                and snap.last_obs is not None
                and snap.age_s is not None
                and snap.age_s <= self._max_fresh_age_s
        )

        if target_fresh:
            print(
                "[APPROACH_OBJECT][LOCAL_AVOIDANCE] "
                "fresh target at handoff "
                "-> ApproachTargetServo"
            )

            return self.status

        # LocalAvoidance has propagated the local target geometry
        # while the target was hidden. stop() does not discard it,
        # so this is our best Stage-2 estimate of target direction.
        expected_bearing_rad = (
            avoidance.goal_bearing_rad
        )

        if (
            expected_bearing_rad is None
            or self.locked_target_id is None
        ):
            return self._fail_target_lost(
                motion_backend=motion_backend,
                reason=(
                    "post_avoidance_target_missing_"
                    "without_expected_bearing"
                ),
            )

        reverse_limit_mm = (
            self._post_avoidance_reverse_limit_mm()
        )

        print(
            "[APPROACH_OBJECT][LOCAL_AVOIDANCE] "
            "target not fresh at handoff "
            f"age={None if snap is None else snap.age_s} "
            "-> bearing/reverse reacquire "
            f"expected="
            f"{math.degrees(expected_bearing_rad):+.1f}deg "
            f"reverse_limit={reverse_limit_mm:.0f}mm"
        )

        self._post_avoidance_reacquire = (
            ReacquireTargetBearingReverse(
                config=self.config,
                kind=self.kind,
                target_id=self.locked_target_id,
                expected_bearing_rad=(
                    expected_bearing_rad
                ),
                reverse_limit_mm=(
                    reverse_limit_mm
                ),
                max_age_s=float(
                    self.config.visible_max_age_s
                ),
            )
        )

        self._post_avoidance_reacquire.start(
            lvl2=lvl2,
            motion_backend=motion_backend,
            io=io,
        )

        print(
            "[APPROACH_OBJECT][LOCAL_AVOIDANCE] "
            "target not visible at handoff "
            "-> bearing/reverse reacquire "
            f"expected="
            f"{math.degrees(expected_bearing_rad):+.1f}deg "
            f"reverse_limit={reverse_limit_mm:.0f}mm"
        )

        return self.status

    def _update_post_avoidance_reacquire(
        self,
        *,
        perception,
        motion_backend,
    ):
        skill = self._post_avoidance_reacquire

        if skill is None:
            return self.status

        st = skill.update(
            perception=perception,
            motion_backend=motion_backend,
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            reason = skill.failure_reason

            self._post_avoidance_reacquire = None

            return self._fail_target_lost(
                motion_backend=motion_backend,
                reason=(
                    "post_avoidance_reacquire_failed:"
                    f"{reason}"
                ),
            )

        if skill.found_target is not None:
            self.target = skill.found_target

        print(
            "[APPROACH_OBJECT][LOCAL_AVOIDANCE] "
            "target reacquired "
            "-> ApproachTargetServo"
        )

        self._post_avoidance_reacquire = None
        self._approach_skill = None

        return self.status

    # -------------------------
    # Phase: APPROACHING (delegated)
    # -------------------------

    def _approach(
            self,
            lvl2,
            perception,
            motion_backend,
            *,
            io=None,
            localisation=None,
    ):

        if self._post_avoidance_reacquire is not None:
            return self._update_post_avoidance_reacquire(
                perception=perception,
                motion_backend=motion_backend,
            )

        # --------------------------------------------------
        # Stage-2 LocalAvoidance supervisor
        # --------------------------------------------------

        if (
            self._navigation_stage == 2
            and io is not None
        ):
            avoidance = (
                self._ensure_local_avoidance()
            )

            if avoidance is not None:
                now_local_s = float(
                    io.time()
                )

                visible_objects = (
                    self._visible_avoidance_objects(
                        perception
                    )
                )

                (
                    goal_distance_mm,
                    goal_bearing_rad,
                ) = self._avoidance_goal()

                obstacle_observations = (
                    self._avoidance_observations(
                        visible_objects=(
                            visible_objects
                        ),
                    )
                )

                # ------------------------------------------
                # LocalAvoidance currently owns motion.
                # ------------------------------------------

                if self._local_avoidance_active:

                    avoidance_status = (
                        self._update_local_avoidance(
                            avoidance=avoidance,
                            visible_objects=(
                                visible_objects
                            ),
                            obstacle_observations=(
                                obstacle_observations
                            ),
                            goal_distance_mm=(
                                goal_distance_mm
                            ),
                            goal_bearing_rad=(
                                goal_bearing_rad
                            ),
                            now_s=now_local_s,
                        )
                    )

                    if (
                        avoidance_status
                        == PrimitiveStatus.RUNNING
                    ):
                        return self.status

                    if (
                        avoidance_status
                        == PrimitiveStatus.FAILED
                    ):
                        print(
                            "[APPROACH_OBJECT]"
                            "[LOCAL_AVOIDANCE] "
                            "FAILED "
                            f"reason={avoidance.reason}"
                        )

                        self.failure_reason = (
                            "local_avoidance_failed"
                        )

                        self._local_avoidance_active = False
                        self.status = (
                            BehaviorStatus.FAILED
                        )

                        return self.status

                    return self._handoff_after_local_avoidance(
                        avoidance=avoidance,
                        lvl2=lvl2,
                        motion_backend=motion_backend,
                        io=io,
                    )

                # ------------------------------------------
                # Nominal servo owns motion.
                # Check whether it should be interrupted.
                # ------------------------------------------

                if (
                    goal_distance_mm is not None
                    and goal_bearing_rad is not None
                    and avoidance.nominal_corridor_blocked(
                        goal_distance_mm=(
                            goal_distance_mm
                        ),
                        goal_bearing_rad=(
                            goal_bearing_rad
                        ),
                        obstacle_observations=(
                            obstacle_observations
                        ),
                    )
                ):
                    print(
                        "[APPROACH_OBJECT]"
                        "[LOCAL_AVOIDANCE] "
                        "nominal corridor blocked "
                        "-> takeover"
                    )

                    self._safe_stop(
                        self._approach_skill,
                        motion_backend=(
                            motion_backend
                        ),
                    )

                    self._approach_skill = None

                    avoidance.start(
                        lvl2=lvl2,
                        calibration=self.calibration,
                        now_s=now_local_s,
                        localisation=localisation,
                        io=io,
                    )

                    self._local_avoidance_active = True

                    avoidance_status = (
                        self._update_local_avoidance(
                            avoidance=avoidance,
                            visible_objects=(
                                visible_objects
                            ),
                            obstacle_observations=(
                                obstacle_observations
                            ),
                            goal_distance_mm=(
                                goal_distance_mm
                            ),
                            goal_bearing_rad=(
                                goal_bearing_rad
                            ),
                            now_s=now_local_s,
                        )
                    )

                    if (
                        avoidance_status
                        == PrimitiveStatus.FAILED
                    ):
                        self.failure_reason = (
                            "local_avoidance_failed"
                        )

                        self._local_avoidance_active = False
                        self.status = (
                            BehaviorStatus.FAILED
                        )


                    elif (

                            avoidance_status

                            == PrimitiveStatus.SUCCEEDED

                    ):

                        return self._handoff_after_local_avoidance(

                            avoidance=avoidance,

                            lvl2=lvl2,

                            motion_backend=motion_backend,

                            io=io,

                        )

                    return self.status

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
                localisation=localisation,
                io=io,
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
        # If that becomes unavailable, stop continuous visual_servoing
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
        self._safe_stop(
            self._wall_acquisition_skill,
            motion_backend=motion_backend,
        )
        self._safe_stop(
            self._align_skill,
            motion_backend=motion_backend,
        )
        self._safe_stop(
            self._local_avoidance,
            motion_backend=motion_backend,
        )

        self._safe_stop(
            self._post_avoidance_reacquire,
            motion_backend=motion_backend,
        )
