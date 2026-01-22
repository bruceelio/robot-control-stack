# behaviors/acquire_object.py

import time

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus
from primitives.manipulation import Grab, LiftUp, LiftDown
from primitives.motion import Rotate

from skills.navigation.align_to_target import AlignToTarget
from skills.navigation.approach_target import ApproachTarget
from skills.perception.reacquire_target import ReacquireTarget
from skills.perception.select_target import get_closest_target
from navigation.height_model import HeightModel


class AcquireObject(Behavior):
    """
    Partitioned version of SeekAndCollect (pickup half only):
      SELECT -> ALIGN -> APPROACHING -> GRABBING -> SUCCEEDED (object held)

    No delegation to SeekAndCollect.
    """

    def __init__(self):
        super().__init__()

        self.config = None
        self.kind = None

        self.phase = "SELECT"
        self.target = None

        # SELECT/ALIGN helpers
        self._align_skill = None
        self._last_no_target_log = None

        # Approach state (migrated from SeekAndCollect)
        self.active_primitive = None
        self.last_action = None  # "rotate" or "drive" or "reacquire"
        self.settle_until = None

        self.cached_distance = None
        self.cached_bearing = None

        self.final_commit = False
        self.approach_started = False

        self.height_model = HeightModel()
        self.target_is_high = None

        self.bearing_consumed = False

        self.last_seen_time = None
        self.last_seen_distance = None
        self.last_seen_bearing = None
        self.last_drive_step = None

        # Grab sequence (migrated from SeekAndCollect)
        self.final_actions = None
        self.final_index = 0

    @staticmethod
    def _band_label(distance, commit, direct):
        if distance <= commit:
            return "A"
        elif distance <= commit + direct:
            return "B"
        else:
            return "C"

    def start(self, *, config, kind=None, seed_target=None, **_):
        print("[ACQUIRE_OBJECT] start")
        self.config = config
        self.kind = kind or config.default_target_kind

        # Allow optional seed handoff (same semantics as SeekAndCollect)
        self.target = seed_target
        self._seed_used = seed_target is None

        self.phase = "SELECT" if seed_target is None else "ALIGN"

        self._align_skill = None
        self._last_no_target_log = None

        self.active_primitive = None
        self.last_action = None
        self.settle_until = None

        self.cached_distance = None
        self.cached_bearing = None

        self.final_commit = False
        self.approach_started = False

        self.height_model = HeightModel()
        self.target_is_high = None

        self.bearing_consumed = False

        now = time.time()
        self.last_seen_time = now if seed_target is not None else None
        self.last_seen_distance = seed_target.get("distance") if seed_target is not None else None
        self.last_seen_bearing = seed_target.get("bearing") if seed_target is not None else None
        self.last_drive_step = None

        self.final_actions = None
        self.final_index = 0

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
        now = time.time()

        # Use seed target first (handover correctness)
        if self.target is not None and not self._seed_used:
            self._seed_used = True
            print(
                f"[ACQUIRE_OBJECT][SELECT] seeded target "
                f"id={self.target.get('id', 'REL')} "
                f"dist={self.target['distance']:.0f} "
                f"bearing={self.target['bearing']:.1f}"
            )
            self.phase = "ALIGN"
            self._align_skill = None
            return self.status

        self.target = get_closest_target(
            perception,
            self.kind,
            now=now,
            max_age_s=self.config.vision_loss_timeout_s,
        )

        if self.target is None:
            if self._last_no_target_log is None or now - self._last_no_target_log > 1.0:
                print("[ACQUIRE_OBJECT] no target visible — waiting")
                self._last_no_target_log = now
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

        # hand off to approach loop
        self.phase = "APPROACHING"

        # Initialise approach state (same as SeekAndCollect handoff)
        self.active_primitive = None
        self.last_action = None
        self.settle_until = None
        self.cached_distance = None
        self.cached_bearing = None
        self.bearing_consumed = False
        self.last_drive_step = None

        now = time.time()
        self.last_seen_time = now
        self.last_seen_distance = float(self.target.get("distance", 0.0))
        self.last_seen_bearing = float(self.target.get("bearing", 0.0))

        return self.status

    # -------------------------
    # Phase: APPROACHING (ported from SeekAndCollect)
    # -------------------------

    def _approach(self, perception, motion_backend):
        now = time.time()

        if self.last_seen_time is None:
            self.last_seen_time = now

        # =================================================
        # SECTION 0 — SETTLING PHASE (AFTER DRIVE)
        # =================================================
        if self.settle_until is not None:
            if now < self.settle_until:
                print(f"[APPROACH][SETTLE] waiting {self.settle_until - now:.2f}s")
                return self.status

            print("[APPROACH][SETTLE] complete — plan consumed")

            self.settle_until = None
            self.last_action = None
            self.bearing_consumed = False

            if not self.final_commit:
                self.cached_distance = None

        # =================================================
        # SECTION 1 — ACTIVE PRIMITIVE (NO REASSESSMENT)
        # =================================================
        if self.active_primitive is not None:
            # Some primitives need perception (ApproachTarget/ReacquireTarget),
            # some need only motion_backend (Rotate/AlignToTarget), and some need nothing.
            prim = self.active_primitive

            if isinstance(prim, (ApproachTarget, ReacquireTarget)):
                prim_status = prim.update(motion_backend=motion_backend, perception=perception)
            elif isinstance(prim, (Rotate, AlignToTarget)):
                prim_status = prim.update(motion_backend=motion_backend)
            else:
                prim_status = prim.update()

            if prim_status == PrimitiveStatus.SUCCEEDED:
                action = self.last_action or "UNKNOWN"
                print(f"[APPROACH][{action.upper()}] complete")

                self.active_primitive = None

                # If we just completed a reacquire attempt, return to reassessment on next tick
                if self.last_action == "reacquire":
                    self.last_action = None
                    return self.status

                # ---------- LOOP CLOSE: ROTATE → DRIVE ----------
                if self.last_action == "rotate":
                    if self.final_commit:
                        drive_mm = self.cached_distance
                    else:
                        drive_mm = max(
                            self.config.min_drive_mm,
                            min(self.config.max_drive_mm, self.cached_distance),
                        )

                    print(f"[MOTION][DRIVE] distance={drive_mm:.0f}mm")

                    self.last_drive_step = drive_mm
                    self.active_primitive = ApproachTarget(distance_mm=drive_mm)
                    self.last_action = "drive"
                    self.active_primitive.start(motion_backend=motion_backend)
                    return self.status

                # ---------- LOOP CLOSE: DRIVE ----------
                if self.last_action == "drive":
                    if self.last_seen_distance is not None and self.last_drive_step is not None:
                        after_est = max(0.0, self.last_seen_distance - self.last_drive_step)
                        band_after = self._band_label(
                            after_est,
                            self.config.final_commit_distance_mm,
                            self.config.final_approach_direct_range_mm,
                        )
                        print(
                            "[APPROACH][POS] "
                            f"before={self.last_seen_distance:.0f}mm "
                            f"drove={self.last_drive_step:.0f}mm "
                            f"after_est={after_est:.0f}mm "
                            f"band={band_after}"
                        )

                    # FINAL COMMIT: drive ends → GRAB
                    if self.final_commit:
                        print("[APPROACH][FINAL] drive complete — executing blind pickup sequence")

                        self.active_primitive = None
                        self.last_action = None
                        self.cached_distance = None
                        self.cached_bearing = None
                        self.settle_until = None

                        self.final_actions = [LiftUp(), LiftDown(), Grab(), LiftUp()]
                        self.final_index = 0

                        self.phase = "GRABBING"
                        return self.status

                    # NORMAL DRIVE: settle + reassess
                    self.settle_until = now + self.config.camera_settle_time

                    # allow corrective steering after a drive
                    self.bearing_consumed = False
                    self.cached_bearing = None

                    print(f"[APPROACH][SETTLE] start {self.config.camera_settle_time:.2f}s")
                    return self.status

            elif prim_status == PrimitiveStatus.FAILED:
                print(f"[APPROACH][{(self.last_action or 'UNKNOWN').upper()}] FAILED")

                # If reacquire fails, fall back to SELECT (partition-aligned: reacquire failed -> pick again)
                if self.last_action == "reacquire":
                    self.active_primitive = None
                    self.last_action = None

                    self.cached_distance = None
                    self.cached_bearing = None
                    self.bearing_consumed = False
                    self.last_drive_step = None
                    self.settle_until = None

                    self.phase = "SELECT"
                    self.status = BehaviorStatus.RUNNING
                    return self.status

                # otherwise fail acquire
                self.active_primitive = None
                self.last_action = None
                self.cached_distance = None
                self.cached_bearing = None
                self.status = BehaviorStatus.FAILED
                return self.status

            return self.status

        # =================================================
        # SECTION 2 — REASSESS TARGET (ONLY THINKING POINT)
        # =================================================
        target = get_closest_target(
            perception,
            self.kind,
            now=now,
            max_age_s=self.config.vision_loss_timeout_s,
        )

        age = (now - self.last_seen_time) if self.last_seen_time is not None else 0.0
        print(
            f"[APPROACH][DEBUG] kind={self.kind} "
            f"target={'YES' if target else 'NO'} "
            f"last_seen_age(before_update)={age:.2f}s"
        )

        # (3) TARGET VISIBLE
        if target is not None:
            distance = float(target["distance"])
            bearing = float(target["bearing"])

            self.last_seen_time = now
            self.last_seen_distance = distance
            self.last_seen_bearing = bearing

            band = self._band_label(
                distance,
                self.config.final_commit_distance_mm,
                self.config.final_approach_direct_range_mm,
            )

            print(
                "[APPROACH][SENSE] "
                f"vision=VISIBLE band={band} "
                f"dist={distance:.0f}mm bearing={bearing:.1f}°"
            )
        else:
            # (4) TARGET NOT VISIBLE
            commit = self.config.final_commit_distance_mm
            direct = self.config.final_approach_direct_range_mm

            # HIGH + band B blind-to-commit rule
            if (
                self.height_model.is_committed()
                and self.height_model.is_high()
                and self.last_seen_distance is not None
                and (commit < self.last_seen_distance <= commit + direct)
            ):
                blind_to_commit = self.last_seen_distance - commit
                blind_to_commit = max(
                    self.config.min_drive_mm,
                    min(self.config.max_drive_mm, blind_to_commit),
                )

                print(
                    "[APPROACH][BLIND] "
                    f"vision=LOST high=YES band=B "
                    f"last_seen={self.last_seen_distance:.0f}mm "
                    f"drive_to_commit={blind_to_commit:.0f}mm"
                )

                self.cached_bearing = 0.0
                self.cached_distance = blind_to_commit
                self.last_action = "rotate"
                self.bearing_consumed = True

                self.active_primitive = Rotate(angle_deg=0.0)
                self.active_primitive.start(motion_backend=motion_backend)
                return self.status

            # Blind FINAL motion rule (after a drive)
            if self.last_seen_distance is not None and self.last_drive_step is not None:
                remaining = self.last_seen_distance - self.last_drive_step
                if remaining <= self.config.final_commit_distance_mm:
                    print(f"[APPROACH][BLIND] final allowed (remaining={remaining:.0f}mm)")

                    self.cached_bearing = 0.0
                    self.cached_distance = max(self.config.min_drive_mm, remaining)

                    self.last_action = "rotate"
                    self.bearing_consumed = True

                    self.active_primitive = Rotate(angle_deg=0.0)
                    self.active_primitive.start(motion_backend=motion_backend)
                    return self.status

            elapsed = now - (self.last_seen_time or now)
            print(f"[APPROACH][SENSE] vision=LOST intent=REACQUIRE t={elapsed:.2f}s")

            self.active_primitive = ReacquireTarget(
                kind=self.kind,
                step_deg=self.config.recover_step_deg,
                max_sweep_deg=self.config.recover_max_sweep_deg,
                max_age_s=self.config.vision_loss_timeout_s,
            )
            self.last_action = "reacquire"
            self.active_primitive.start(motion_backend=motion_backend)
            return self.status

        # ---------- target guard / debug ----------
        tid = target.get("id", "REL")

        if not self.approach_started:
            self.approach_started = True
            print("[APPROACH] approach accepted — locking target type")

        print(
            f"[APPROACH][TARGET] kind={self.kind} id={tid} "
            f"dist={target['distance']:.0f}mm bearing={target['bearing']:.1f}°"
        )

        distance = float(target["distance"])
        bearing = float(target["bearing"])
        self.last_seen_time = now
        self.last_seen_distance = distance

        # ---------- HEIGHT INFERENCE ----------
        if distance <= self.config.marker_height_max_distance_mm and not self.height_model.is_committed():
            pitch = target["marker"].orientation.pitch
            self.height_model.update(pitch_deg=pitch)
            committed = self.height_model.try_commit(
                high_thresh=self.config.marker_pitch_high_deg,
                low_thresh=self.config.marker_pitch_low_deg,
            )
            if committed:
                print(
                    f"[HEIGHT] committed {'HIGH' if self.height_model.is_high() else 'LOW'} "
                    f"(pitch={pitch:.3f})"
                )

        # ---------- FINAL COMMIT DECISION ----------
        if (
            not self.final_commit
            and distance <= self.config.final_commit_distance_mm
            and self.height_model.is_committed()
        ):
            self.final_commit = True
            self.target_is_high = self.height_model.is_high()

            final_drive_mm = distance + 70
            print(f"[APPROACH][FINAL] commit dist={distance:.0f}mm final_drive={final_drive_mm:.0f}mm")

            self.cached_distance = final_drive_mm
            self.cached_bearing = bearing
            self.last_action = "rotate"

            angle = max(-self.config.max_rotate_deg, min(self.config.max_rotate_deg, self.cached_bearing))

            self.active_primitive = AlignToTarget(
                bearing_deg=angle,
                tolerance_deg=0.0,
                max_rotate_deg=self.config.max_rotate_deg,
            )
            self.active_primitive.start(motion_backend=motion_backend)
            return self.status

        # =================================================
        # SECTION 3 — PLAN ATOMIC ROTATE → DRIVE
        # =================================================
        if self.last_action is None and self.active_primitive is None:
            self.last_drive_step = None

        if self.bearing_consumed:
            return self.status

        # tiny-angle deadband: skip rotate, still drive
        if abs(bearing) < self.config.min_rotate_deg:
            print("[APPROACH] bearing within tolerance — skipping rotate, planning drive")

            commit = self.config.final_commit_distance_mm
            direct = self.config.final_approach_direct_range_mm

            if distance > commit + direct:
                remaining_to_commit = distance - commit
                step = max(
                    self.config.min_drive_mm,
                    min((remaining_to_commit / 2), self.config.max_drive_mm),
                )
                self.cached_distance = step
            elif distance > commit:
                self.cached_distance = distance - commit
            else:
                self.cached_distance = self.config.min_drive_mm

            drive_mm = max(self.config.min_drive_mm, min(self.config.max_drive_mm, self.cached_distance))
            print(f"[MOTION][DRIVE] distance={drive_mm:.0f}mm (no-rotate path)")

            self.last_drive_step = drive_mm
            self.active_primitive = ApproachTarget(distance_mm=drive_mm)
            self.last_action = "drive"
            self.active_primitive.start(motion_backend=motion_backend)
            return self.status

        self.cached_bearing = bearing
        angle = max(-self.config.max_rotate_deg, min(self.config.max_rotate_deg, self.cached_bearing))

        print(f"[ROTATE-DEBUG] bearing={bearing:.2f} angle_cmd={angle:.2f}")

        commit = self.config.final_commit_distance_mm
        direct = self.config.final_approach_direct_range_mm

        print(f"[APPROACH][RANGES] commit={commit}mm direct={direct}mm")

        # B/C distance planning
        if distance > commit + direct:
            remaining_to_commit = distance - commit
            step = remaining_to_commit / 2
            step = max(self.config.min_drive_mm, min(step, self.config.max_drive_mm))
            self.cached_distance = step
            print(
                "[APPROACH][PLAN] band=C vision=VISIBLE "
                f"dist={distance:.0f}mm intent=HALF_TO_COMMIT drive_target={step:.0f}mm"
            )
        elif distance > commit:
            self.cached_distance = distance - commit
            print(
                "[APPROACH][PLAN] band=B vision=VISIBLE "
                f"dist={distance:.0f}mm intent=DIRECT_TO_COMMIT drive_target={self.cached_distance:.0f}mm"
            )
        else:
            return self.status

        self.last_action = "rotate"
        self.bearing_consumed = True

        print(f"[MOTION][ROTATE] angle={angle:.1f}° (bearing={bearing:.1f}°)")

        self.active_primitive = AlignToTarget(
            bearing_deg=angle,
            tolerance_deg=0.0,
            max_rotate_deg=self.config.max_rotate_deg,
        )
        self.active_primitive.start(motion_backend=motion_backend)
        return self.status

    # -------------------------
    # Phase: GRABBING (ported from SeekAndCollect)
    # -------------------------

    def _grab(self, lvl2):
        if self.final_actions is None:
            print("[GRAB] ERROR: no final action plan")
            self.status = BehaviorStatus.FAILED
            return self.status

        if self.active_primitive is None:
            if self.final_index >= len(self.final_actions):
                print("[GRAB] pickup sequence complete")
                self.final_actions = None
                self.status = BehaviorStatus.SUCCEEDED
                return self.status

            prim = self.final_actions[self.final_index]
            self.final_index += 1
            self.active_primitive = prim

            print(f"[GRAB] starting {prim.__class__.__name__}")
            prim.start(lvl2=lvl2)
            return self.status

        prim_status = self.active_primitive.update()

        if prim_status == PrimitiveStatus.SUCCEEDED:
            print(f"[GRAB] {self.active_primitive.__class__.__name__} complete")
            self.active_primitive = None
            return self.status

        if prim_status == PrimitiveStatus.FAILED:
            print(f"[GRAB] {self.active_primitive.__class__.__name__} FAILED")
            self.active_primitive = None
            self.status = BehaviorStatus.FAILED
            return self.status

        return self.status
