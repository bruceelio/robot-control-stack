# skills/navigation/approach_target.py

import time

from primitives.base import Primitive, PrimitiveStatus
from primitives.motion import Drive, Rotate

from skills.navigation.align_to_target import AlignToTarget
from skills.perception.reacquire_target import ReacquireTarget

# Your perception helper (from perception.py)
from perception import get_visible_targets


class ApproachTarget(Primitive):
    """
    Skill: ApproachTarget (FULL APPROACH LOOP)

    Responsibilities:
      - Repeatedly: sense -> plan -> rotate -> drive -> settle -> repeat
      - Handles vision loss via ReacquireTarget
      - Handles final commit and returns SUCCEEDED when positioned for grabbing

    Inputs (constructor):
      - config: config object (drive/rotate/vision params)
      - kind: target kind string (e.g. "basic")
      - height_model: HeightModel instance (shared with AcquireObject)
    """

    def __init__(self, *, config, kind: str, height_model):
        super().__init__()
        self.config = config
        self.kind = kind
        self.height_model = height_model

        # Approach loop state
        self.active_primitive = None
        self.last_action = None  # "rotate" / "drive" / "reacquire"
        self.settle_until = None

        self.cached_distance = None
        self.cached_bearing = None

        self.final_commit = False
        self.approach_started = False
        self.target_is_high = None

        self.bearing_consumed = False

        self.last_seen_time = None
        self.last_seen_distance = None
        self.last_seen_bearing = None
        self.last_drive_step = None
        self.target_id = None

    @staticmethod
    def _band_label(distance, commit, direct):
        if distance <= commit:
            return "A"
        elif distance <= commit + direct:
            return "B"
        else:
            return "C"

    def start(self, *, motion_backend, seed_target=None, **_):
        # IMPORTANT: do NOT rotate here. AcquireObject already did ALIGN.
        now = time.time()

        self.active_primitive = None
        self.last_action = None
        self.settle_until = None

        self.cached_distance = None
        self.cached_bearing = None

        self.final_commit = False
        self.approach_started = False
        self.target_is_high = None
        self.bearing_consumed = False

        self.last_drive_step = None
        self.target_id = seed_target.get("id") if seed_target else None

        if seed_target is not None:
            self.last_seen_time = now
            self.last_seen_distance = float(seed_target.get("distance", 0.0))
            self.last_seen_bearing = float(seed_target.get("bearing", 0.0))
        else:
            self.last_seen_time = None
            self.last_seen_distance = None
            self.last_seen_bearing = None

    def update(self, *, perception, motion_backend, **_):
        now = time.time()

        if self.last_seen_time is None:
            self.last_seen_time = now

        # =================================================
        # SECTION 0 — SETTLING PHASE (AFTER DRIVE)
        # =================================================
        if self.settle_until is not None:
            if now < self.settle_until:
                print(f"[APPROACH][SETTLE] waiting {self.settle_until - now:.2f}s")
                return PrimitiveStatus.RUNNING

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
            prim = self.active_primitive

            # Dispatch by “needs perception?”
            if isinstance(prim, ReacquireTarget):
                prim_status = prim.update(motion_backend=motion_backend, perception=perception)
            elif isinstance(prim, (Drive, Rotate)):
                prim_status = prim.update(motion_backend=motion_backend)
            else:
                # AlignToTarget (motion-only skill)
                prim_status = prim.update(motion_backend=motion_backend)

            if prim_status == PrimitiveStatus.SUCCEEDED:
                action = self.last_action or "UNKNOWN"
                print(f"[APPROACH][{action.upper()}] complete")

                self.active_primitive = None

                if self.last_action == "reacquire":
                    # Reacquire finished; loop around and re-sense/plan
                    self.last_action = None
                    return PrimitiveStatus.RUNNING

                if self.last_action == "rotate":
                    # after rotate, do the drive
                    if self.final_commit:
                        drive_mm = float(self.cached_distance or self.config.min_drive_mm)
                    else:
                        drive_mm = max(
                            self.config.min_drive_mm,
                            min(self.config.max_drive_mm, float(self.cached_distance or self.config.min_drive_mm)),
                        )

                    print(f"[MOTION][DRIVE] distance={drive_mm:.0f}mm")
                    self.last_drive_step = drive_mm

                    self.active_primitive = Drive(distance_mm=drive_mm)
                    self.last_action = "drive"
                    self.active_primitive.start(motion_backend=motion_backend)
                    return PrimitiveStatus.RUNNING

                if self.last_action == "drive":
                    # Post-drive bookkeeping
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

                    # Final drive completed -> SUCCESS (ready to grab)
                    if self.final_commit:
                        print("[APPROACH][FINAL] drive complete — positioned for grab")
                        return PrimitiveStatus.SUCCEEDED

                    # otherwise settle then re-sense
                    self.settle_until = now + self.config.camera_settle_time
                    self.bearing_consumed = False
                    self.cached_bearing = None

                    print(f"[APPROACH][SETTLE] start {self.config.camera_settle_time:.2f}s")
                    return PrimitiveStatus.RUNNING

            elif prim_status == PrimitiveStatus.FAILED:
                print(f"[APPROACH][{(self.last_action or 'UNKNOWN').upper()}] FAILED")

                # If we fail to reacquire, bubble up a failure
                if self.last_action == "reacquire":
                    self.active_primitive = None
                    self.last_action = None
                    self.cached_distance = None
                    self.cached_bearing = None
                    self.bearing_consumed = False
                    self.last_drive_step = None
                    self.settle_until = None
                    return PrimitiveStatus.FAILED

                self.active_primitive = None
                self.last_action = None
                self.cached_distance = None
                self.cached_bearing = None
                return PrimitiveStatus.FAILED

            return PrimitiveStatus.RUNNING

        # =================================================
        # SECTION 2 — REASSESS TARGET (ONLY THINKING POINT)
        # =================================================

        visible = get_visible_targets(
            perception,
            self.kind,
            now=now,
            max_age_s=self.config.vision_loss_timeout_s,
        )

        # Prefer the originally-selected id if we have one (sticky)
        target = None
        if self.target_id is not None:
            target = next((t for t in visible if t.get("id") == self.target_id), None)

            # Don't immediately switch ids on loss; keep trying for a short window
            if target is None:
                GIVE_UP_S = 2.0
                age = (now - self.last_seen_time) if self.last_seen_time is not None else 0.0
                if age < GIVE_UP_S:
                    visible = []  # force "lost" behavior -> reacquire
                else:
                    # allow switching after timeout
                    self.target_id = None

        # fallback: closest visible of this kind (only if not sticky anymore)
        if target is None and self.target_id is None:
            target = visible[0] if visible else None

        age = (now - self.last_seen_time) if self.last_seen_time is not None else 0.0
        print(
            f"[APPROACH][DEBUG] kind={self.kind} "
            f"target={'YES' if target else 'NO'} "
            f"last_seen_age(before_update)={age:.2f}s"
        )

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
            # Vision lost — decide blind-continue (rare) or reacquire
            commit = self.config.final_commit_distance_mm
            direct = self.config.final_approach_direct_range_mm

            # “high object” blind allowance
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

                # rotate(0) acts as "do drive next" using the same rotate->drive pipeline
                self.active_primitive = Rotate(angle_deg=0.0)
                self.active_primitive.start(motion_backend=motion_backend)
                return PrimitiveStatus.RUNNING

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
            return PrimitiveStatus.RUNNING

        # =================================================
        # SECTION 3 — PLAN (we have a target)
        # =================================================

        tid = target.get("id", "REL")

        if not self.approach_started:
            self.approach_started = True
            print("[APPROACH] approach accepted — locking target type")

        print(
            f"[APPROACH][TARGET] kind={self.kind} id={tid} "
            f"dist={float(target['distance']):.0f}mm bearing={float(target['bearing']):.1f}°"
        )

        distance = float(target["distance"])
        bearing = float(target["bearing"])
        self.last_seen_time = now
        self.last_seen_distance = distance

        # Height model update/commit (ported)
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

        # Final commit trigger
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
            return PrimitiveStatus.RUNNING

        # If bearing already good, skip rotate -> drive
        if self.bearing_consumed:
            return PrimitiveStatus.RUNNING

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
            self.active_primitive = Drive(distance_mm=drive_mm)
            self.last_action = "drive"
            self.active_primitive.start(motion_backend=motion_backend)
            return PrimitiveStatus.RUNNING

        # Otherwise rotate then drive
        self.cached_bearing = bearing
        angle = max(-self.config.max_rotate_deg, min(self.config.max_rotate_deg, self.cached_bearing))

        print(f"[ROTATE-DEBUG] bearing={bearing:.2f} angle_cmd={angle:.2f}")

        commit = self.config.final_commit_distance_mm
        direct = self.config.final_approach_direct_range_mm
        print(f"[APPROACH][RANGES] commit={commit}mm direct={direct}mm")

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
            return PrimitiveStatus.RUNNING

        self.last_action = "rotate"
        self.bearing_consumed = True

        print(f"[MOTION][ROTATE] angle={angle:.1f}° (bearing={bearing:.1f}°)")

        self.active_primitive = AlignToTarget(
            bearing_deg=angle,
            tolerance_deg=0.0,
            max_rotate_deg=self.config.max_rotate_deg,
        )
        self.active_primitive.start(motion_backend=motion_backend)
        return PrimitiveStatus.RUNNING

    def stop(self):
        if self.active_primitive is not None:
            self.active_primitive.stop()
        self.active_primitive = None
