# behaviors/seek_and_collect.py

import time

from behaviors.base import Behavior, BehaviorStatus
from primitives.motion import Rotate, Drive
from primitives.manipulation import Grab, LiftUp, LiftDown
from primitives.base import PrimitiveStatus
from navigation.target_selection import get_closest_target
from navigation.height_model import HeightModel


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
        self.last_target = None
        self.last_target_time = None
        self.last_seen_time = None
        self.VISION_LOSS_TIMEOUT = 0.5  # seconds (config later)
        self.pending_drive = False
        self.drive_distance = None
        self.cached_distance = None
        self.cached_bearing = None
        self.search_blocked_until = None
        self.final_commit = False
        self.approach_started = False
        self.height_model = HeightModel()
        self.target_is_high = None
        self.final_actions = None
        self.final_index = 0
        self.bearing_consumed = False
        self.last_seen_distance = None

    @staticmethod
    def _band_label(distance, commit, direct):
        if distance <= commit:
            return "A"
        elif distance <= commit + direct:
            return "B"
        else:
            return "C"

    def start(self, *, config, kind=None):
        self.config = config
        self.state = "SEARCHING"
        self.kind = kind or config.default_target_kind
        print(f"[SEEK_AND_COLLECT][START] kind={self.kind}")
        self.target = None
        self.active_primitive = None
        self.final_commit = False
        self.approach_started = False
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
        self.bearing_consumed = False

        if self.target is None:
            self.status = BehaviorStatus.FAILED
            return self.status

        print(
            f"[SEARCH] target found "
            f"id={self.target.get('id', 'REL')} "
            f"dist={self.target['distance']:.0f} "
            f"bearing={self.target['bearing']:.1f}"
        )

        # Hand off to APPROACH — NO MOTION HERE
        self.state = "APPROACHING"
        self.approach_started = False
        self.active_primitive = None
        self.last_action = None
        self.cached_distance = None
        self.cached_bearing = None

        return self.status

    def _approach(self, perception, motion_backend):
        # HARD GUARD — do not re-enter after state change
        if self.state != "APPROACHING":
            return self.status

        now = time.time()

        # =================================================
        # SECTION 0 — SETTLING PHASE (AFTER DRIVE)
        # =================================================
        if self.settle_until is not None:
            if now < self.settle_until:
                print(
                    f"[APPROACH][SETTLE] waiting "
                    f"{self.settle_until - now:.2f}s"
                )
                return self.status

            # ---- LOOP CLOSE: settle complete
            print("[APPROACH][SETTLE] complete — plan consumed")

            self.settle_until = None
            self.cached_distance = None
            self.last_action = None
            self.bearing_consumed = False

            # reassessment allowed after this point

        # =================================================
        # SECTION 1 — ACTIVE PRIMITIVE (NO REASSESSMENT)
        # =================================================

        if self.active_primitive is not None:
            prim_status = self.active_primitive.update(
                motion_backend=motion_backend
            )

            if prim_status == PrimitiveStatus.SUCCEEDED:
                print(f"[APPROACH][{self.last_action.upper()}] complete")

                self.active_primitive = None

                # ---------- LOOP CLOSE: ROTATE → DRIVE ----------
                if self.last_action == "rotate":
                    if self.final_commit:
                        drive_mm = self.cached_distance
                    else:
                        drive_mm = max(
                            self.config.min_drive_mm,
                            min(
                                self.config.max_drive_mm,
                                self.cached_distance
                            )
                        )

                    print(
                        "[MOTION][DRIVE] "
                        f"distance={drive_mm:.0f}mm"
                    )

                    self.active_primitive = Drive(distance_mm=drive_mm)
                    self.last_action = "drive"
                    self.active_primitive.start(
                        motion_backend=motion_backend
                    )
                    return self.status

                                # ---------- LOOP CLOSE: DRIVE ----------
                if self.last_action == "drive":

                    # --- POSITION SUMMARY (POST-DRIVE) ---
                    if self.last_seen_distance is not None and self.cached_distance is not None:
                        after_est = max(
                            0.0,
                            self.last_seen_distance - self.cached_distance
                        )

                        band_after = self._band_label(
                            after_est,
                            self.config.final_commit_distance_mm,
                            self.config.final_approach_direct_range_mm,
                        )

                        print(
                            "[APPROACH][POS] "
                            f"before={self.last_seen_distance:.0f}mm "
                            f"drove={self.cached_distance:.0f}mm "
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

                        # -------------------------
                        # FINAL BLIND PICKUP PLAN
                        # -------------------------
                        self.final_actions = [
                            LiftUp(),
                            LiftDown(),
                            Grab(),
                            LiftUp(),
                        ]
                        self.final_index = 0

                        self.state = "GRABBING"
                        return self.status

                    # NORMAL DRIVE: settle + reassess
                    self.settle_until = now + self.config.camera_settle_time

                    # 🔧 IMPORTANT: allow corrective steering after a drive
                    self.bearing_consumed = False
                    self.cached_bearing = None

                    print(f"[APPROACH][SETTLE] start {self.config.camera_settle_time:.2f}s")
                    return self.status


            elif prim_status == PrimitiveStatus.FAILED:
                print(f"[APPROACH][{self.last_action.upper()}] FAILED")
                self.active_primitive = None
                self.last_action = None
                self.cached_distance = None
                self.status = BehaviorStatus.FAILED
                return self.status

            return self.status

        # =================================================
        # SECTION 2 — REASSESS TARGET (ONLY THINKING POINT)
        # =================================================

        target = get_closest_target(perception, self.kind)

        # =================================================
        # (3) TARGET VISIBLE → USE BEARING DIRECTLY
        # =================================================
        if target is not None:
            distance = target["distance"]
            bearing = target["bearing"]

            self.last_seen_time = now
            self.last_seen_distance = distance

            band = self._band_label(
                distance,
                self.config.final_commit_distance_mm,
                self.config.final_approach_direct_range_mm,
            )

            print(
                "[APPROACH][SENSE] "
                f"vision=VISIBLE "
                f"band={band} "
                f"dist={distance:.0f}mm "
                f"bearing={bearing:.1f}°"
            )

        # =================================================
        # (4) TARGET NOT VISIBLE → CONSIDER BLIND FINAL
        # =================================================
        else:
            # Only consider blind motion AFTER a drive + settle
            if (
                    self.last_seen_distance is not None
                    and self.cached_distance is not None
            ):
                remaining = self.last_seen_distance - self.cached_distance

                if remaining <= self.config.final_commit_distance_mm:
                    print(
                        "[APPROACH][BLIND] final allowed "
                        f"(remaining={remaining:.0f}mm)"
                    )

                    # --- BLIND RULES YOU STATED ---
                    # rotate 0°
                    # drive remaining distance
                    self.cached_bearing = 0.0
                    self.cached_distance = max(
                        self.config.min_drive_mm,
                        remaining
                    )

                    self.last_action = "rotate"
                    self.bearing_consumed = True

                    self.active_primitive = Rotate(angle_deg=0.0)
                    self.active_primitive.start(
                        motion_backend=motion_backend
                    )
                    return self.status

            # Blind not allowed → WAIT for vision
            print(
                "[APPROACH][SENSE] "
                "vision=LOST "
                "intent=WAIT"
            )

            return self.status

        # ---------- REL TARGET GUARD ----------
        tid = target.get("id", "REL")

        if not self.approach_started:
            self.approach_started = True

            # ---------- LATCH DEBUG (NAVIGATION-BASED, SAFE) ----------
            acidic_t = get_closest_target(perception, "acidic")
            basic_t = get_closest_target(perception, "basic")

            acidic_min = acidic_t["distance"] if acidic_t else None
            basic_min = basic_t["distance"] if basic_t else None

            print(
                "[APPROACH][LATCH] "
                f"selected_kind={self.kind} "
                f"acidic_min={acidic_min} "
                f"basic_min={basic_min}"
            )
            # ---------- END LATCH DEBUG ----------

            print("[APPROACH] approach accepted — locking target type")

        # ---------- SAFE DEBUG PRINT ----------
        print(
            f"[APPROACH][TARGET] "
            f"kind={self.kind} id={tid} "
            f"dist={target['distance']:.0f}mm "
            f"bearing={target['bearing']:.1f}°"
        )

        # TARGET SEEN — SNAPSHOT
        self.last_seen_time = now

        distance = target["distance"]
        bearing = target["bearing"]
        self.last_seen_distance = distance

        # ---------- HEIGHT INFERENCE (VISION PHASE ONLY) ----------
        if (
                distance <= self.config.marker_height_max_distance_mm
                and not self.height_model.is_committed()
        ):
            pitch = target["marker"].orientation.pitch
            self.height_model.update(pitch_deg=pitch)

            committed = self.height_model.try_commit(
                high_thresh=self.config.marker_pitch_high_deg,
                low_thresh=self.config.marker_pitch_low_deg,
            )

            if committed:
                print(
                    f"[HEIGHT] committed "
                    f"{'HIGH' if self.height_model.is_high() else 'LOW'} "
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

            print(
                f"[APPROACH][FINAL] commit "
                f"dist={distance:.0f}mm "
                f"final_drive={final_drive_mm:.0f}mm"
            )

            # lock values — NO MORE VISION AFTER THIS
            self.cached_distance = final_drive_mm
            self.cached_bearing = bearing
            self.last_action = "rotate"

            angle = max(
                -self.config.max_rotate_deg,
                min(self.config.max_rotate_deg, self.cached_bearing)
            )

            self.active_primitive = Rotate(angle_deg=angle)
            self.active_primitive.start(motion_backend=motion_backend)

            return self.status


        # =================================================
        # SECTION 3 — PLAN ATOMIC ROTATE → DRIVE
        # =================================================

        assert self.active_primitive is None
        assert self.last_action is None
        if self.bearing_consumed:
            # Rotate already decided for this plan; waiting for drive/settle to close loop.
            return self.status

        # tiny-angle deadband
        # tiny-angle deadband: skip rotate, but still DRIVE
        if abs(bearing) < self.config.min_rotate_deg:
            print("[APPROACH] bearing within tolerance — skipping rotate, planning drive")

            # Treat as straight drive
            self.cached_bearing = 0.0

            # Use the same B/C distance planning logic as below
            commit = self.config.final_commit_distance_mm
            direct = self.config.final_approach_direct_range_mm

            if distance > commit + direct:
                remaining_to_commit = distance - commit
                step = max(self.config.min_drive_mm, min((remaining_to_commit / 2), self.config.max_drive_mm))
                self.cached_distance = step
            elif distance > commit:
                self.cached_distance = distance - commit
            else:
                # If we’re inside commit band, normal flow should have handled final_commit earlier
                self.cached_distance = self.config.min_drive_mm

            # Start DRIVE immediately (no rotate primitive)
            drive_mm = max(self.config.min_drive_mm, min(self.config.max_drive_mm, self.cached_distance))
            print(f"[MOTION][DRIVE] distance={drive_mm:.0f}mm (no-rotate path)")

            self.active_primitive = Drive(distance_mm=drive_mm)
            self.last_action = "drive"
            self.active_primitive.start(motion_backend=motion_backend)

            return self.status

        self.cached_bearing = bearing

        angle = max(
            -self.config.max_rotate_deg,
            min(self.config.max_rotate_deg, self.cached_bearing)
        )

        print(
            f"[ROTATE-DEBUG] bearing={bearing:.2f} "
            f"angle_cmd={angle:.2f}"
        )

        # ---------- PROGRESSIVE APPROACH STEP ----------
        commit = self.config.final_commit_distance_mm
        direct = self.config.final_approach_direct_range_mm

        print(
            "[APPROACH][RANGES] "
            f"commit={commit}mm "
            f"direct={direct}mm"
        )

        # ===============================
        # B / C DISTANCE PLANNING LOGIC
        # ===============================

        # --- C) Beyond DIRECT band ---
        if distance > commit + direct:
            # drive halfway toward COMMIT (not toward marker)
            remaining_to_commit = distance - commit
            step = remaining_to_commit / 2

            step = max(
                self.config.min_drive_mm,
                min(step, self.config.max_drive_mm)
            )

            self.cached_distance = step

            print(
                "[APPROACH][PLAN] "
                "band=C "
                "vision=VISIBLE "
                f"dist={distance:.0f}mm "
                "intent=HALF_TO_COMMIT "
                f"drive_target={step:.0f}mm"
            )

        # --- B) Inside DIRECT band ---
        elif distance > commit:
            # drive directly to COMMIT
            self.cached_distance = distance - commit

            print(
                "[APPROACH][PLAN] "
                "band=B "
                "vision=VISIBLE "
                f"dist={distance:.0f}mm "
                "intent=DIRECT_TO_COMMIT "
                f"drive_target={self.cached_distance:.0f}mm"
            )


        # --- A) Inside COMMIT ---
        else:
            # should never hit here; handled earlier
            return self.status

        self.last_action = "rotate"
        self.bearing_consumed = True
        self.active_primitive = Rotate(angle_deg=angle)

        print(
            "[MOTION][ROTATE] "
            f"angle={angle:.1f}° "
            f"(bearing={bearing:.1f}°)"
        )

        self.active_primitive = Rotate(angle_deg=angle)
        self.active_primitive.start(
            motion_backend=motion_backend
        )

        return self.status

    def _grab(self, lvl2):
        if self.final_actions is None:
            print("[GRAB] ERROR: no final action plan")
            self.status = BehaviorStatus.FAILED
            return self.status

        # Start next primitive if none active
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

        # Update active primitive
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

