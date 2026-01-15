import time

from behaviors.base import Behavior, BehaviorStatus
from primitives.motion import Rotate, Drive
from primitives.manipulation import Grab, LiftUp, LiftDown
from primitives.base import PrimitiveStatus
from navigation.target_selection import get_closest_target
from navigation.height_model import HeightModel
from config.base import DEFAULT_TARGET_KIND
from config.base import (
    MARKER_HEIGHT_MAX_DISTANCE_MM,
    FINAL_COMMIT_DISTANCE_MM,
    MARKER_PITCH_HIGH_DEG,
    MARKER_PITCH_LOW_DEG,
)
from config.base import (
    MIN_ROTATE_DEG,
    MAX_ROTATE_DEG,
    MIN_DRIVE_MM,
    MAX_DRIVE_MM,
    GRAB_DISTANCE_MM,
    CAMERA_SETTLE_TIME,
    FINAL_COMMIT_DISTANCE_MM
)


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

    def start(self, *, kind=None):
        self.state = "SEARCHING"
        self.kind = kind or DEFAULT_TARGET_KIND
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
        now = time.time()

        if self.search_blocked_until is not None:
            if now < self.search_blocked_until:
                return self.status
            self.search_blocked_until = None

        self.target = get_closest_target(perception, self.kind)
        if self.target is None:
            self.status = BehaviorStatus.FAILED
            return self.status

        if self.active_primitive is None:
            self.active_primitive = Rotate(
                angle_deg=self.target["bearing"]
            )
            self.active_primitive.start(
                motion_backend=motion_backend
            )

        prim_status = self.active_primitive.update(
            motion_backend=motion_backend
        )

        if prim_status == PrimitiveStatus.SUCCEEDED:
            self.active_primitive = None
            self.state = "APPROACHING"
            self.approach_started = False  # reset latch

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
            self.cached_bearing = None
            self.last_action = None

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
                            MIN_DRIVE_MM,
                            min(
                                MAX_DRIVE_MM,
                                self.cached_distance - GRAB_DISTANCE_MM
                            )
                        )

                    print(f"[APPROACH][DRIVE] start distance={drive_mm:.0f}mm")

                    self.active_primitive = Drive(distance_mm=drive_mm)
                    self.last_action = "drive"
                    self.active_primitive.start(
                        motion_backend=motion_backend
                    )
                    return self.status

                                # ---------- LOOP CLOSE: DRIVE ----------
                if self.last_action == "drive":

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
                    self.settle_until = now + CAMERA_SETTLE_TIME
                    print(f"[APPROACH][SETTLE] start {CAMERA_SETTLE_TIME:.2f}s")
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

        if target is None:
            print("[APPROACH][EXIT] no target — returning to SEARCHING")
            self.state = "SEARCHING"
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

        # ---------- HEIGHT INFERENCE (VISION PHASE ONLY) ----------
        if (
                distance <= MARKER_HEIGHT_MAX_DISTANCE_MM
                and not self.height_model.is_committed()
        ):
            pitch = target["marker"].orientation.pitch
            self.height_model.update(pitch_deg=pitch)

            committed = self.height_model.try_commit(
                high_thresh=MARKER_PITCH_HIGH_DEG,
                low_thresh=MARKER_PITCH_LOW_DEG,
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
                and distance <= FINAL_COMMIT_DISTANCE_MM
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
                -MAX_ROTATE_DEG,
                min(MAX_ROTATE_DEG, bearing)
            )

            self.active_primitive = Rotate(angle_deg=angle)
            self.active_primitive.start(motion_backend=motion_backend)

            return self.status


        # =================================================
        # SECTION 3 — PLAN ATOMIC ROTATE → DRIVE
        # =================================================

        assert self.active_primitive is None
        assert self.last_action is None

        self.cached_bearing = bearing

        angle = max(
            -MAX_ROTATE_DEG,
            min(MAX_ROTATE_DEG, self.cached_bearing)
        )

        self.cached_distance = distance  # <-- consumed once
        self.last_action = "rotate"

        print(
            f"[APPROACH][ROTATE] start "
            f"angle={angle:.1f}° "
            f"(raw={bearing:.1f}°)"
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

