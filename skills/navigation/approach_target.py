# skills/navigation/approach_target.py

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Optional

from primitives.base import Primitive, PrimitiveStatus
from primitives.motion import Drive, Rotate

from skills.navigation.align_to_target import AlignToTarget
from skills.perception.reacquire_target import ReacquireTarget
from skills.perception.select_target_utils import get_closest_target

# IMPORTANT: your perception.py defines this helper at module level
# (not as a method on Perception), so we import it.
from perception import get_visible_targets


def _cfg(config: Any, name: str, fallback: Any) -> Any:
    """Read config field if present; else fallback."""
    return getattr(config, name, fallback)


@dataclass(frozen=True)
class ApproachTunables:
    """
    Tunables used by the approach loop.

    IMPORTANT: This file should not be a "single point of truth" for values.
    We still provide fallbacks here so the code runs even if a config field
    hasn't been added yet — but the intention is that your schema/resolver
    defines them centrally.
    """

    # Motion gating
    min_rotate_deg: float
    max_rotate_deg: float
    min_drive_mm: float
    max_drive_mm: float

    # Vision / timing
    camera_settle_time_s: float
    vision_loss_timeout_s: float
    visible_max_age_s: float  # used for "actually visible" vs "in memory"

    # Recovery sweep
    recover_step_deg: float
    recover_max_sweep_deg: float

    # LOW geometry (normal shelf / floor)
    final_commit_distance_mm: float
    final_approach_direct_range_mm: float
    final_approach_marker_push_mm: float

    # HIGH geometry (high shelf) — its own A/B/C track
    final_commit_distance_high_mm: float
    final_approach_direct_range_high_mm: float
    final_approach_max_degree_high: float

    # Height-model update gating
    marker_height_max_distance_mm: float
    marker_pitch_high_deg: float
    marker_pitch_low_deg: float


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

        self.t = self._load_tunables(config)

        # Approach loop state
        self.active_primitive: Optional[Primitive] = None
        self.last_action: Optional[str] = None  # "rotate" / "drive" / "reacquire"
        self.settle_until: Optional[float] = None

        self.cached_distance: Optional[float] = None
        self.cached_bearing: Optional[float] = None

        self.final_commit: bool = False
        self.approach_started: bool = False
        self.target_is_high: Optional[bool] = None

        self.bearing_consumed: bool = False

        self.last_seen_time: Optional[float] = None
        self.last_seen_distance: Optional[float] = None
        self.last_seen_bearing: Optional[float] = None
        self.last_drive_step: Optional[float] = None

        # lock onto initially chosen marker id (if provided)
        self.target_id: Optional[int] = None

    @property
    def approached_target_id(self) -> Optional[int]:
        """
        Read this after SUCCEEDED so the caller can:
          - remove from preferred list,
          - add to blacklist,
          - etc.
        """
        return self.target_id

    def _load_tunables(self, config: Any) -> ApproachTunables:
        # NOTE: fallbacks are here only so this module runs
        # even before you add to schema/resolver.
        return ApproachTunables(
            # Motion gating
            min_rotate_deg=float(_cfg(config, "min_rotate_deg", 2.0)),
            max_rotate_deg=float(_cfg(config, "max_rotate_deg", 90.0)),
            min_drive_mm=float(_cfg(config, "min_drive_mm", 5.0)),
            max_drive_mm=float(_cfg(config, "max_drive_mm", 2500.0)),

            # Vision / timing
            camera_settle_time_s=float(_cfg(config, "camera_settle_time", 0.30)),
            vision_loss_timeout_s=float(_cfg(config, "vision_loss_timeout_s", 0.50)),
            visible_max_age_s=float(_cfg(config, "visible_max_age_s", 0.35)),

            # Recovery sweep
            recover_step_deg=float(_cfg(config, "recover_step_deg", 15.0)),
            recover_max_sweep_deg=float(_cfg(config, "recover_max_sweep_deg", 180.0)),

            # LOW geometry (normal)
            final_commit_distance_mm=float(_cfg(config, "final_commit_distance_mm", 650.0)),
            final_approach_direct_range_mm=float(_cfg(config, "final_approach_direct_range_mm", 500.0)),
            final_approach_marker_push_mm=float(_cfg(config, "final_approach_marker_push", 50.0)),

            # HIGH geometry (high shelf)
            final_commit_distance_high_mm=float(_cfg(config, "final_commit_distance_high_mm", 1200.0)),
            final_approach_direct_range_high_mm=float(_cfg(config, "final_approach_direct_range_high_mm", 800.0)),
            final_approach_max_degree_high=float(_cfg(config, "final_approach_max_degree_high", 10.0)),

            # Height-model update gating
            marker_height_max_distance_mm=float(_cfg(config, "marker_height_max_distance_mm", 2000.0)),
            marker_pitch_high_deg=float(_cfg(config, "marker_pitch_high_deg", 0.35)),
            marker_pitch_low_deg=float(_cfg(config, "marker_pitch_low_deg", 0.10)),
        )

    @staticmethod
    def _band_label(distance_mm: float, commit_mm: float, direct_mm: float) -> str:
        if distance_mm <= commit_mm:
            return "A"
        elif distance_mm <= commit_mm + direct_mm:
            return "B"
        else:
            return "C"

    def _geometry_params(self) -> tuple[str, float, float]:
        """
        Returns: (mode_label, commit_mm, direct_mm)
          - mode_label in {"LOW","HIGH"} for logging/debug
        """
        if self.height_model.is_committed() and self.height_model.is_high():
            return ("HIGH", float(self.t.final_commit_distance_high_mm), float(self.t.final_approach_direct_range_high_mm))
        return ("LOW", float(self.t.final_commit_distance_mm), float(self.t.final_approach_direct_range_mm))

    def start(self, *, motion_backend, seed_target=None, **_):
        # IMPORTANT: do NOT rotate here. Caller already did ALIGN.
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
        self.target_id = int(seed_target.get("id")) if seed_target else None

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
                    self.last_action = None
                    return PrimitiveStatus.RUNNING

                if self.last_action == "rotate":
                    # after rotate, do the drive
                    if self.final_commit:
                        drive_mm = float(self.cached_distance or self.t.min_drive_mm)
                    else:
                        drive_mm = max(
                            self.t.min_drive_mm,
                            min(self.t.max_drive_mm, float(self.cached_distance or self.t.min_drive_mm)),
                        )

                    print(f"[MOTION][DRIVE] distance={drive_mm:.0f}mm")
                    self.last_drive_step = drive_mm

                    self.active_primitive = Drive(distance_mm=drive_mm)
                    self.last_action = "drive"
                    self.active_primitive.start(motion_backend=motion_backend)
                    return PrimitiveStatus.RUNNING

                if self.last_action == "drive":
                    # Post-drive bookkeeping
                    mode, commit, direct = self._geometry_params()
                    if self.last_seen_distance is not None and self.last_drive_step is not None:
                        after_est = max(0.0, self.last_seen_distance - self.last_drive_step)
                        band_after = self._band_label(after_est, commit, direct)
                        print(
                            "[APPROACH][POS] "
                            f"mode={mode} "
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
                    self.settle_until = now + self.t.camera_settle_time_s
                    self.bearing_consumed = False
                    self.cached_bearing = None

                    print(f"[APPROACH][SETTLE] start {self.t.camera_settle_time_s:.2f}s")
                    return PrimitiveStatus.RUNNING

            elif prim_status == PrimitiveStatus.FAILED:
                print(f"[APPROACH][{(self.last_action or 'UNKNOWN').upper()}] FAILED")

                # If we fail to reacquire, bubble up a failure to let caller decide retry/abort
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
        # 2a) Prefer the originally-selected id if we have one, but ONLY if it's actually visible.
        target = None
        if self.target_id is not None:
            visible = get_visible_targets(
                perception,
                self.kind,
                now=now,
                max_age_s=self.t.visible_max_age_s,
            )
            for t in visible:
                if int(t.get("id", -1)) == int(self.target_id):
                    target = t
                    break

        # 2b) fallback: closest of this kind (your util may consult memory/age)
        if target is None:
            target = get_closest_target(
                perception,
                self.kind,
                now=now,
                max_age_s=self.t.vision_loss_timeout_s,
            )

        age = (now - self.last_seen_time) if self.last_seen_time is not None else 0.0
        print(
            f"[APPROACH][DEBUG] kind={self.kind} "
            f"target={'YES' if target else 'NO'} "
            f"last_seen_age(before_update)={age:.2f}s"
        )

        mode, commit, direct = self._geometry_params()

        if target is not None:
            distance = float(target["distance"])
            bearing = float(target["bearing"])

            self.last_seen_time = now
            self.last_seen_distance = distance
            self.last_seen_bearing = bearing

            band = self._band_label(distance, commit, direct)

            print(
                "[APPROACH][SENSE] "
                f"vision=VISIBLE mode={mode} band={band} "
                f"dist={distance:.0f}mm bearing={bearing:.1f}°"
            )
        else:
            # Vision lost — decide blind-continue (rare) or reacquire

            # “high object” blind allowance (ported) — if high and in band B,
            # allow blind-to-commit in a small step (this will often lose contact).
            if (
                self.height_model.is_committed()
                and self.height_model.is_high()
                and self.last_seen_distance is not None
                and (commit < self.last_seen_distance <= commit + direct)
            ):
                blind_to_commit = self.last_seen_distance - commit
                blind_to_commit = max(self.t.min_drive_mm, min(self.t.max_drive_mm, blind_to_commit))

                print(
                    "[APPROACH][BLIND] "
                    f"vision=LOST mode=HIGH band=B "
                    f"last_seen={self.last_seen_distance:.0f}mm "
                    f"drive_to_commit={blind_to_commit:.0f}mm"
                )

                self.cached_bearing = 0.0
                self.cached_distance = blind_to_commit
                self.last_action = "rotate"
                self.bearing_consumed = True

                # Rotate 0 to enter "rotate->drive" path without changing heading
                self.active_primitive = Rotate(angle_deg=0.0)
                self.active_primitive.start(motion_backend=motion_backend)
                return PrimitiveStatus.RUNNING

            # Otherwise: actively try to reacquire
            elapsed = now - (self.last_seen_time or now)
            print(f"[APPROACH][SENSE] vision=LOST intent=REACQUIRE t={elapsed:.2f}s")

            self.active_primitive = ReacquireTarget(
                kind=self.kind,
                step_deg=self.t.recover_step_deg,
                max_sweep_deg=self.t.recover_max_sweep_deg,
                max_age_s=self.t.vision_loss_timeout_s,
            )
            self.last_action = "reacquire"
            self.active_primitive.start(motion_backend=motion_backend)
            return PrimitiveStatus.RUNNING

        # Have a target
        tid = int(target.get("id", -1))
        if self.target_id is None and tid >= 0:
            self.target_id = tid

        if not self.approach_started:
            self.approach_started = True
            print("[APPROACH] approach accepted — locking target id")

        print(
            f"[APPROACH][TARGET] kind={self.kind} id={tid if tid >= 0 else 'REL'} "
            f"dist={float(target['distance']):.0f}mm bearing={float(target['bearing']):.1f}°"
        )

        distance = float(target["distance"])
        bearing = float(target["bearing"])
        self.last_seen_time = now
        self.last_seen_distance = distance

        # Height model update/commit (ported)
        if distance <= self.t.marker_height_max_distance_mm and not self.height_model.is_committed():
            pitch = target["marker"].orientation.pitch
            self.height_model.update(pitch_deg=pitch)
            committed = self.height_model.try_commit(
                high_thresh=self.t.marker_pitch_high_deg,
                low_thresh=self.t.marker_pitch_low_deg,
            )
            if committed:
                print(
                    f"[HEIGHT] committed {'HIGH' if self.height_model.is_high() else 'LOW'} "
                    f"(pitch={pitch:.3f})"
                )

        # Refresh geometry after potential height commit
        mode, commit, direct = self._geometry_params()

        # Optional: policy hook for HIGH approach angle constraint.
        # We do NOT "solve wall angle" here; we just gate based on marker bearing.
        # If you want wall-relative geometry, that belongs in a separate module.
        if mode == "HIGH" and abs(bearing) > float(self.t.final_approach_max_degree_high):
            print(
                "[APPROACH][HIGH_POLICY] "
                f"bearing={bearing:.1f}° exceeds final_approach_max_degree_high={self.t.final_approach_max_degree_high:.1f}° "
                "— waiting / caller should invoke wall-geometry approach"
            )
            return PrimitiveStatus.RUNNING

        # Final commit trigger
        if (not self.final_commit) and (distance <= commit) and self.height_model.is_committed():
            self.final_commit = True
            self.target_is_high = self.height_model.is_high()

            # "marker push" replaces old hardcoded "+ 70"
            final_drive_mm = distance + float(self.t.final_approach_marker_push_mm)
            print(
                f"[APPROACH][FINAL] mode={mode} "
                f"commit dist={distance:.0f}mm final_drive={final_drive_mm:.0f}mm"
            )

            self.cached_distance = final_drive_mm
            self.cached_bearing = bearing
            self.last_action = "rotate"

            angle = max(-self.t.max_rotate_deg, min(self.t.max_rotate_deg, self.cached_bearing))

            self.active_primitive = AlignToTarget(
                bearing_deg=angle,
                tolerance_deg=0.0,
                max_rotate_deg=self.t.max_rotate_deg,
            )
            self.active_primitive.start(motion_backend=motion_backend)
            return PrimitiveStatus.RUNNING

        # If bearing already good, skip rotate -> drive
        if self.bearing_consumed:
            return PrimitiveStatus.RUNNING

        # If bearing already within tolerance, drive without rotating
        if abs(bearing) < self.t.min_rotate_deg:
            print("[APPROACH] bearing within tolerance — skipping rotate, planning drive")

            if distance > commit + direct:
                remaining_to_commit = distance - commit
                step = max(self.t.min_drive_mm, min((remaining_to_commit / 2.0), self.t.max_drive_mm))
                self.cached_distance = step
            elif distance > commit:
                self.cached_distance = distance - commit
            else:
                self.cached_distance = self.t.min_drive_mm

            drive_mm = max(self.t.min_drive_mm, min(self.t.max_drive_mm, float(self.cached_distance)))
            print(f"[MOTION][DRIVE] distance={drive_mm:.0f}mm (no-rotate path)")

            self.last_drive_step = drive_mm
            self.active_primitive = Drive(distance_mm=drive_mm)
            self.last_action = "drive"
            self.active_primitive.start(motion_backend=motion_backend)
            return PrimitiveStatus.RUNNING

        # Otherwise rotate then drive
        self.cached_bearing = bearing
        angle = max(-self.t.max_rotate_deg, min(self.t.max_rotate_deg, self.cached_bearing))

        print(f"[ROTATE-DEBUG] bearing={bearing:.2f} angle_cmd={angle:.2f}")
        print(f"[APPROACH][RANGES] mode={mode} commit={commit:.0f}mm direct={direct:.0f}mm")

        if distance > commit + direct:
            remaining_to_commit = distance - commit
            step = remaining_to_commit / 2.0
            step = max(self.t.min_drive_mm, min(step, self.t.max_drive_mm))
            self.cached_distance = step
            print(
                "[APPROACH][PLAN] band=C vision=VISIBLE "
                f"mode={mode} dist={distance:.0f}mm intent=HALF_TO_COMMIT drive_target={step:.0f}mm"
            )
        elif distance > commit:
            self.cached_distance = distance - commit
            print(
                "[APPROACH][PLAN] band=B vision=VISIBLE "
                f"mode={mode} dist={distance:.0f}mm intent=DIRECT_TO_COMMIT drive_target={self.cached_distance:.0f}mm"
            )
        else:
            # already inside commit band but height model not committed -> wait for another frame
            return PrimitiveStatus.RUNNING

        self.last_action = "rotate"
        self.bearing_consumed = True

        print(f"[MOTION][ROTATE] angle={angle:.1f}° (bearing={bearing:.1f}°)")

        self.active_primitive = AlignToTarget(
            bearing_deg=angle,
            tolerance_deg=0.0,
            max_rotate_deg=self.t.max_rotate_deg,
        )
        self.active_primitive.start(motion_backend=motion_backend)
        return PrimitiveStatus.RUNNING

    def stop(self):
        if self.active_primitive is not None:
            self.active_primitive.stop()
        self.active_primitive = None
