# skills/navigation/approach_target.py

from __future__ import annotations

import math
import time
import statistics
from dataclasses import dataclass
from typing import Any, Optional

from navigation.dog_leg_side_step import DogLegSideStep, compute_dog_leg_plan

from primitives.base import Primitive, PrimitiveStatus
from primitives.motion import Drive, Rotate

from skills.navigation.align_to_target import AlignToTarget
from skills.navigation.parallel_to_wall import ParallelToWall  # NEW
from skills.perception.reacquire_target import ReacquireTarget
from skills.perception.select_target_utils import get_closest_target

# perception.py defines this helper at module level
from perception import get_visible_targets


def _cfg(config: Any, name: str, fallback: Any) -> Any:
    """Read config field if present; else fallback."""
    return getattr(config, name, fallback)


def _to_rad(v: float) -> tuple[float, str]:
    """
    Heuristic unit fix:
      - if magnitude looks like degrees (eg 4.5, 10, 20), convert to radians
      - if magnitude already small (<= ~1.3), assume radians
    """
    v = float(v)
    if abs(v) > 1.3:
        return math.radians(v), "deg->rad"
    return v, "rad"


def _get_dict_path(d: dict, *path: str) -> Any:
    cur: Any = d
    for k in path:
        if not isinstance(cur, dict) or k not in cur:
            return None
        cur = cur[k]
    return cur


def _marker_elevation(marker, *, img_h: int, fov_y_rad: float) -> tuple[float, str]:
    """
    Pose-free elevation cue.

    Priority order:
      1) position.vertical_angle (pose-free, from detector)  [object or dict]
      2) orientation.pitch       (pose-full-ish but sometimes available) [object or dict]
      3) pixel-centre/corners fallback (pose-free-ish)

    Returns (pitch_rad, src_label)
    """

    # -----------------------------
    # 1) BEST: position.vertical_angle
    # -----------------------------
    pos = getattr(marker, "position", None)
    va = getattr(pos, "vertical_angle", None) if pos is not None else None
    if va is not None:
        try:
            pitch, unit = _to_rad(va)
            return pitch, f"position.vertical_angle({unit})"
        except (TypeError, ValueError):
            pass

    if isinstance(marker, dict):
        for key_path, label in (
            (("position", "vertical_angle"), "marker['position']['vertical_angle']"),
            (("position_vertical_angle",), "marker['position_vertical_angle']"),
            (("vertical_angle",), "marker['vertical_angle']"),
        ):
            va2 = _get_dict_path(marker, *key_path)
            if va2 is not None:
                try:
                    pitch, unit = _to_rad(va2)
                    return pitch, f"{label}({unit})"
                except (TypeError, ValueError):
                    pass

    # -----------------------------
    # 2) Next best: orientation.pitch
    # -----------------------------
    ori = getattr(marker, "orientation", None)
    op = getattr(ori, "pitch", None) if ori is not None else None
    if op is not None:
        try:
            pitch, unit = _to_rad(op)
            return pitch, f"orientation.pitch({unit})"
        except (TypeError, ValueError):
            pass

    if isinstance(marker, dict):
        for key_path, label in (
            (("orientation", "pitch"), "marker['orientation']['pitch']"),
            (("pitch",), "marker['pitch']"),
        ):
            op2 = _get_dict_path(marker, *key_path)
            if op2 is not None:
                try:
                    pitch, unit = _to_rad(op2)
                    return pitch, f"{label}({unit})"
                except (TypeError, ValueError):
                    pass

    # -----------------------------
    # 3) Fallback: derive from image y coordinate
    # -----------------------------
    for name in ("centre", "center", "centroid"):
        c = getattr(marker, name, None)
        if c is not None:
            if hasattr(c, "__len__") and len(c) >= 2:
                y_px = float(c[1])
                y_norm = max(0.0, min(1.0, y_px / float(img_h)))
                pitch = (0.5 - y_norm) * fov_y_rad
                return pitch, f"{name}(y)"

            y = getattr(c, "y", None)
            if y is not None:
                y_px = float(y)
                y_norm = max(0.0, min(1.0, y_px / float(img_h)))
                pitch = (0.5 - y_norm) * fov_y_rad
                return pitch, f"{name}.y"

    corners = getattr(marker, "corners", None)
    if corners is not None and len(corners) >= 4:
        ys = []
        for p in corners:
            if hasattr(p, "__len__") and len(p) >= 2:
                ys.append(float(p[1]))
            else:
                py = getattr(p, "y", None)
                if py is not None:
                    ys.append(float(py))
        if ys:
            y_px = sum(ys) / len(ys)
            y_norm = max(0.0, min(1.0, y_px / float(img_h)))
            pitch = (0.5 - y_norm) * fov_y_rad
            return pitch, "corners(avg_y)"

    return 0.0, "none"


def _debug_dump_visible_vertical_angles(
    perception,
    *,
    kind: str,
    now: float,
    max_age_s: float,
    img_h: int,
    fov_y_deg: float,
) -> None:
    """
    Debug: print distance + vertical angle for every visible marker of this kind.
    Uses the same elevation extractor used by HeightModel (_marker_elevation).
    """
    visible = get_visible_targets(perception, kind, now=now, max_age_s=max_age_s)
    if not visible:
        print(f"[VA][ALL] kind={kind} visible=0")
        return

    # Sort closest-first so the print is stable and easy to read.
    visible_sorted = sorted(visible, key=lambda t: float(t.get("distance", 1e9)))

    parts = []
    fov_y_rad = math.radians(float(fov_y_deg))

    for t in visible_sorted:
        tid = t.get("id", "REL")
        dist = float(t.get("distance", 0.0))
        bearing = float(t.get("bearing", 0.0))

        pitch, src = _marker_elevation(
            t.get("marker", t),
            img_h=img_h,
            fov_y_rad=fov_y_rad,
        )

        parts.append(
            f"id={tid} d={dist:.0f} b={bearing:+.1f} "
            f"va={pitch:+.4f}rad({math.degrees(pitch):+.2f}deg) src={src}"
        )

    print("[VA][ALL] " + " | ".join(parts))


@dataclass(frozen=True)
class ApproachTunables:
    # Motion gating
    min_rotate_deg: float
    max_rotate_deg: float
    min_drive_mm: float
    max_drive_mm: float
    camera_image_height_px: int
    camera_fov_y_deg: float
    height_decision_deadline_mm: float

    # Vision / timing
    camera_settle_time_s: float
    camera_fresh_obs_max_age_s: float
    vision_loss_timeout_s: float
    visible_max_age_s: float

    # Recovery sweep
    recover_step_deg: float
    recover_max_sweep_deg: float

    # LOW geometry
    final_commit_distance_mm: float
    final_approach_direct_range_mm: float
    final_approach_marker_push_mm: float

    # HIGH geometry
    final_commit_distance_high_mm: float
    final_approach_direct_range_high_mm: float
    final_approach_max_degree_high: float

    # Height-model update gating (thresholds are treated as radians in HeightModel)
    marker_height_max_distance_mm: float
    marker_pitch_high_deg: float
    marker_pitch_low_deg: float


class ApproachTarget(Primitive):
    def __init__(self, *, config, kind: str, height_model, locked_target_id: Optional[int] = None):
        super().__init__()
        self.config = config
        self.kind = kind
        self.height_model = height_model
        self.locked_target_id: Optional[int] = int(locked_target_id) if locked_target_id is not None else None

        self.t = self._load_tunables(config)

        # Approach loop state
        self.active_primitive: Optional[Primitive] = None
        self.last_action: Optional[str] = None  # "rotate" / "drive" / "reacquire" / "parallel" / "dogleg"
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

        # Parallel-to-wall helper state
        self._parallel_skill: Optional[ParallelToWall] = None

        self._dogleg_cooldown_until: Optional[float] = None

        # After a motion+settle cycle, require a fresh camera observation
        # before replanning. This prevents stale last_seen_* state from
        # causing repeated blind rotate/drive loops.
        self._require_fresh_obs_after_settle: bool = False
        self._fresh_obs_wait_started: Optional[float] = None

    @property
    def approached_target_id(self) -> Optional[int]:
        return self.target_id

    def _load_tunables(self, config: Any) -> ApproachTunables:
        return ApproachTunables(
            min_rotate_deg=float(_cfg(config, "min_rotate_deg", 2.0)),
            max_rotate_deg=float(_cfg(config, "max_rotate_deg", 90.0)),
            min_drive_mm=float(_cfg(config, "min_drive_mm", 5.0)),
            max_drive_mm=float(_cfg(config, "max_drive_mm", 2500.0)),
            camera_settle_time_s=float(_cfg(config, "camera_settle_time", 0.30)),
            camera_fresh_obs_max_age_s=float(_cfg(config, "camera_fresh_obs_max_age_s", 0.12)),
            vision_loss_timeout_s=float(_cfg(config, "vision_loss_timeout_s", 0.50)),
            visible_max_age_s=float(_cfg(config, "visible_max_age_s", 0.35)),
            recover_step_deg=float(_cfg(config, "recover_step_deg", 15.0)),
            recover_max_sweep_deg=float(_cfg(config, "recover_max_sweep_deg", 180.0)),
            final_commit_distance_mm=float(_cfg(config, "final_commit_distance_mm", 650.0)),
            final_approach_direct_range_mm=float(_cfg(config, "final_approach_direct_range_mm", 500.0)),
            final_approach_marker_push_mm=float(_cfg(config, "final_approach_marker_push", 50.0)),
            final_commit_distance_high_mm=float(_cfg(config, "final_commit_distance_high_mm", 1200.0)),
            final_approach_direct_range_high_mm=float(_cfg(config, "final_approach_direct_range_high_mm", 800.0)),
            final_approach_max_degree_high=float(_cfg(config, "final_approach_max_degree_high", 10.0)),
            marker_height_max_distance_mm=float(_cfg(config, "marker_height_max_distance_mm", 2000.0)),
            marker_pitch_high_deg=float(_cfg(config, "marker_pitch_high_deg", 0.35)),
            marker_pitch_low_deg=float(_cfg(config, "marker_pitch_low_deg", 0.10)),
            camera_image_height_px=int(_cfg(config, "camera_image_height_px", 720)),
            camera_fov_y_deg=float(_cfg(config, "camera_fov_y_deg", 49.0)),
            height_decision_deadline_mm=float(_cfg(config, "height_decision_deadline_mm", 1100.0)),
        )

    @staticmethod
    def _band_label(distance_mm: float, commit_mm: float, direct_mm: float) -> str:
        if distance_mm <= commit_mm:
            return "A"
        if distance_mm <= commit_mm + direct_mm:
            return "B"
        return "C"

    def _geometry_params(self) -> tuple[str, float, float]:
        if self.height_model.is_committed() and self.height_model.is_high():
            return (
                "HIGH",
                float(self.t.final_commit_distance_high_mm),
                float(self.t.final_approach_direct_range_high_mm),
            )
        return ("LOW", float(self.t.final_commit_distance_mm), float(self.t.final_approach_direct_range_mm))

    def start(self, *, motion_backend, seed_target=None, **_):
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

        if self.locked_target_id is not None:
            self.target_id = int(self.locked_target_id)
        else:
            self.target_id = int(seed_target.get("id")) if seed_target else None

        if seed_target is not None:
            self.last_seen_time = now
            self.last_seen_distance = float(seed_target.get("distance", 0.0))
            self.last_seen_bearing = float(seed_target.get("bearing", 0.0))
        else:
            self.last_seen_time = None
            self.last_seen_distance = None
            self.last_seen_bearing = None

        self._parallel_skill = None
        self._require_fresh_obs_after_settle = False
        self._fresh_obs_wait_started = None

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
                self.cached_bearing = None
                self._require_fresh_obs_after_settle = True
                self._fresh_obs_wait_started = now
                print(
                    "[APPROACH][VISION] post-settle fresh observation required "
                    f"(max_age={self.t.camera_fresh_obs_max_age_s:.2f}s)"
                )

        # =================================================
        # SECTION 1 — ACTIVE PRIMITIVE (NO REASSESSMENT)
        # =================================================
        if self.active_primitive is not None:
            prim = self.active_primitive

            if isinstance(prim, ReacquireTarget):
                prim_status = prim.update(motion_backend=motion_backend, perception=perception)
            elif isinstance(prim, ParallelToWall):
                prim_status = prim.update(motion_backend=motion_backend, perception=perception)
            elif isinstance(prim, (Drive, Rotate)):
                prim_status = prim.update(motion_backend=motion_backend)
            else:
                prim_status = prim.update(motion_backend=motion_backend)

            if prim_status == PrimitiveStatus.SUCCEEDED:
                action = self.last_action or "UNKNOWN"
                print(f"[APPROACH][{action.upper()}] complete")

                self.active_primitive = None

                if self.last_action == "reacquire":
                    self.last_action = None
                    return PrimitiveStatus.RUNNING

                if self.last_action == "parallel":
                    self.last_action = None
                    self._parallel_skill = None
                    self.cached_distance = None
                    self.cached_bearing = None
                    self.bearing_consumed = False
                    return PrimitiveStatus.RUNNING

                if self.last_action == "dogleg":
                    self.last_action = None
                    self.cached_distance = None
                    self.cached_bearing = None
                    self.bearing_consumed = False
                    self.settle_until = now + self.t.camera_settle_time_s
                    print(f"[APPROACH][SETTLE] start {self.t.camera_settle_time_s:.2f}s (after dogleg)")
                    return PrimitiveStatus.RUNNING

                if self.last_action == "rotate":
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

                    if self.final_commit:
                        print("[APPROACH][FINAL] drive complete — positioned for grab")
                        return PrimitiveStatus.SUCCEEDED

                    self.settle_until = now + self.t.camera_settle_time_s
                    self.bearing_consumed = False
                    self.cached_bearing = None
                    print(f"[APPROACH][SETTLE] start {self.t.camera_settle_time_s:.2f}s")
                    return PrimitiveStatus.RUNNING

            elif prim_status == PrimitiveStatus.FAILED:
                print(f"[APPROACH][{(self.last_action or 'UNKNOWN').upper()}] FAILED")

                if self.last_action == "reacquire":
                    self.active_primitive = None
                    self.last_action = None
                    self.cached_distance = None
                    self.cached_bearing = None
                    self.bearing_consumed = False
                    self.last_drive_step = None
                    self.settle_until = None
                    self._parallel_skill = None
                    return PrimitiveStatus.FAILED

                self.active_primitive = None
                self.last_action = None
                self.cached_distance = None
                self.cached_bearing = None
                self._parallel_skill = None
                return PrimitiveStatus.FAILED

            return PrimitiveStatus.RUNNING

        # =================================================
        # SECTION 2 — REASSESS TARGET (ONLY THINKING POINT)
        # =================================================
        target = None

        # -------------------------------------------------
        # POST-SETTLE FRESH-VISION GATE
        # -------------------------------------------------
        # After a rotate/drive + settle cycle, do not immediately continue
        # from stale last_seen_* memory. Require a freshly visible target
        # before replanning, unless we are already in the true final commit.
        if self._require_fresh_obs_after_settle and not self.final_commit:
            fresh_target = None

            visible_fresh = get_visible_targets(
                perception,
                self.kind,
                now=now,
                max_age_s=self.t.camera_fresh_obs_max_age_s,
            )

            if self.target_id is not None:
                for t in visible_fresh:
                    if int(t.get("id", -1)) == int(self.target_id):
                        fresh_target = t
                        break
            elif visible_fresh:
                fresh_target = get_closest_target(
                    perception,
                    self.kind,
                    now=now,
                    max_age_s=self.t.camera_fresh_obs_max_age_s,
                )

            if fresh_target is None:
                waited = now - (self._fresh_obs_wait_started or now)
                print(
                    "[APPROACH][VISION] waiting for fresh post-settle obs "
                    f"t={waited:.2f}s max_age={self.t.camera_fresh_obs_max_age_s:.2f}s"
                )

                if waited >= self.t.vision_loss_timeout_s:
                    print(
                        "[APPROACH][VISION] no fresh post-settle obs "
                        f"within {self.t.vision_loss_timeout_s:.2f}s -> REACQUIRE"
                    )
                    self._require_fresh_obs_after_settle = False
                    self._fresh_obs_wait_started = None

                    self.active_primitive = ReacquireTarget(
                        kind=self.kind,
                        step_deg=self.t.recover_step_deg,
                        max_sweep_deg=self.t.recover_max_sweep_deg,
                        max_age_s=self.t.vision_loss_timeout_s,
                        target_id=self.target_id,
                    )
                    self.last_action = "reacquire"
                    self.active_primitive.start(motion_backend=motion_backend)
                    return PrimitiveStatus.RUNNING

                return PrimitiveStatus.RUNNING

            self._require_fresh_obs_after_settle = False
            self._fresh_obs_wait_started = None
            target = fresh_target

            print(
                "[APPROACH][VISION] fresh post-settle obs accepted "
                f"id={int(target.get('id', -1)) if target.get('id', None) is not None else 'REL'} "
                f"dist={float(target['distance']):.0f}mm "
                f"bearing={float(target['bearing']):.1f}°"
            )

        # Debug: dump vertical angles for ALL visible markers every reassessment tick
        _debug_dump_visible_vertical_angles(
            perception,
            kind=self.kind,
            now=now,
            max_age_s=self.t.visible_max_age_s,
            img_h=self.t.camera_image_height_px,
            fov_y_deg=self.t.camera_fov_y_deg,
        )

        # If the fresh-post-settle gate already supplied a target, keep it.
        if target is None:
            # Prefer locked id if actually visible
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

            # If locked to an id, do NOT substitute another
            if target is None and self.target_id is not None:
                pass
            elif target is None:
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
            # Vision lost: optionally blind-to-commit for HIGH band B, else reacquire
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

                self.active_primitive = Rotate(angle_deg=0.0)
                self.active_primitive.start(motion_backend=motion_backend)
                return PrimitiveStatus.RUNNING

            elapsed = now - (self.last_seen_time or now)
            print(f"[APPROACH][SENSE] vision=LOST intent=REACQUIRE t={elapsed:.2f}s")

            self.active_primitive = ReacquireTarget(
                kind=self.kind,
                step_deg=self.t.recover_step_deg,
                max_sweep_deg=self.t.recover_max_sweep_deg,
                max_age_s=self.t.vision_loss_timeout_s,
                target_id=self.target_id,
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

        # =================================================
        # HEIGHT MODEL UPDATE/COMMIT (POSE-FREE)
        # =================================================
        if distance <= self.t.marker_height_max_distance_mm and not self.height_model.is_committed():
            pitch, src = _marker_elevation(
                target.get("marker", target),  # unwrap marker object if present
                img_h=self.t.camera_image_height_px,
                fov_y_rad=math.radians(self.t.camera_fov_y_deg),
            )

            print(f"[HEIGHT][RAW] src={src} pitch={pitch:.3f}")

            if src != "none":
                self.height_model.update(
                    pitch_rad=pitch,  # radians
                    distance_mm=distance,
                    high_thresh=float(self.t.marker_pitch_high_deg),
                    low_thresh=float(self.t.marker_pitch_low_deg),
                )

            decision = self.height_model.try_commit(
                distance_mm=distance,
                high_thresh=float(self.t.marker_pitch_high_deg),
                low_thresh=float(self.t.marker_pitch_low_deg),
                decision_deadline_mm=float(self.t.height_decision_deadline_mm),
            )
            if decision.committed:
                print(
                    f"[HEIGHT] committed {'HIGH' if self.height_model.is_high() else 'LOW'} "
                    f"reason={decision.reason} score={self.height_model.score:.2f} "
                    f"max_pitch={self.height_model.max_pitch:.3f} samples={self.height_model.samples}"
                )

        # Refresh geometry after potential height commit
        mode, commit, direct = self._geometry_params()

        # --------------------------------------------------
        # DOGLEG policy: Band B (both HIGH and LOW)
        # --------------------------------------------------
        band_now = self._band_label(distance, commit, direct)

        DOGLEG_TRIGGER_DEG = 12.0
        DOGLEG_COOLDOWN_S = 1.0

        if (not self.final_commit) and band_now == "B" and abs(bearing) >= DOGLEG_TRIGGER_DEG:
            if self._dogleg_cooldown_until is None or now >= self._dogleg_cooldown_until:
                # Optional clamp so we don't sidestep huge distances
                plan = compute_dog_leg_plan(
                    distance_mm=distance,
                    bearing_deg=bearing,
                    min_sidestep_mm=80.0,
                    max_drive_mm=600.0,
                )

                if plan.drive_mm > 0.0:
                    print(
                        "[APPROACH][DOGLEG] "
                        f"mode={mode} band=B dist={distance:.0f}mm bearing={bearing:.1f}° "
                        f"rot1={plan.rotate1_deg:+.0f} drive={plan.drive_mm:.0f} rot2={plan.rotate2_deg:+.0f}"
                    )
                    self.active_primitive = DogLegSideStep(distance_mm=distance, bearing_deg=bearing)
                    self.last_action = "dogleg"
                    self.active_primitive.start(motion_backend=motion_backend)
                    self._dogleg_cooldown_until = now + DOGLEG_COOLDOWN_S
                    return PrimitiveStatus.RUNNING

        # --------------------------------------------------
        # HIGH policy: if too angled in A/B, run ParallelToWall.
        # --------------------------------------------------
        if mode == "HIGH" and abs(bearing) > float(self.t.final_approach_max_degree_high):
            band_now = self._band_label(distance, commit, direct)

            if band_now in ("A", "B"):
                print(
                    "[APPROACH][HIGH_POLICY] "
                    f"band={band_now} bearing={bearing:.1f}° exceeds "
                    f"final_approach_max_degree_high={self.t.final_approach_max_degree_high:.1f}° "
                    "— invoking ParallelToWall"
                )

                if self._parallel_skill is None:
                    self._parallel_skill = ParallelToWall(config=self.config)
                    self._parallel_skill.start()

                self.active_primitive = self._parallel_skill
                self.last_action = "parallel"
                return PrimitiveStatus.RUNNING

            print(
                "[APPROACH][HIGH_POLICY] "
                f"band={band_now} bearing={bearing:.1f}° exceeds max, but in band C — skipping parallel-to-wall until closer"
            )

        # Final commit trigger
        if (not self.final_commit) and (distance <= commit) and self.height_model.is_committed():
            self.final_commit = True
            self.target_is_high = self.height_model.is_high()
            self._require_fresh_obs_after_settle = False
            self._fresh_obs_wait_started = None

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

        # If bearing within tolerance, drive without rotating
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
            # inside commit band but height model not committed -> wait for another frame
            return PrimitiveStatus.RUNNING

        self.last_action = "rotate"
        self.bearing_consumed = True

        print(
            f"[MOTION][ROTATE] angle={angle:.1f}° "
            f"(bearing={bearing:.1f}° target_id={self.target_id} mode={mode})"
        )

        if self.last_seen_bearing is not None:
            print(
                f"[APPROACH][ROTATE_CTX] "
                f"last_seen_bearing={self.last_seen_bearing:.2f} "
                f"current_bearing={bearing:.2f} "
                f"distance={distance:.0f} "
                f"commit={commit:.0f} direct={direct:.0f}"
            )

        self.active_primitive = AlignToTarget(
            bearing_deg=angle,
            tolerance_deg=0.0,
            max_rotate_deg=self.t.max_rotate_deg,
        )
        self.active_primitive.start(motion_backend=motion_backend)
        return PrimitiveStatus.RUNNING

    def _safe_stop(self, prim, *, motion_backend=None):
        if prim is None:
            return
        try:
            prim.stop(motion_backend=motion_backend) if motion_backend is not None else prim.stop()
        except TypeError:
            # stop() doesn't accept motion_backend
            try:
                prim.stop()
            except Exception:
                pass
        except Exception:
            pass

    def stop(self, *, motion_backend=None):
        self._safe_stop(self.active_primitive, motion_backend=motion_backend)
        self.active_primitive = None
        self._parallel_skill = None

