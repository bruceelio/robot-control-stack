# behaviors/recover_lost_target.py

from __future__ import annotations

import time
import inspect
from dataclasses import dataclass
from typing import Optional, Any, Literal

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus
from log_trace import trace


from skills.perception.reacquire_target import ReacquireTarget
from skills.navigation.backoff_scan import BackoffScan
from skills.navigation.search_rotate import SearchRotate


RecoverOutcome = Literal[
    "LOCKED_RECOVERED",        # reacquired the same locked id (or regained it after backoff)
    "NEW_TARGET_FOUND",        # found something else during SearchRotate (lock was dropped)
    "POSE_OBTAINED_NO_TARGET", # pose was seen at least once during the scan ladder, but no target was found
    "FAILED",                  # nothing recovered
]


@dataclass
class RecoverLostTargetResult:
    outcome: RecoverOutcome
    # If a caller wants to use this directly it can, but recommended flow is:
    #   outcome -> funnel back to SELECT (if NEW_TARGET_FOUND) or TRACK (if LOCKED_RECOVERED)
    found_target_id: Optional[int] = None
    found_kind: Optional[str] = None

    # Opportunistic pose capture (last known good pose during recovery)
    pose_xy: Optional[tuple[float, float]] = None
    pose_heading: Optional[float] = None
    pose_time_s: Optional[float] = None


class RecoverLostTarget(Behavior):
    """
    Behavior: Recover from a LOST visual target.

    Ladder (budgeted, escalating):
      1) ReacquireTarget (locked-id only)
      2) BackoffScan (still locked-biased / viewpoint change)
      3) SearchRotate (UNLOCK + full 360 scan for anything)
         - while rotating, opportunistically "bank" the last-good pose if localisation becomes valid.

    NOTE:
      - This behavior does NOT directly call TrackObject.
      - It returns a result; the caller (AcquireObject) should funnel back through SELECT/TRACK.
    """

    def __init__(self):
        super().__init__()
        # Optional: set by caller (AcquireObject) so recovery trace lines share the same run id
        self.run_id: int = -1

        self.config = None
        self.kind = None

        self._banked_pose_time_s: Optional[float] = None
        self._last_pose_trace_xy: Optional[tuple[float, float]] = None

        self.locked_target_id: Optional[int] = None
        self.last_bearing_deg: float = 0.0
        self.last_distance_mm: Optional[float] = None

        self.phase: str = "REACQUIRE"  # REACQUIRE -> BACKOFF_SCAN -> SEARCH_ROTATE -> DONE
        self.result: RecoverLostTargetResult = RecoverLostTargetResult(outcome="FAILED")

        self._reacquire: Optional[ReacquireTarget] = None
        self._backoff: Optional[BackoffScan] = None
        self._search: Optional[SearchRotate] = None

        # Pose banking (updated opportunistically)
        self._banked_pose_xy: Optional[tuple[float, float]] = None
        self._banked_pose_heading: Optional[float] = None
        self._banked_pose_time_s: Optional[float] = None

    # -------------------------
    # Public API
    # -------------------------

    def start(
        self,
        *,
        config: Any,
        kind: str,
        locked_target_id: Optional[int],
        last_bearing_deg: float = 0.0,
        last_distance_mm: Optional[float] = None,
        **_,
    ):
        print("[RECOVER_LOST_TARGET] start")
        trace(
            src="RECOVER",
            evt="RECOVER_ENTER",
            phase="RECOVER_LOST_TARGET",
            run=self.run_id,
            kind=kind,
            lock=locked_target_id or "none",
            bear=last_bearing_deg,
            dist=last_distance_mm,
        )

        self.config = config
        self.kind = kind
        self.locked_target_id = locked_target_id
        self.last_bearing_deg = float(last_bearing_deg or 0.0)
        self.last_distance_mm = last_distance_mm

        self.phase = "REACQUIRE"
        self.result = RecoverLostTargetResult(outcome="FAILED")

        self._reacquire = None
        self._backoff = None
        self._search = None

        self._banked_pose_xy = None
        self._banked_pose_heading = None
        self._banked_pose_time_s = None

        self.status = BehaviorStatus.RUNNING
        return self.status

    def update(self, *, perception, localisation, motion_backend, **_):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        # Opportunistically bank pose on every tick we run (even before SearchRotate)
        self._maybe_bank_pose(localisation)

        if self.phase == "REACQUIRE":
            return self._update_reacquire(perception, motion_backend)

        if self.phase == "BACKOFF_SCAN":
            return self._update_backoff(perception, motion_backend)

        if self.phase == "SEARCH_ROTATE":
            return self._update_search_rotate(perception, localisation, motion_backend)

        # DONE (should not be reached while RUNNING)
        return self.status

    def stop(self, *, motion_backend=None, **_):
        # Best-effort stop of any active skill
        self._safe_stop(self._reacquire, motion_backend=motion_backend)
        self._safe_stop(self._backoff, motion_backend=motion_backend)
        self._safe_stop(self._search, motion_backend=motion_backend)

        self._reacquire = None
        self._backoff = None
        self._search = None

        self.status = BehaviorStatus.FAILED
        return self.status

    # -------------------------
    # Internals: ladder rungs
    # -------------------------

    def _update_reacquire(self, perception, motion_backend):
        # If we don't have a lock, skip straight to SearchRotate (unlock scan for anything)
        if self.locked_target_id is None:
            print("[RECOVER_LOST_TARGET][REACQUIRE] no lock -> SEARCH_ROTATE")

            trace(
                src="RECOVER",
                evt="RECOVER_RUNG_DONE",
                phase="REACQUIRE",
                run=self.run_id,
                result="SKIPPED",
                reason="no_lock",
            )
            trace(
                src="RECOVER",
                evt="RECOVER_RUNG_ENTER",
                phase="SEARCH_ROTATE",
                run=self.run_id,
                lock="none",
            )

            self.phase = "SEARCH_ROTATE"
            return self.status

        if self._reacquire is None:
            self._reacquire = self._make_reacquire_skill(
                target_id=self.locked_target_id,
                last_bearing=self.last_bearing_deg,
                last_distance=self.last_distance_mm,
            )
            trace(
                src="RECOVER",
                evt="RECOVER_RUNG_ENTER",
                phase="REACQUIRE",
                run=self.run_id,
                lock=self.locked_target_id or "none",
            )
            self._reacquire.start(motion_backend=motion_backend)

        st = self._reacquire.update(
            motion_backend=motion_backend,
            perception=perception,
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.SUCCEEDED:
            print("[RECOVER_LOST_TARGET][REACQUIRE] succeeded (locked recovered)")
            trace(
                src="RECOVER",
                evt="RECOVER_RUNG_DONE",
                phase="REACQUIRE",
                run=self.run_id,
                result="SUCCEEDED",
            )

            self._reacquire = None
            self._finish(
                outcome="LOCKED_RECOVERED",
                found_target_id=self.locked_target_id,
                found_kind=self.kind,
            )
            return self.status

        # FAILED
        print("[RECOVER_LOST_TARGET][REACQUIRE] failed -> BACKOFF_SCAN")

        trace(
            src="RECOVER",
            evt="RECOVER_RUNG_DONE",
            phase="REACQUIRE",
            run=self.run_id,
            result="FAILED",
        )
        trace(
            src="RECOVER",
            evt="RECOVER_RUNG_ENTER",
            phase="BACKOFF_SCAN",
            run=self.run_id,
            lock=self.locked_target_id or "none",
        )

        self._reacquire = None
        self.phase = "BACKOFF_SCAN"
        return self.status

    def _update_backoff(self, perception, motion_backend):
        if self._backoff is None:
            # BackoffScan implementation may be "locked biased" internally.
            # We do NOT clear the lock here; that only happens before SearchRotate rung.
            self._backoff = BackoffScan(
                kind=self.kind,
                label="RECOVER_LOST_TARGET][BACKOFF_SCAN",
            )
            trace(
                src="RECOVER",
                evt="RECOVER_RUNG_ENTER",
                phase="BACKOFF_SCAN",
                run=self.run_id,
                lock=self.locked_target_id or "none",
            )

            self._backoff.start(motion_backend=motion_backend)

        st = self._backoff.update(
            motion_backend=motion_backend,
            perception=perception,
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.SUCCEEDED:
            # Treat as "locked recovered" *if* we still have a locked id.
            # If caller passed no lock, this is simply "some recovery succeeded" but we
            # don't have a specific id to report, so we funnel back to SELECT by returning NEW_TARGET_FOUND.
            if self.locked_target_id is not None:
                print("[RECOVER_LOST_TARGET][BACKOFF_SCAN] succeeded (assume locked recovered)")
                trace(
                    src="RECOVER",
                    evt="RECOVER_RUNG_DONE",
                    phase="BACKOFF_SCAN",
                    run=self.run_id,
                    result="SUCCEEDED",
                )

                self._backoff = None
                self._finish(
                    outcome="LOCKED_RECOVERED",
                    found_target_id=self.locked_target_id,
                    found_kind=self.kind,
                )
                return self.status

            print("[RECOVER_LOST_TARGET][BACKOFF_SCAN] succeeded (no lock) -> NEW_TARGET_FOUND (select again)")
            trace(
                src="RECOVER",
                evt="RECOVER_RUNG_DONE",
                phase="BACKOFF_SCAN",
                run=self.run_id,
                result="SUCCEEDED",
                reason="no_lock",
            )

            self._backoff = None
            self._finish(outcome="NEW_TARGET_FOUND")
            return self.status

        # FAILED
        print("[RECOVER_LOST_TARGET][BACKOFF_SCAN] failed -> SEARCH_ROTATE (unlock)")

        trace(
            src="RECOVER",
            evt="RECOVER_RUNG_DONE",
            phase="BACKOFF_SCAN",
            run=self.run_id,
            result="FAILED",
        )
        trace(
            src="RECOVER",
            evt="RECOVER_RUNG_ENTER",
            phase="SEARCH_ROTATE",
            run=self.run_id,
            lock="none",
        )

        self._backoff = None
        self.phase = "SEARCH_ROTATE"
        return self.status

    def _update_search_rotate(self, perception, localisation, motion_backend):
        # IMPORTANT: SearchRotate rung is "unlock + scan for anything".
        self.locked_target_id = None

        if self._search is None:
            self._search = SearchRotate(
                kinds=[self.kind],
                step_deg=float(getattr(self.config, "recover_search_step_deg", 15.0)),
                max_deg=float(getattr(self.config, "recover_search_max_deg", 360.0)),
                timeout_s=float(getattr(self.config, "recover_search_timeout_s", 3.0)),
                max_age_s=float(getattr(self.config, "vision_loss_timeout_s", 0.5)),
                label="RECOVER_LOST_TARGET][SEARCH",
            )
            trace(
                src="RECOVER",
                evt="RECOVER_RUNG_ENTER",
                phase="SEARCH_ROTATE",
                run=self.run_id,
                lock="none",
            )

            self._search.start(motion_backend=motion_backend)

        sr = self._search.update(
            motion_backend=motion_backend,
            perception=perception,
        )

        # Keep banking pose during the scan (controller is updating localisation every tick)
        self._maybe_bank_pose(localisation)

        if sr == PrimitiveStatus.RUNNING:
            return self.status

        # If SearchRotate has a richer output API later, this is where you'd extract it.
        # For now, we assume "SUCCEEDED" means "saw something of interest" (as hinted in AcquireObject),
        # but we stay conservative: if it SUCCEEDED, we ask AcquireObject to funnel back to SELECT.
        if sr == PrimitiveStatus.SUCCEEDED:
            print("[RECOVER_LOST_TARGET][SEARCH_ROTATE] succeeded -> NEW_TARGET_FOUND (funnel to SELECT)")
            trace(
                src="RECOVER",
                evt="RECOVER_RUNG_DONE",
                phase="SEARCH_ROTATE",
                run=self.run_id,
                result="SUCCEEDED",
            )

            self._search = None
            self._finish(outcome="NEW_TARGET_FOUND")
            return self.status

        # FAILED: nothing found. If we banked pose at any point, return that (no need to run RecoverLocalisation immediately).
        self._search = None
        if self._banked_pose_xy is not None:
            print("[RECOVER_LOST_TARGET][SEARCH_ROTATE] no target, but pose was obtained")
            trace(
                src="RECOVER",
                evt="RECOVER_RUNG_DONE",
                phase="SEARCH_ROTATE",
                run=self.run_id,
                result="FAILED",
                reason="pose_obtained",
            )

            self._finish(outcome="POSE_OBTAINED_NO_TARGET")
            return self.status

        print("[RECOVER_LOST_TARGET][SEARCH_ROTATE] failed (no target, no pose)")
        trace(
            src="RECOVER",
            evt="RECOVER_RUNG_DONE",
            phase="SEARCH_ROTATE",
            run=self.run_id,
            result="FAILED",
            reason="no_target_no_pose",
        )

        self._finish(outcome="FAILED")
        return self.status

    # -------------------------
    # Helpers
    # -------------------------

    def _finish(self, *, outcome: RecoverOutcome, found_target_id=None, found_kind=None):
        self.result = RecoverLostTargetResult(
            outcome=outcome,
            found_target_id=found_target_id,
            found_kind=found_kind,
            pose_xy=self._banked_pose_xy,
            pose_heading=self._banked_pose_heading,
            pose_time_s=self._banked_pose_time_s,
        )
        self.status = BehaviorStatus.SUCCEEDED if outcome != "FAILED" else BehaviorStatus.FAILED
        trace(
            src="RECOVER",
            evt="RECOVER_DONE",
            phase="RECOVER_LOST_TARGET",
            run=self.run_id,
            outcome=outcome,
            lock=self.locked_target_id or "none",
            tid=found_target_id or "none",
            kind=found_kind or self.kind,
            pose=("yes" if self._banked_pose_xy is not None else "no"),
        )

        return self.status

    def _maybe_bank_pose(self, localisation):
        """
        Bank *any* valid pose we see during recovery.
        We don't care where it was seen; we care that we now have a usable pose snapshot.
        """
        try:
            if localisation is None or not localisation.has_pose():
                return

            pos, heading = localisation.get_pose()
            if pos is None:
                return

            x, y = pos
            self._banked_pose_xy = (float(x), float(y))
            self._banked_pose_heading = float(heading) if heading is not None else None

            # Prefer localisation.pose.timestamp if present
            t = None
            try:
                if getattr(localisation, "pose", None) is not None:
                    t = float(getattr(localisation.pose, "timestamp", 0.0))
            except Exception:
                t = None

            self._banked_pose_time_s = t if (t and t > 0) else time.time()

            # Only trace when pose meaningfully changes
            prev = getattr(self, "_last_pose_trace_xy", None)
            cur = self._banked_pose_xy

            if prev is None or abs(prev[0] - cur[0]) > 50 or abs(prev[1] - cur[1]) > 50:
                trace(
                    src="RECOVER",
                    evt="POSE_BANK",
                    phase=self.phase,
                    run=self.run_id,
                    pose=f"{cur[0]:.1f},{cur[1]:.1f}",
                    hdg=self._banked_pose_heading,
                )
                self._last_pose_trace_xy = cur


        except Exception:
            # never let recovery fail due to pose plumbing
            return

    def _make_reacquire_skill(self, *, target_id: int, last_bearing: float, last_distance: Optional[float]):
        """
        Mirror AcquireObject's compatibility-friendly ReacquireTarget construction.
        """
        step_deg = float(getattr(self.config, "recover_step_deg", 15.0))
        max_sweep_deg = float(getattr(self.config, "recover_max_sweep_deg", 180.0))
        max_age_s = float(getattr(self.config, "vision_loss_timeout_s", 0.5))

        # Try a few common constructor signatures.
        try:
            return ReacquireTarget(
                kind=self.kind,
                target_id=target_id,
                step_deg=step_deg,
                max_sweep_deg=max_sweep_deg,
                max_age_s=max_age_s,
                last_bearing_deg=float(last_bearing),
                last_distance_mm=(float(last_distance) if last_distance is not None else None),
                label="RECOVER_LOST_TARGET][REACQUIRE",
            )
        except TypeError:
            pass

        try:
            return ReacquireTarget(
                kind=self.kind,
                target_id=target_id,
                step_deg=step_deg,
                max_sweep_deg=max_sweep_deg,
                max_age_s=max_age_s,
                last_bearing_deg=float(last_bearing),
                label="RECOVER_LOST_TARGET][REACQUIRE",
            )
        except TypeError:
            pass

        # Minimal signature fallback
        return ReacquireTarget(
            kind=self.kind,
            target_id=target_id,
            step_deg=step_deg,
            max_sweep_deg=max_sweep_deg,
            max_age_s=max_age_s,
        )

    def _safe_stop(self, thing, *, motion_backend=None):
        """
        Stop helper tolerant to mixed stop() signatures and nested primitives.
        Mirrors AcquireObject._safe_stop.
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
            # fall through to more brute-force attempts
            pass

        # 2) Try without args
        try:
            thing.stop()
        except Exception:
            pass

        # 3) Try stopping common nested children
        for attr in ("active_primitive", "_child", "child", "_active", "primitive"):
            try:
                nested = getattr(thing, attr, None)
                if nested is None:
                    continue
                self._safe_stop(nested, motion_backend=motion_backend)
            except Exception:
                continue
