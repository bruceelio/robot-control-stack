# skills/navigation/backoff_scan.py

import math

from typing import Optional

from config import CONFIG
from primitives.base import Primitive, PrimitiveStatus
from primitives.motion import Drive
from skills.navigation.search_sweep import SearchSweep
from skills.perception.select_target_utils import get_closest_target


class BackoffScan(Primitive):
    """
    BACKOFF_SCAN (one-shot rung)

    Flow:
      1) Drive backwards (CONFIG.backoff_scan_mm)
      2) Settle (CONFIG.recover_settle_time)
      3) Continuously sweep from centre -> +cap -> -cap -> centre
      4) Stop immediately if the target is seen during the sweep
      5) If still not found: FAILED (caller escalates to GlobalRecovery)

    Config:
      - backoff_scan_mm
      - backoff_scan_cap_deg
      - backoff_scan_timeout_s
      - recover_settle_time

    Success:
      - If target_id is provided: only succeed when that exact id is visible.
      - Otherwise: succeed when any visible target of `kind` exists (closest).
    """

    def __init__(
        self,
        *,
        kind: str,
        target_id: int | None = None,
        max_age_s: float | None = None,
        label: str = "BACKOFF_SCAN",
        **_,
    ):
        super().__init__()
        self.kind = kind
        self.target_id = int(target_id) if target_id is not None else None
        self.label = label
        self._io = None

        # Policy from CONFIG (you added these to schema+resolve map)
        self.backoff_mm = float(CONFIG.backoff_scan_mm)
        self.cap_rel_deg = float(CONFIG.backoff_scan_cap_deg)

        self.timeout_s = float(CONFIG.backoff_scan_timeout_s)

        # settle after the backoff translation
        self.settle_s = float(getattr(CONFIG, "recover_settle_time", 0.5))

        # how fresh a detection must be to count
        self.max_age_s = float(max_age_s) if max_age_s is not None else float(getattr(CONFIG, "vision_loss_timeout_s", 0.5))

        # internal state

        self._phase = "BACKOFF"  # BACKOFF -> SETTLE -> SCAN -> DONE
        self._child: Optional[Primitive] = None

        self._settle_until: Optional[float] = None

        self._sweep: SearchSweep | None = None
        self._sweep_started = False

        self.found_target = None

    def start(self, *, motion_backend, **_):
        self._io = motion_backend.lvl2.io

        self.status = PrimitiveStatus.RUNNING

        self._phase = "BACKOFF"
        self._child = None

        self._settle_until = None

        self._sweep = None
        self._sweep_started = False
        self.found_target = None

        cap = abs(
            self.cap_rel_deg
        )

        if cap < 1e-6:
            print(
                f"[{self.label}] "
                f"invalid cap={cap} -> FAILED"
            )
            self.status = PrimitiveStatus.FAILED
            return self.status

        angles_deg = (
            +cap,
            -(2.0 * cap),
            +cap,
        )

        angular_speed_rad_s = abs(
            float(
                CONFIG.servoing_angular_max_rad_s
            )
        )

        sweep_timeout_s = self.timeout_s

        if angular_speed_rad_s > 0.0:
            sweep_timeout_s = max(
                sweep_timeout_s,
                (
                        math.radians(
                            sum(
                                abs(angle)
                                for angle in angles_deg
                            )
                        )
                        / angular_speed_rad_s
                        + 1.0
                ),
            )

        self._sweep = SearchSweep(
            angles_deg=angles_deg,
            timeout_s=sweep_timeout_s,
            config=CONFIG,
            label=f"{self.label}][SWEEP",
        )

        print(
            f"[{self.label}] start "
            f"backoff_mm={self.backoff_mm:.0f} "
            f"cap={cap:.1f} "
            f"settle={self.settle_s:.2f} "
            f"sweep_timeout={sweep_timeout_s:.2f}"
        )

        return self.status

    # -------------------------
    # Perception check
    # -------------------------

    def _try_found(self, *, perception, now: float):
        if perception is None:
            return None

        if self.target_id is not None:
            from perception.perception import get_visible_targets

            visible = get_visible_targets(perception, self.kind, now=now, max_age_s=self.max_age_s)
            for t in visible:
                if int(t.get("id", -1)) == self.target_id:
                    return t
            return None

        return get_closest_target(perception, self.kind, now=now, max_age_s=self.max_age_s)

    # -------------------------
    # Main update
    # -------------------------

    def update(self, *, motion_backend, perception=None, **_):
        if self.status != PrimitiveStatus.RUNNING:
            return self.status

        now = float(self._io.time())

        # If we are in a settle window, wait it out
        if self._settle_until is not None:
            if now < self._settle_until:
                return self.status
            self._settle_until = None
            # after settle, fall through to reassess / next step

        # Outside the active sweep, check for target visibility.
        # During SCAN, SearchSweep must receive found_item so that it
        # can stop the active SearchRotate before succeeding.
        if (
                self._phase != "SCAN"
                and self._child is None
        ):
            t = self._try_found(
                perception=perception,
                now=now,
            )

            if t is not None:
                self.found_target = t

                print(
                    f"[{self.label}] "
                    f"found -> SUCCEEDED "
                    f"id={t.get('id', 'N/A')}"
                )

                self.status = PrimitiveStatus.SUCCEEDED
                return self.status

        # -----------------
        # Phase: BACKOFF
        # -----------------
        if self._phase == "BACKOFF":
            if self._child is None:
                self._child = Drive(
                    distance_mm=-self.backoff_mm
                )

                self._child.start(
                    motion_backend=motion_backend
                )

            st = self._child.update(
                motion_backend=motion_backend
            )
            if st == PrimitiveStatus.SUCCEEDED:
                self._child = None
                self._phase = "SETTLE"
                self._settle_until = now + max(
                    0.0,
                    self.settle_s,
                )
                return self.status
            if st == PrimitiveStatus.FAILED:
                print(f"[{self.label}] backoff drive FAILED -> FAILED")
                self.status = PrimitiveStatus.FAILED
                return self.status

            return self.status

        # -----------------
        # Phase: SETTLE (post-backoff)
        # -----------------
        if self._phase == "SETTLE":
            # settle handled at top via _settle_until; once cleared we move on
            self._phase = "SCAN"

        # -----------------
        # Phase: SCAN
        # -----------------
        if self._phase == "SCAN":
            if self._sweep is None:
                print(
                    f"[{self.label}] "
                    "missing sweep -> FAILED"
                )
                self.status = PrimitiveStatus.FAILED
                return self.status

            target = self._try_found(
                perception=perception,
                now=now,
            )

            # Preserve the fast path before starting rotation.
            if (
                not self._sweep_started
                and target is not None
            ):
                self.found_target = target

                print(
                    f"[{self.label}] "
                    f"found -> SUCCEEDED "
                    f"id={target.get('id', 'N/A')}"
                )

                self.status = PrimitiveStatus.SUCCEEDED
                return self.status

            if not self._sweep_started:
                st = self._sweep.start(
                    motion_backend=motion_backend,
                )

                self._sweep_started = True

                if st == PrimitiveStatus.FAILED:
                    self.status = PrimitiveStatus.FAILED

                return self.status

            st = self._sweep.update(
                motion_backend=motion_backend,
                found_item=target,
            )

            if st == PrimitiveStatus.RUNNING:
                return self.status

            if st == PrimitiveStatus.SUCCEEDED:
                self.found_target = (
                    self._sweep.found_item
                )

                print(
                    f"[{self.label}] "
                    f"found -> SUCCEEDED "
                    f"id="
                    f"{self.found_target.get('id', 'N/A')}"
                )

                self.status = PrimitiveStatus.SUCCEEDED
                return self.status

            print(
                f"[{self.label}] "
                "sweep complete -> FAILED "
                "(handoff to global recovery)"
            )

            self.status = PrimitiveStatus.FAILED
            return self.status

    def stop(
            self,
            *,
            motion_backend=None,
    ):
        if self._child is not None:
            try:
                self._child.stop(
                    motion_backend=motion_backend,
                )
            except TypeError:
                self._child.stop()

        if self._sweep is not None:
            self._sweep.stop(
                motion_backend=motion_backend,
            )

        self._child = None
        self._sweep = None
        self._sweep_started = False
        self._settle_until = None

