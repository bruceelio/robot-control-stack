# skills/navigation/backoff_scan.py

import time
from typing import Optional

from config import CONFIG
from primitives.base import Primitive, PrimitiveStatus
from primitives.motion import Drive, Rotate
from skills.perception.select_target_utils import get_closest_target


class BackoffScan(Primitive):
    """
    BACKOFF_SCAN (one-shot rung)

    Flow:
      1) Drive backwards (CONFIG.backoff_scan_mm)
      2) Settle (CONFIG.recover_settle_time)
      3) Scan by rotating across +/- cap in step increments, with settle between each rotate
      4) Final rotate recenters back to 0 (sequence includes this)
      5) If still not found: FAILED (caller escalates to GlobalRecovery)

    Config:
      - backoff_scan_mm
      - backoff_scan_cap_deg
      - backoff_scan_step_deg
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

        # Policy from CONFIG (you added these to schema+resolve map)
        self.backoff_mm = float(CONFIG.backoff_scan_mm)
        self.cap_rel_deg = float(CONFIG.backoff_scan_cap_deg)
        self.step_deg = float(CONFIG.backoff_scan_step_deg)
        self.timeout_s = float(CONFIG.backoff_scan_timeout_s)

        # settle between views/steps
        self.settle_s = float(getattr(CONFIG, "recover_settle_time", 0.5))

        # how fresh a detection must be to count
        self.max_age_s = float(max_age_s) if max_age_s is not None else float(getattr(CONFIG, "vision_loss_timeout_s", 0.5))

        # internal state
        self._deadline: Optional[float] = None
        self._phase = "BACKOFF"  # BACKOFF -> SETTLE -> SCAN -> DONE
        self._child: Optional[Primitive] = None

        self._settle_until: Optional[float] = None

        self._seq: list[float] = []
        self._i = 0
        self._rel_deg = 0.0

        self.found_target = None

    def start(self, *, motion_backend, **_):
        now = time.time()
        self.status = PrimitiveStatus.RUNNING

        self._deadline = now + max(0.1, self.timeout_s)
        self._phase = "BACKOFF"
        self._child = None

        self._settle_until = None

        self._i = 0
        self._rel_deg = 0.0
        self.found_target = None

        # Build scan sequence: +cap, then -step repeated to reach -cap, then +cap to recenter.
        cap = abs(self.cap_rel_deg)
        step = abs(self.step_deg)

        if step < 1e-6 or cap < 1e-6:
            print(f"[{self.label}] invalid cap/step (cap={cap}, step={step}) -> FAILED")
            self.status = PrimitiveStatus.FAILED
            return

        # From +cap down to -cap in -step increments:
        # n_down = (2*cap)/step  (e.g. 120/20=6)
        n_down = int(round((2.0 * cap) / step))
        n_down = max(1, n_down)

        self._seq = [cap] + ([-step] * n_down) + [cap]  # last +cap returns to ~0

        print(
            f"[{self.label}] start backoff_mm={self.backoff_mm:.0f} "
            f"cap={cap:.1f} step={step:.1f} settle={self.settle_s:.2f} "
            f"timeout={self.timeout_s:.2f} seq_len={len(self._seq)}"
        )

    # -------------------------
    # Perception check
    # -------------------------

    def _try_found(self, *, perception, now: float):
        if perception is None:
            return None

        if self.target_id is not None:
            from perception import get_visible_targets

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

        now = time.time()

        # Hard timeout
        if self._deadline is not None and now > self._deadline:
            print(f"[{self.label}] timeout -> FAILED")
            self.status = PrimitiveStatus.FAILED
            return self.status

        # If we are in a settle window, wait it out
        if self._settle_until is not None:
            if now < self._settle_until:
                return self.status
            self._settle_until = None
            # after settle, fall through to reassess / next step

        # If not rotating/driving right now, check for target visibility
        if self._child is None:
            t = self._try_found(perception=perception, now=now)
            if t is not None:
                self.found_target = t
                print(f"[{self.label}] found -> SUCCEEDED id={t.get('id', 'N/A')}")
                self.status = PrimitiveStatus.SUCCEEDED
                return self.status

        # -----------------
        # Phase: BACKOFF
        # -----------------
        if self._phase == "BACKOFF":
            if self._child is None:
                self._child = Drive(distance_mm=-self.backoff_mm)
                self._child.start(motion_backend=motion_backend)

            st = self._child.update(motion_backend=motion_backend)
            if st == PrimitiveStatus.SUCCEEDED:
                self._child = None
                self._phase = "SETTLE"
                self._settle_until = time.time() + max(0.0, self.settle_s)
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
            # Continue active rotate
            if self._child is not None:
                st = self._child.update(motion_backend=motion_backend)
                if st == PrimitiveStatus.RUNNING:
                    return self.status
                if st == PrimitiveStatus.FAILED:
                    print(f"[{self.label}] rotate primitive FAILED -> FAILED")
                    self._child = None
                    self.status = PrimitiveStatus.FAILED
                    return self.status

                # rotate finished -> settle before next view/step
                self._child = None
                self._settle_until = time.time() + max(0.0, self.settle_s)
                return self.status

            # Start next rotate step
            if self._i >= len(self._seq):
                print(f"[{self.label}] complete -> FAILED (handoff to global recovery)")
                self.status = PrimitiveStatus.FAILED
                return self.status

            angle = float(self._seq[self._i])
            self._i += 1
            n = len(self._seq)

            self._rel_deg += angle

            print(
                f"[{self.label}] step {self._i}/{n} "
                f"rotate={angle:+.1f}deg settle={self.settle_s:.2f}s "
                f"rel={self._rel_deg:+.1f}deg"
            )

            self._child = Rotate(angle_deg=angle)
            self._child.start(motion_backend=motion_backend)
            return self.status

        # Fallback
        return self.status

    def stop(self):
        if self._child is not None:
            self._child.stop()
        self._child = None
        self._settle_until = None
        self._deadline = None
