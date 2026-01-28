# skills/perception/reacquire_target.py

import time
from typing import Optional

from config import CONFIG
from primitives.base import Primitive, PrimitiveStatus
from primitives.motion import Rotate
from skills.perception.select_target_utils import get_closest_target


class ReacquireTarget(Primitive):
    def __init__(
        self,
        *,
        kind: str,
        step_deg: float,
        max_sweep_deg: float,
        max_age_s: float,
        target_id: int | None = None,
        cap_rel_deg: float = 30.0,
        **_,
    ):
        super().__init__()
        self.kind = kind
        self.target_id = int(target_id) if target_id is not None else None

        self.step_deg = float(step_deg)
        self.max_sweep_deg = float(max_sweep_deg)
        self.max_age_s = float(max_age_s)

        # Policy pulled directly from global config
        self.settle_s = float(CONFIG.recover_settle_time)
        self.cap_rel_deg = float(cap_rel_deg)

        # NEW: reacquire owns its own failure timing
        # Add this to your config resolver as REACQUIRE_TARGET_VISION_LOSS (seconds)
        self.vision_loss_s = float(getattr(CONFIG, "reacquire_target_vision_loss", 3.0))

        self._child: Optional[Rotate] = None
        self._settle_until: Optional[float] = None

        self._seq: list[float] = []
        self._i = 0
        self._rel_deg = 0.0

        self._start_time: Optional[float] = None

        self.found_target = None

    def start(self, *, motion_backend, **_):
        self._child = None
        self._settle_until = None
        self.found_target = None

        self._i = 0
        self._rel_deg = 0.0

        # NEW: start the reacquire-owned timer
        self._start_time = time.time()

        # Requested plan:
        #   +cap, then -step x4, then recenter to 0, then FAIL.
        cap = min(self.cap_rel_deg, 2.0 * self.step_deg)

        self._seq = [
            +cap,
            -self.step_deg,
            -self.step_deg,
            -self.step_deg,
            -self.step_deg,
        ]

    def _try_reacquire(self, *, perception, now: float):
        if perception is None:
            return None

        if self.target_id is not None:
            from perception import get_visible_targets

            visible = get_visible_targets(
                perception,
                self.kind,
                now=now,
                max_age_s=self.max_age_s,
            )
            for t in visible:
                if int(t.get("id", -1)) == self.target_id:
                    return t
            return None

        return get_closest_target(
            perception,
            self.kind,
            now=now,
            max_age_s=self.max_age_s,
        )

    def _timed_out(self, now: float) -> bool:
        if self._start_time is None:
            return False
        # 0 or negative -> "no timeout"
        if self.vision_loss_s <= 0.0:
            return False
        return (now - self._start_time) > self.vision_loss_s

    def update(self, *, motion_backend, perception=None, **_):
        now = time.time()

        # NEW: hard deadline owned by reacquire. If exceeded, stop sweeping and recenter -> FAIL.
        if self._timed_out(now):
            # stop any active child rotate
            if self._child is not None:
                self._child.stop()
                self._child = None
            self._settle_until = None
            self._i = len(self._seq)  # skip remaining plan

            print(
                f"[REACQUIRE] timeout {now - (self._start_time or now):.2f}s"
                f" > {self.vision_loss_s:.2f}s -> recenter then FAIL"
            )

            if abs(self._rel_deg) > 1e-3:
                angle = -self._rel_deg
                self._rel_deg = 0.0
                self._child = Rotate(angle_deg=angle)
                self._child.start(motion_backend=motion_backend)
                return PrimitiveStatus.RUNNING

            return PrimitiveStatus.FAILED

        # 0) Settling: wait for camera to stabilize
        if self._settle_until is not None:
            if now < self._settle_until:
                return PrimitiveStatus.RUNNING

            # settle finished — allow immediate reacquire before any further motion
            self._settle_until = None
            t = self._try_reacquire(perception=perception, now=now)
            if t is not None:
                self.found_target = t
                return PrimitiveStatus.SUCCEEDED

        # 1) If not currently rotating, check if target is visible now
        if self._child is None:
            t = self._try_reacquire(perception=perception, now=now)
            if t is not None:
                self.found_target = t
                return PrimitiveStatus.SUCCEEDED

        # 2) If rotating, advance rotate primitive
        if self._child is not None:
            st = self._child.update(motion_backend=motion_backend)
            if st == PrimitiveStatus.RUNNING:
                return PrimitiveStatus.RUNNING
            if st == PrimitiveStatus.FAILED:
                self._child = None
                return PrimitiveStatus.FAILED

            # Rotate completed -> begin settle time
            self._child = None
            self._settle_until = time.time() + max(0.0, self.settle_s)
            return PrimitiveStatus.RUNNING

        # 3) Start next planned rotate
        if self._i >= len(self._seq):
            print("[REACQUIRE] complete -> FAILED (recentering)")

            if abs(self._rel_deg) > 1e-3:
                angle = -self._rel_deg
                self._rel_deg = 0.0
                self._child = Rotate(angle_deg=angle)
                self._child.start(motion_backend=motion_backend)
                return PrimitiveStatus.RUNNING

            print("[REACQUIRE] recenter complete -> FAILED")
            return PrimitiveStatus.FAILED

        angle = float(self._seq[self._i])

        self._i += 1
        n = len(self._seq)

        self._rel_deg += angle

        print(
            f"[REACQUIRE] step {self._i}/{n} "
            f"rotate={angle:+.1f}deg settle={self.settle_s:.2f}s "
            f"rel={self._rel_deg:+.1f}deg"
        )

        self._child = Rotate(angle_deg=angle)
        self._child.start(motion_backend=motion_backend)
        return PrimitiveStatus.RUNNING

    def stop(self):
        if self._child is not None:
            self._child.stop()
        self._child = None
        self._settle_until = None
        self._start_time = None
