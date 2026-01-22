# skills/navigation/parallel_to_wall.py

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Optional

from primitives.base import Primitive, PrimitiveStatus
from primitives.motion import Rotate

from navigation.wall_angle_estimator import WallAngleEstimator


def _cfg(config: Any, name: str, fallback: Any) -> Any:
    return getattr(config, name, fallback)


@dataclass(frozen=True)
class ParallelTunables:
    tolerance_deg: float
    trigger_deg: float
    max_rotate_deg: float
    step_deg: float
    timeout_s: float


class ParallelToWall(Primitive):
    """
    Rotate until parallel-to-wall error is within tolerance.

    Uses WallAngleEstimator internally (backend chosen by config).
    """

    def __init__(self, *, config: Any):
        super().__init__()
        self.config = config
        self.t = ParallelTunables(
            tolerance_deg=float(_cfg(config, "wall_parallel_tolerance_deg", 3.0)),
            trigger_deg=float(_cfg(config, "wall_parallel_trigger_deg", 10.0)),
            max_rotate_deg=float(_cfg(config, "wall_parallel_max_rotate_deg", 15.0)),
            step_deg=float(_cfg(config, "wall_parallel_step_deg", 5.0)),
            timeout_s=float(_cfg(config, "wall_parallel_timeout_s", 2.0)),
        )

        self._est = WallAngleEstimator(config=config)
        self._rotate: Optional[Rotate] = None
        self._deadline: Optional[float] = None

    def start(self, *, **_):
        self._est.start()
        self._rotate = None
        self._deadline = time.time() + self.t.timeout_s
        self.status = PrimitiveStatus.RUNNING
        return self.status

    def update(self, *, motion_backend=None, perception=None, **_) -> PrimitiveStatus:
        now = time.time()
        if self._deadline is not None and now > self._deadline:
            print("[PARALLEL] timeout")
            return PrimitiveStatus.FAILED

        # If rotating, finish it first
        if self._rotate is not None:
            st = self._rotate.update(motion_backend=motion_backend)
            if st == PrimitiveStatus.RUNNING:
                return PrimitiveStatus.RUNNING
            if st == PrimitiveStatus.FAILED:
                print("[PARALLEL] rotate failed")
                self._rotate = None
                return PrimitiveStatus.FAILED
            self._rotate = None
            # after rotate, re-estimate next tick
            return PrimitiveStatus.RUNNING

        # Get an angle estimate
        st = self._est.update(motion_backend=motion_backend, perception=perception)
        if st == PrimitiveStatus.FAILED:
            print("[PARALLEL] wall angle estimator failed")
            return PrimitiveStatus.FAILED
        if st == PrimitiveStatus.RUNNING:
            return PrimitiveStatus.RUNNING

        # SUCCEEDED: we have an angle
        angle = self._est.angle_deg
        if angle is None:
            return PrimitiveStatus.RUNNING

        err = float(angle)
        print(f"[PARALLEL] wall_parallel_error={err:.2f}°")

        if abs(err) <= self.t.tolerance_deg:
            print("[PARALLEL] within tolerance")
            return PrimitiveStatus.SUCCEEDED

        # Rotate in small steps toward reducing error.
        # Clamp to avoid wild spins on bad readings.
        step = max(-self.t.max_rotate_deg, min(self.t.max_rotate_deg, err))
        # Also cap step magnitude to configured step_deg (for stability)
        step = max(-self.t.step_deg, min(self.t.step_deg, step))

        self._rotate = Rotate(angle_deg=step)
        self._rotate.start(motion_backend=motion_backend)
        return PrimitiveStatus.RUNNING

    def stop(self):
        if self._rotate is not None:
            self._rotate.stop()
        self._rotate = None
