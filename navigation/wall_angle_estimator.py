# navigation/wall_angle_estimator.py

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Dict, Any, Tuple, List

from primitives.base import Primitive, PrimitiveStatus
from primitives.motion import Rotate

from navigation.wall_angle_one_ultrasonic import parallel_error_from_two_scans
from navigation.wall_angle_two_ultrasonics import estimate_wall_parallel_error_two_ultrasonics


def _cfg(config: Any, name: str, fallback: Any) -> Any:
    return getattr(config, name, fallback)


def _get_io_from_context(*, perception=None, motion_backend=None):
    """
    Best-effort IO discovery so we don't have to refactor every call site immediately.
    Priority:
      - motion_backend.io
      - motion_backend.lvl2.io
      - perception.io / perception._io
    """
    if motion_backend is not None:
        io = getattr(motion_backend, "io", None)
        if io is not None:
            return io
        lvl2 = getattr(motion_backend, "lvl2", None)
        if lvl2 is not None:
            io = getattr(lvl2, "io", None)
            if io is not None:
                return io

    if perception is not None:
        io = getattr(perception, "io", None)
        if io is not None:
            return io
        io = getattr(perception, "_io", None)
        if io is not None:
            return io

    return None


@dataclass(frozen=True)
class WallAngleTunables:
    wall_angle_backend: str

    # two ultrasonic
    wall_two_ultrasonic_keys: Tuple[str, str]
    wall_two_ultrasonic_baseline_mm: float

    # one ultrasonic scan
    wall_one_ultrasonic_key: str
    wall_scan_angle_1_deg: float
    wall_scan_angle_2_deg: float
    wall_scan_samples_per_angle: int
    wall_scan_settle_time_s: float

    # sanity
    wall_ultrasonic_min_mm: float
    wall_ultrasonic_max_mm: float

    # stability / staleness
    wall_angle_stable_samples: int
    wall_angle_max_age_s: float


class WallAngleEstimator(Primitive):
    """
    Stateful estimator returning "parallel error (deg)":
      0 = robot parallel to wall
      sign indicates direction to rotate to become parallel

    Primitive-like:
      - update() returns RUNNING while scanning / stabilising
      - returns SUCCEEDED when an estimate is available (get via .angle_deg)
      - returns FAILED if cannot produce (e.g. no IO, no readings, etc.)
    """

    def __init__(self, *, config: Any):
        super().__init__()
        self.config = config
        self.t = self._load_tunables(config)

        self._angle_deg: Optional[float] = None
        self._angle_time: Optional[float] = None
        self._stable_count: int = 0

        # scan state (one ultrasonic)
        self._phase: str = "IDLE"
        self._scan_angles: List[float] = []
        self._scan_idx: int = 0
        self._settle_until: Optional[float] = None
        self._samples: List[float] = []
        self._d1: Optional[float] = None
        self._d2: Optional[float] = None

        self._active_prim: Optional[Primitive] = None

    @property
    def angle_deg(self) -> Optional[float]:
        return self._angle_deg

    def _load_tunables(self, config: Any) -> WallAngleTunables:
        return WallAngleTunables(
            wall_angle_backend=str(_cfg(config, "wall_angle_backend", "one_ultrasonic_scan")),

            wall_two_ultrasonic_keys=tuple(_cfg(config, "wall_two_ultrasonic_keys", ("left", "right"))),
            wall_two_ultrasonic_baseline_mm=float(_cfg(config, "wall_two_ultrasonic_baseline_mm", 160.0)),

            wall_one_ultrasonic_key=str(_cfg(config, "wall_one_ultrasonic_key", "front")),
            wall_scan_angle_1_deg=float(_cfg(config, "wall_scan_angle_1_deg", -20.0)),
            wall_scan_angle_2_deg=float(_cfg(config, "wall_scan_angle_2_deg", 20.0)),
            wall_scan_samples_per_angle=int(_cfg(config, "wall_scan_samples_per_angle", 3)),
            wall_scan_settle_time_s=float(_cfg(config, "wall_scan_settle_time_s", 0.10)),

            wall_ultrasonic_min_mm=float(_cfg(config, "wall_ultrasonic_min_mm", 50.0)),
            wall_ultrasonic_max_mm=float(_cfg(config, "wall_ultrasonic_max_mm", 2500.0)),

            wall_angle_stable_samples=int(_cfg(config, "wall_angle_stable_samples", 2)),
            wall_angle_max_age_s=float(_cfg(config, "wall_angle_max_age_s", 0.25)),
        )

    def start(self, *, **_):
        self._angle_deg = None
        self._angle_time = None
        self._stable_count = 0

        self._phase = "IDLE"
        self._scan_angles = [self.t.wall_scan_angle_1_deg, self.t.wall_scan_angle_2_deg]
        self._scan_idx = 0
        self._settle_until = None
        self._samples = []
        self._d1 = None
        self._d2 = None

        self._active_prim = None
        self.status = PrimitiveStatus.RUNNING
        return self.status

    def _record_angle(self, angle: Optional[float], now: float) -> Optional[float]:
        if angle is None:
            self._stable_count = 0
            return None

        # stability: require consecutive valid estimates
        if self._angle_deg is None:
            self._stable_count = 1
        else:
            self._stable_count += 1

        self._angle_deg = float(angle)
        self._angle_time = now

        if self._stable_count >= max(1, self.t.wall_angle_stable_samples):
            return self._angle_deg
        return None

    def _is_fresh(self, now: float) -> bool:
        if self._angle_time is None:
            return False
        return (now - self._angle_time) <= self.t.wall_angle_max_age_s

    def update(self, *, motion_backend=None, perception=None, **_) -> PrimitiveStatus:
        now = time.time()

        # If we already have a fresh, stable estimate -> succeed immediately
        if self._angle_deg is not None and self._is_fresh(now) and self._stable_count >= max(1, self.t.wall_angle_stable_samples):
            return PrimitiveStatus.SUCCEEDED

        io = _get_io_from_context(perception=perception, motion_backend=motion_backend)
        if io is None:
            print("[WALL_ANGLE] no IO available in context")
            return PrimitiveStatus.FAILED

        backend = self.t.wall_angle_backend.strip().lower()

        # -------------------------
        # Backend: two ultrasonics (stateless)
        # -------------------------
        if backend == "two_ultrasonics":
            us = io.ultrasonics()
            kL, kR = self.t.wall_two_ultrasonic_keys
            angle = estimate_wall_parallel_error_two_ultrasonics(
                left_mm=us.get(kL),
                right_mm=us.get(kR),
                baseline_mm=self.t.wall_two_ultrasonic_baseline_mm,
                min_mm=self.t.wall_ultrasonic_min_mm,
                max_mm=self.t.wall_ultrasonic_max_mm,
            )
            stable = self._record_angle(angle, now)
            if stable is not None:
                print(f"[WALL_ANGLE][TWO] angle={stable:.2f}° (stable)")
                return PrimitiveStatus.SUCCEEDED

            print(f"[WALL_ANGLE][TWO] angle={self._angle_deg if self._angle_deg is not None else None}° (stabilising)")
            return PrimitiveStatus.RUNNING

        # -------------------------
        # Backend: one ultrasonic scan (stateful)
        # -------------------------
        if backend != "one_ultrasonic_scan":
            print(f"[WALL_ANGLE] unknown backend={backend!r}")
            return PrimitiveStatus.FAILED

        # Drive scan state machine
        if self._phase == "IDLE":
            self._scan_idx = 0
            self._d1 = None
            self._d2 = None
            self._samples = []
            self._phase = "ROTATE_TO_ANGLE"
            self._active_prim = None

        # If we have an active primitive (Rotate), run it
        if self._active_prim is not None:
            st = self._active_prim.update(motion_backend=motion_backend)
            if st == PrimitiveStatus.RUNNING:
                return PrimitiveStatus.RUNNING
            if st == PrimitiveStatus.FAILED:
                print("[WALL_ANGLE][SCAN] rotate failed")
                self._active_prim = None
                return PrimitiveStatus.FAILED

            # rotate complete -> settle
            self._active_prim = None
            self._settle_until = now + self.t.wall_scan_settle_time_s
            self._samples = []
            self._phase = "SETTLE"
            return PrimitiveStatus.RUNNING

        if self._phase == "SETTLE":
            if self._settle_until is not None and now < self._settle_until:
                return PrimitiveStatus.RUNNING
            self._phase = "SAMPLE"
            return PrimitiveStatus.RUNNING

        if self._phase == "ROTATE_TO_ANGLE":
            angle = float(self._scan_angles[self._scan_idx])
            self._active_prim = Rotate(angle_deg=angle)
            self._active_prim.start(motion_backend=motion_backend)
            self._phase = "ROTATING"
            return PrimitiveStatus.RUNNING

        if self._phase == "SAMPLE":
            us = io.ultrasonics()
            d = us.get(self.t.wall_one_ultrasonic_key)
            if d is not None and (self.t.wall_ultrasonic_min_mm <= float(d) <= self.t.wall_ultrasonic_max_mm):
                self._samples.append(float(d))

            if len(self._samples) < max(1, self.t.wall_scan_samples_per_angle):
                return PrimitiveStatus.RUNNING

            # average this angle’s samples
            avg = sum(self._samples) / float(len(self._samples))
            if self._scan_idx == 0:
                self._d1 = avg
            else:
                self._d2 = avg

            self._scan_idx += 1
            if self._scan_idx >= 2:
                # compute angle from (theta1,d1) and (theta2,d2)
                if self._d1 is None or self._d2 is None:
                    return PrimitiveStatus.FAILED

                angle = parallel_error_from_two_scans(
                    theta1_deg=self.t.wall_scan_angle_1_deg,
                    d1_mm=self._d1,
                    theta2_deg=self.t.wall_scan_angle_2_deg,
                    d2_mm=self._d2,
                )

                stable = self._record_angle(angle, now)
                if stable is not None:
                    print(f"[WALL_ANGLE][SCAN] angle={stable:.2f}° (stable)")
                    self._phase = "IDLE"
                    return PrimitiveStatus.SUCCEEDED

                print(f"[WALL_ANGLE][SCAN] angle={self._angle_deg if self._angle_deg is not None else None}° (stabilising)")
                self._phase = "IDLE"
                return PrimitiveStatus.RUNNING

            # go rotate to second scan angle
            self._phase = "ROTATE_TO_ANGLE"
            return PrimitiveStatus.RUNNING

        # ROTATING is handled by active primitive
        return PrimitiveStatus.RUNNING
