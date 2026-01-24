# navigation/wall_angle.py

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Any, Tuple, List

from primitives.base import Primitive, PrimitiveStatus
from primitives.motion import Rotate

from navigation.providers.wall_angle_ultrasonic1 import parallel_error_from_two_scans
from navigation.providers.wall_angle_ultrasonic2 import estimate_wall_parallel_error_two_ultrasonics


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


def _norm_ultrasonic(v: Any) -> Optional[float]:
    """
    Normalise SR ultrasonic readings:
      - None -> None
      - 0 or <=0 -> None   (0 commonly means no echo / timeout)
      - numeric -> float
      - non-numeric -> None
    """
    if v is None:
        return None
    try:
        f = float(v)
    except Exception:
        return None
    if f <= 0.0:
        return None
    return f


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
    wall_scan_sample_timeout_s: float

    # NEW: if one angle yields no valid samples, retry that angle this many times
    wall_scan_retries_per_angle: int

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
        self._sample_deadline: Optional[float] = None

        self._d1: Optional[float] = None
        self._d2: Optional[float] = None

        self._current_rel_deg: float = 0.0
        self._return_pending: bool = False

        self._active_prim: Optional[Primitive] = None
        self._printed_us_keys: bool = False

        # NEW: retry tracking for one-ultrasonic scan
        self._angle_retry_count: int = 0

    @property
    def angle_deg(self) -> Optional[float]:
        return self._angle_deg

    def _load_tunables(self, config: Any) -> WallAngleTunables:
        return WallAngleTunables(
            wall_angle_backend=str(_cfg(config, "wall_angle_backend", "one_ultrasonic_scan")),

            wall_two_ultrasonic_keys=tuple(_cfg(config, "wall_two_ultrasonic_keys", ("left", "right"))),
            wall_two_ultrasonic_baseline_mm=float(_cfg(config, "wall_two_ultrasonic_baseline_mm", 160.0)),

            wall_one_ultrasonic_key=str(_cfg(config, "wall_one_ultrasonic_key", "front")),
            wall_scan_angle_1_deg=float(_cfg(config, "wall_scan_angle_1_deg", -8.0)),
            wall_scan_angle_2_deg=float(_cfg(config, "wall_scan_angle_2_deg", 8.0)),
            wall_scan_samples_per_angle=int(_cfg(config, "wall_scan_samples_per_angle", 3)),
            wall_scan_settle_time_s=float(_cfg(config, "wall_scan_settle_time_s", 0.10)),
            wall_scan_sample_timeout_s=float(_cfg(config, "wall_scan_sample_timeout_s", 0.35)),

            wall_scan_retries_per_angle=int(_cfg(config, "wall_scan_retries_per_angle", 1)),

            wall_ultrasonic_min_mm=float(_cfg(config, "wall_ultrasonic_min_mm", 50.0)),
            wall_ultrasonic_max_mm=float(_cfg(config, "wall_ultrasonic_max_mm", 2500.0)),

            wall_angle_stable_samples=int(_cfg(config, "wall_angle_stable_samples", 2)),
            wall_angle_max_age_s=float(_cfg(config, "wall_angle_max_age_s", 0.25)),
        )

    def start(self, **_):
        self._angle_deg = None
        self._angle_time = None
        self._stable_count = 0

        self._phase = "IDLE"
        self._scan_angles = [self.t.wall_scan_angle_1_deg, self.t.wall_scan_angle_2_deg]
        self._scan_idx = 0
        self._settle_until = None

        self._samples = []
        self._sample_deadline = None

        self._d1 = None
        self._d2 = None

        self._current_rel_deg = 0.0
        self._return_pending = False

        self._active_prim = None
        self._printed_us_keys = False

        self._angle_retry_count = 0

        self.status = PrimitiveStatus.RUNNING
        return self.status

    def _record_angle(self, angle: Optional[float], now: float) -> Optional[float]:
        if angle is None:
            self._stable_count = 0
            return None

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

        # Already have a fresh, stable estimate -> succeed
        if (
            self._angle_deg is not None
            and self._is_fresh(now)
            and self._stable_count >= max(1, self.t.wall_angle_stable_samples)
        ):
            return PrimitiveStatus.SUCCEEDED

        io = _get_io_from_context(perception=perception, motion_backend=motion_backend)
        if io is None:
            print("[WALL_ANGLE] no IO available in context")
            return PrimitiveStatus.FAILED

        backend = self.t.wall_angle_backend.strip().lower()

        # -------------------------
        # Backend: two ultrasonics
        # -------------------------
        if backend == "two_ultrasonics":
            us = io.ultrasonics()
            if not self._printed_us_keys:
                try:
                    keys = list(us.keys())
                except Exception:
                    keys = []
                print(f"[WALL_ANGLE][US] keys={keys} want={self.t.wall_two_ultrasonic_keys}")
                self._printed_us_keys = True

            kL, kR = self.t.wall_two_ultrasonic_keys
            angle = estimate_wall_parallel_error_two_ultrasonics(
                left_mm=_norm_ultrasonic(us.get(kL)),
                right_mm=_norm_ultrasonic(us.get(kR)),
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
        # Backend: one ultrasonic scan
        # -------------------------
        if backend != "one_ultrasonic_scan":
            print(f"[WALL_ANGLE] unknown backend={backend!r}")
            return PrimitiveStatus.FAILED

        # init scan cycle
        if self._phase == "IDLE":
            self._scan_idx = 0
            self._d1 = None
            self._d2 = None
            self._samples = []
            self._sample_deadline = None
            self._settle_until = None
            self._current_rel_deg = 0.0
            self._return_pending = False
            self._phase = "ROTATE_TO_ANGLE"
            self._active_prim = None
            self._angle_retry_count = 0

        # run active rotation
        if self._active_prim is not None:
            st = self._active_prim.update(motion_backend=motion_backend)
            if st == PrimitiveStatus.RUNNING:
                return PrimitiveStatus.RUNNING
            if st == PrimitiveStatus.FAILED:
                print("[WALL_ANGLE][SCAN] rotate failed")
                self._active_prim = None
                return PrimitiveStatus.FAILED

            self._active_prim = None
            self._settle_until = now + self.t.wall_scan_settle_time_s
            self._samples = []
            self._sample_deadline = None
            self._phase = "SETTLE"
            return PrimitiveStatus.RUNNING

        if self._phase == "SETTLE":
            if self._settle_until is not None and now < self._settle_until:
                return PrimitiveStatus.RUNNING
            self._phase = "SAMPLE"
            self._sample_deadline = now + self.t.wall_scan_sample_timeout_s
            return PrimitiveStatus.RUNNING

        if self._phase == "ROTATE_TO_ANGLE":
            target_rel = float(self._scan_angles[self._scan_idx])
            delta = target_rel - float(self._current_rel_deg)

            print(f"[WALL_ANGLE][SCAN] rotate_to rel={target_rel:.1f}° (delta={delta:.1f}°)")
            self._active_prim = Rotate(angle_deg=delta)
            self._active_prim.start(motion_backend=motion_backend)

            self._current_rel_deg = target_rel
            self._phase = "ROTATING"
            return PrimitiveStatus.RUNNING

        if self._phase == "SAMPLE":
            us = io.ultrasonics()

            if not self._printed_us_keys:
                try:
                    keys = list(us.keys())
                except Exception:
                    keys = []
                print(f"[WALL_ANGLE][US] keys={keys} want='{self.t.wall_one_ultrasonic_key}'")
                self._printed_us_keys = True

            raw = us.get(self.t.wall_one_ultrasonic_key)
            d = _norm_ultrasonic(raw)

            # take sample if valid and within max (min is soft)
            if d is not None and d <= self.t.wall_ultrasonic_max_mm:
                if d >= self.t.wall_ultrasonic_min_mm:
                    self._samples.append(d)
                else:
                    # below min but >0: allow it as a sample (soft min)
                    self._samples.append(d)

            # got enough samples
            if len(self._samples) >= max(1, self.t.wall_scan_samples_per_angle):
                avg = sum(self._samples) / float(len(self._samples))
                print(f"[WALL_ANGLE][SCAN] samples ok idx={self._scan_idx} avg={avg:.1f}mm n={len(self._samples)}")

                if self._scan_idx == 0:
                    self._d1 = avg
                else:
                    self._d2 = avg

                # reset retry counter when we succeed at an angle
                self._angle_retry_count = 0

                self._scan_idx += 1
                if self._scan_idx >= 2:
                    if self._d1 is None or self._d2 is None:
                        print("[WALL_ANGLE][SCAN] missing d1/d2 after sampling")
                        return PrimitiveStatus.FAILED

                    angle = parallel_error_from_two_scans(
                        theta1_deg=self.t.wall_scan_angle_1_deg,
                        d1_mm=self._d1,
                        theta2_deg=self.t.wall_scan_angle_2_deg,
                        d2_mm=self._d2,
                    )

                    stable = self._record_angle(angle, now)
                    print(f"[WALL_ANGLE][SCAN] raw_angle={angle:.2f}° stable_count={self._stable_count}")

                    self._return_pending = True
                    self._phase = "RETURN_TO_CENTER"
                    return PrimitiveStatus.RUNNING

                self._phase = "ROTATE_TO_ANGLE"
                return PrimitiveStatus.RUNNING

            # timeout: no enough samples yet
            if self._sample_deadline is not None and now > self._sample_deadline:
                # NEW: if we got *zero* samples at this angle, retry the same angle once
                if len(self._samples) == 0 and self._angle_retry_count < self.t.wall_scan_retries_per_angle:
                    self._angle_retry_count += 1
                    print(
                        "[WALL_ANGLE][SCAN] sample timeout with 0 samples; retrying same angle "
                        f"(retry={self._angle_retry_count}/{self.t.wall_scan_retries_per_angle}) "
                        f"last_raw={raw!r} last_norm={d}"
                    )
                    # restart sample window (don’t rotate again)
                    self._samples = []
                    self._sample_deadline = now + self.t.wall_scan_sample_timeout_s
                    return PrimitiveStatus.RUNNING

                print(
                    "[WALL_ANGLE][SCAN] sample timeout "
                    f"(got={len(self._samples)}/{max(1, self.t.wall_scan_samples_per_angle)}) "
                    f"last_raw={raw!r} last_norm={d}"
                )
                return PrimitiveStatus.FAILED

            return PrimitiveStatus.RUNNING

        if self._phase == "RETURN_TO_CENTER":
            if self._return_pending:
                self._return_pending = False
                delta_back = -float(self._current_rel_deg)
                print(f"[WALL_ANGLE][SCAN] return_to_center delta={delta_back:.1f}°")
                self._current_rel_deg = 0.0
                self._active_prim = Rotate(angle_deg=delta_back)
                self._active_prim.start(motion_backend=motion_backend)
                self._phase = "ROTATING"
                return PrimitiveStatus.RUNNING

            if self._angle_deg is not None and self._stable_count >= max(1, self.t.wall_angle_stable_samples):
                print(f"[WALL_ANGLE][SCAN] angle={self._angle_deg:.2f}° (stable)")
                self._phase = "IDLE"
                return PrimitiveStatus.SUCCEEDED

            print(f"[WALL_ANGLE][SCAN] angle={self._angle_deg if self._angle_deg is not None else None}° (stabilising)")
            self._phase = "IDLE"
            return PrimitiveStatus.RUNNING

        return PrimitiveStatus.RUNNING
