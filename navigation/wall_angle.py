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
    if f <= 0:
        return None
    return f


@dataclass
class WallAngleEstimate:
    ok: bool
    angle_deg: Optional[float] = None
    age_s: float = 999.0
    d1_mm: Optional[float] = None
    d2_mm: Optional[float] = None
    reason: str = ""


class WallAngleEstimator(Primitive):
    """
    Estimates the wall-parallel error angle (deg) using either:
      - one ultrasonic scanned at two angles (config.wall_angle_backend == "one_ultrasonic_scan")
      - two ultrasonics (config.wall_angle_backend == "two_ultrasonics")

    For one_ultrasonic_scan:
      rotates to angle_1 -> reads -> rotates to angle_2 -> reads -> computes error

    Debug requirement:
      prints each raw ultrasonic reading on its own line as it happens.
    """

    def __init__(self, *, config: Any, label: str = "WALL_ANGLE"):
        try:
            super().__init__(label=label)
        except TypeError:
            super().__init__()
            self.label = label

        self.config = config
        self.backend = str(_cfg(config, "wall_angle_backend", "one_ultrasonic_scan"))

        # --- Config compatibility ---
        legacy = _cfg(config, "wall_scan_angle_deg", None)
        if legacy is not None:
            a1 = -float(legacy)
            a2 = +float(legacy)
        else:
            a1 = float(_cfg(config, "wall_scan_angle_1_deg", -8.0))
            a2 = float(_cfg(config, "wall_scan_angle_2_deg", +8.0))

        self.scan_angles_primary: Tuple[float, float] = (a1, a2)

        self.samples_per_angle = int(_cfg(config, "wall_scan_samples_per_angle", 3))
        self.settle_time_s = float(_cfg(config, "wall_scan_settle_time_s", 0.1))

        self.ultra_key_one = str(_cfg(config, "wall_one_ultrasonic_key", "front"))
        self.ultra_keys_two = tuple(_cfg(config, "wall_two_ultrasonic_keys", ("left", "right")))
        self.baseline_mm = float(_cfg(config, "wall_two_ultrasonic_baseline_mm", 160.0))

        self.min_mm = float(_cfg(config, "wall_ultrasonic_min_mm", 50.0))
        self.max_mm = float(_cfg(config, "wall_ultrasonic_max_mm", 2500.0))

        self.stable_samples = int(_cfg(config, "wall_angle_stable_samples", 2))
        self.max_age_s = float(_cfg(config, "wall_angle_max_age_s", 0.25))

        # Optional guardrails to reduce “platform vs far wall” bad angles
        self.max_pair_delta_mm = float(_cfg(config, "wall_scan_max_pair_delta_mm", 1200.0))
        self.max_pair_ratio = float(_cfg(config, "wall_scan_max_pair_ratio", 2.2))

        # internal state
        self._io = None
        self._phase = "IDLE"  # ROTATE_1, READ_1, ROTATE_2, READ_2, DONE, TWO_US
        self._active: Optional[Primitive] = None
        self._angle_target_deg: Optional[float] = None

        self._samples_1: List[float] = []
        self._samples_2: List[float] = []

        self._last_estimate: WallAngleEstimate = WallAngleEstimate(ok=False, reason="not_started")
        self._last_ok_s: Optional[float] = None
        self._stable_ok_count = 0

    # ---- Public API ----

    @property
    def estimate(self) -> WallAngleEstimate:
        if self._last_ok_s is None:
            return WallAngleEstimate(ok=False, reason=self._last_estimate.reason)
        age = time.time() - self._last_ok_s
        e = self._last_estimate
        return WallAngleEstimate(
            ok=(e.ok and age <= self.max_age_s),
            angle_deg=e.angle_deg,
            age_s=age,
            d1_mm=e.d1_mm,
            d2_mm=e.d2_mm,
            reason=e.reason if age <= self.max_age_s else "stale",
        )

    def start(self, *, perception=None, motion_backend=None, io=None, **_):
        self._io = io or _get_io_from_context(perception=perception, motion_backend=motion_backend)
        self._phase = "IDLE"
        self._active = None
        self._angle_target_deg = None
        self._samples_1 = []
        self._samples_2 = []
        self._stable_ok_count = 0

        if self.backend == "two_ultrasonics":
            self._phase = "TWO_US"
        else:
            self._phase = "ROTATE_1"

        self.status = PrimitiveStatus.RUNNING
        return self.status

    def update(self, *, perception=None, motion_backend=None, io=None, **_):
        if self.status != PrimitiveStatus.RUNNING:
            return self.status

        if self._io is None:
            self._io = io or _get_io_from_context(perception=perception, motion_backend=motion_backend)

        if self._io is None:
            self._last_estimate = WallAngleEstimate(ok=False, reason="no_io")
            return self.status

        if self.backend == "two_ultrasonics":
            return self._update_two_ultrasonics()

        return self._update_one_ultrasonic(motion_backend=motion_backend)

    def stop(self, *, motion_backend=None, **_):
        if self._active is not None:
            try:
                self._active.stop(motion_backend=motion_backend)
            except Exception:
                try:
                    self._active.stop()
                except Exception:
                    pass
        self._active = None
        self.status = PrimitiveStatus.FAILED

    # ---- Internals ----

    def _read_ultrasonic_once(self, *, key: str, angle_tag: str, sample_idx: int) -> Optional[float]:
        """
        Read a single sample and print it (even if invalid).

        MODIFICATION:
          - Prefer canonical IOMap: io.ultrasonics() -> dict
          - Fallback to legacy: io.ultrasonic[key] if present
        """
        raw = None
        try:
            # Canonical IOMap path (works in sim + real with your IOMap)
            u = self._io.ultrasonics()
            raw = u.get(key)
        except Exception:
            # Legacy compatibility path (older code may expose .ultrasonic as a dict)
            try:
                raw = self._io.ultrasonic[key]
            except Exception:
                raw = None

        d = _norm_ultrasonic(raw)

        # Always print one line per reading, as requested.
        if d is None:
            print(f"[WALL_ANGLE][US] {angle_tag} sample={sample_idx} key={key} raw={raw!r} -> INVALID")
            return None

        if not (self.min_mm <= d <= self.max_mm):
            print(
                f"[WALL_ANGLE][US] {angle_tag} sample={sample_idx} key={key} raw={raw!r} -> {d:.0f}mm OUT_OF_RANGE"
            )
            return None

        print(f"[WALL_ANGLE][US] {angle_tag} sample={sample_idx} key={key} raw={raw!r} -> {d:.0f}mm")
        return d

    def _median(self, xs: List[float]) -> Optional[float]:
        if not xs:
            return None
        s = sorted(xs)
        return s[len(s) // 2]

    def _update_two_ultrasonics(self):
        k1, k2 = self.ultra_keys_two[0], self.ultra_keys_two[1]

        d1 = self._read_ultrasonic_once(key=k1, angle_tag="two_us[left]", sample_idx=1)
        d2 = self._read_ultrasonic_once(key=k2, angle_tag="two_us[right]", sample_idx=1)

        if d1 is None or d2 is None:
            self._last_estimate = WallAngleEstimate(ok=False, reason="two_us_invalid", d1_mm=d1, d2_mm=d2)
            return self.status

        try:
            angle = estimate_wall_parallel_error_two_ultrasonics(
                left_mm=d1, right_mm=d2, baseline_mm=self.baseline_mm
            )
        except Exception as e:
            self._last_estimate = WallAngleEstimate(ok=False, reason=f"two_us_math:{e!s}", d1_mm=d1, d2_mm=d2)
            return self.status

        self._mark_estimate_ok(angle_deg=angle, d1_mm=d1, d2_mm=d2, reason="two_ultrasonics")
        return self.status

    def _update_one_ultrasonic(self, *, motion_backend):
        a1, a2 = self.scan_angles_primary

        # ROTATE_1
        if self._phase == "ROTATE_1":
            self._angle_target_deg = a1
            if self._active is None:
                self._active = Rotate(angle_deg=a1)
                self._active.start(motion_backend=motion_backend)
            st = self._active.update(motion_backend=motion_backend)
            if st == PrimitiveStatus.RUNNING:
                return self.status
            self._active = None
            self._phase = "READ_1"
            self._t_settle = time.time() + self.settle_time_s
            return self.status

        # READ_1
        if self._phase == "READ_1":
            if time.time() < getattr(self, "_t_settle", 0):
                return self.status

            idx = len(self._samples_1) + 1
            d = self._read_ultrasonic_once(key=self.ultra_key_one, angle_tag=f"a1={a1:+.1f}deg", sample_idx=idx)
            if d is not None:
                self._samples_1.append(d)

            if len(self._samples_1) >= self.samples_per_angle:
                self._phase = "ROTATE_2"
                self._samples_2 = []
            else:
                self._t_settle = time.time() + self.settle_time_s
            return self.status

        # ROTATE_2
        if self._phase == "ROTATE_2":
            self._angle_target_deg = a2
            if self._active is None:
                self._active = Rotate(angle_deg=a2)
                self._active.start(motion_backend=motion_backend)
            st = self._active.update(motion_backend=motion_backend)
            if st == PrimitiveStatus.RUNNING:
                return self.status
            self._active = None
            self._phase = "READ_2"
            self._t_settle = time.time() + self.settle_time_s
            return self.status

        # READ_2
        if self._phase == "READ_2":
            if time.time() < getattr(self, "_t_settle", 0):
                return self.status

            idx = len(self._samples_2) + 1
            d = self._read_ultrasonic_once(key=self.ultra_key_one, angle_tag=f"a2={a2:+.1f}deg", sample_idx=idx)
            if d is not None:
                self._samples_2.append(d)

            if len(self._samples_2) >= self.samples_per_angle:
                self._compute_from_two_scans(a1=a1, a2=a2)
                self._phase = "ROTATE_1"
                self._samples_1 = []
                self._samples_2 = []
            else:
                self._t_settle = time.time() + self.settle_time_s
            return self.status

        self._phase = "ROTATE_1"
        return self.status

    def _compute_from_two_scans(self, *, a1: float, a2: float):
        d1 = self._median(self._samples_1)
        d2 = self._median(self._samples_2)

        if d1 is None or d2 is None:
            self._last_estimate = WallAngleEstimate(ok=False, reason="scan_invalid", d1_mm=d1, d2_mm=d2)
            return

        delta = abs(d1 - d2)
        ratio = (max(d1, d2) / max(1.0, min(d1, d2)))
        if delta > self.max_pair_delta_mm or ratio > self.max_pair_ratio:
            self._last_estimate = WallAngleEstimate(
                ok=False,
                reason=f"pair_mismatch(delta={delta:.0f}mm ratio={ratio:.2f})",
                d1_mm=d1,
                d2_mm=d2,
            )
            print(
                f"[WALL_ANGLE] rejected scan pair: d1={d1:.0f} d2={d2:.0f} "
                f"delta={delta:.0f}mm ratio={ratio:.2f}"
            )
            self._stable_ok_count = 0
            return

        try:
            angle = parallel_error_from_two_scans(
                d1_mm=float(d1),
                d2_mm=float(d2),
                scan_angle_1_deg=float(a1),
                scan_angle_2_deg=float(a2),
            )
        except Exception as e:
            self._last_estimate = WallAngleEstimate(ok=False, reason=f"math:{e!s}", d1_mm=d1, d2_mm=d2)
            self._stable_ok_count = 0
            return

        self._mark_estimate_ok(angle_deg=angle, d1_mm=d1, d2_mm=d2, reason="one_ultrasonic_scan")

    def _mark_estimate_ok(self, *, angle_deg: float, d1_mm: float, d2_mm: float, reason: str):
        self._last_ok_s = time.time()
        self._last_estimate = WallAngleEstimate(
            ok=True,
            angle_deg=float(angle_deg),
            age_s=0.0,
            d1_mm=d1_mm,
            d2_mm=d2_mm,
            reason=reason,
        )

        self._stable_ok_count += 1
        print(
            f"[WALL_ANGLE] ok angle={float(angle_deg):+.2f}deg d1={d1_mm:.0f} d2={d2_mm:.0f} "
            f"stable={self._stable_ok_count}/{self.stable_samples} ({reason})"
        )
