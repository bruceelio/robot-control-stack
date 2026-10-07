# skills/navigation/search_rotate.py

from __future__ import annotations


from typing import Any, Optional

from primitives.base import Primitive, PrimitiveStatus
from primitives.motion import Rotate


class SearchRotate(Primitive):
    """
    Generic stepped rotational search pattern.

    SearchRotate owns only the search motion.

    It does NOT know:
      - what is being searched for
      - how perception works
      - object kinds
      - tag IDs
      - exclusions
      - localisation policy

    The caller evaluates its own search condition and passes the
    current result into update() as found_item.

    found_item may therefore be:
      - an object observation
      - an arena marker
      - a localisation landmark
      - some future vision result
      - any other caller-defined object

    Returns:
      RUNNING
        while the search is active

      SUCCEEDED
        as soon as found_item is not None

      FAILED
        if the rotation budget or timeout is exhausted,
        or if a rotation primitive fails
    """

    def __init__(
        self,
        *,
        step_deg: float,
        max_deg: float,
        timeout_s: float,
        settle_s: float = 0.0,
        label: str = "SEARCH_ROTATE",
    ):
        super().__init__()

        self.step_deg = float(step_deg)
        self.max_deg = abs(float(max_deg))
        self.timeout_s = float(timeout_s)
        self.settle_s = max(0.0, float(settle_s))
        self.label = label

        self._rotated_deg = 0.0
        self._child: Optional[Rotate] = None
        self._start_time: Optional[float] = None
        self._settle_until: Optional[float] = None
        self._io = None

        self.found_item: Any = None
        self._status = PrimitiveStatus.RUNNING

    def start(self, *, motion_backend, **_):
        self._io = motion_backend.lvl2.io
        self._rotated_deg = 0.0
        self._child = None
        self._start_time = float(self._io.time())
        self._settle_until = None

        self.found_item = None
        self._status = PrimitiveStatus.RUNNING

        if abs(self.step_deg) < 1e-6:
            print(
                f"[{self.label}] "
                "invalid step_deg -> FAILED"
            )
            self._status = PrimitiveStatus.FAILED
            return self._status

        if self.max_deg <= 0.0:
            print(
                f"[{self.label}] "
                "invalid max_deg -> FAILED"
            )
            self._status = PrimitiveStatus.FAILED
            return self._status

        print(
            f"[{self.label}] start "
            f"step={self.step_deg:+.1f}deg "
            f"max={self.max_deg:.1f}deg "
            f"timeout={self.timeout_s:.2f}s "
            f"settle={self.settle_s:.2f}s"
        )

        return self._status

    def update(
        self,
        *,
        motion_backend,
        found_item=None,
        **_,
    ):
        if self._status != PrimitiveStatus.RUNNING:
            return self._status

        now = float(self._io.time())

        # -------------------------
        # Search condition satisfied
        # -------------------------
        if found_item is not None:
            self.found_item = found_item

            self._stop_child(
                motion_backend=motion_backend,
            )

            print(
                f"[{self.label}] "
                "found -> SUCCEEDED"
            )

            self._status = PrimitiveStatus.SUCCEEDED
            return self._status

        # -------------------------
        # Timeout
        # -------------------------
        if (
            self._start_time is not None
            and self.timeout_s > 0.0
            and (now - self._start_time) >= self.timeout_s
        ):
            self._stop_child(
                motion_backend=motion_backend,
            )

            print(
                f"[{self.label}] "
                "timeout -> FAILED"
            )

            self._status = PrimitiveStatus.FAILED
            return self._status

        # -------------------------
        # Settle after rotation
        # -------------------------
        if self._settle_until is not None:

            if now < self._settle_until:
                return self._status

            self._settle_until = None

        # -------------------------
        # Active rotation
        # -------------------------
        if self._child is not None:

            st = self._child.update(
                motion_backend=motion_backend,
            )

            if st == PrimitiveStatus.RUNNING:
                return self._status

            if st == PrimitiveStatus.FAILED:
                self._child = None

                print(
                    f"[{self.label}] "
                    "rotate failed -> FAILED"
                )

                self._status = PrimitiveStatus.FAILED
                return self._status

            # Rotation completed.
            self._child = None

            if self.settle_s > 0.0:
                self._settle_until = (
                        now + self.settle_s
                )

            return self._status

        # -------------------------
        # Rotation budget exhausted
        # -------------------------
        remaining_deg = (
            self.max_deg - self._rotated_deg
        )

        if remaining_deg <= 1e-6:
            print(
                f"[{self.label}] "
                "search sweep complete -> FAILED"
            )

            self._status = PrimitiveStatus.FAILED
            return self._status

        # -------------------------
        # Start next rotation step
        # -------------------------
        step_mag = min(
            abs(self.step_deg),
            remaining_deg,
        )

        angle = (
            step_mag
            if self.step_deg > 0.0
            else -step_mag
        )

        self._rotated_deg += abs(angle)

        print(
            f"[{self.label}] "
            f"rotate={angle:+.1f}deg "
            f"sweep={self._rotated_deg:.1f}/"
            f"{self.max_deg:.1f}deg"
        )

        self._child = Rotate(
            angle_deg=angle,
        )

        self._child.start(
            motion_backend=motion_backend,
        )

        return self._status

    def _stop_child(
        self,
        *,
        motion_backend,
    ):
        if self._child is None:
            return

        try:
            self._child.stop(
                motion_backend=motion_backend,
            )
        except Exception:
            pass

        self._child = None

    def stop(
        self,
        *,
        motion_backend=None,
        **_,
    ):
        self._settle_until = None

        if (
            self._child is not None
            and motion_backend is not None
        ):
            self._stop_child(
                motion_backend=motion_backend,
            )
        else:
            self._child = None

        self._status = PrimitiveStatus.FAILED