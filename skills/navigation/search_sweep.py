# skills/navigation/search_sweep.py

from __future__ import annotations

import math
from typing import Any, Optional, Sequence

from config import CONFIG
from primitives.base import Primitive, PrimitiveStatus
from skills.navigation.search_rotate import SearchRotate


class SearchSweep(Primitive):
    """
    Generic continuous rotational sweep pattern.

    SearchSweep composes one or more SearchRotate legs.

    It owns only the physical search pattern. It does NOT know:
      - what is being searched for
      - how perception works
      - object kinds
      - tag IDs
      - exclusions
      - localisation policy

    The caller evaluates its own search condition and passes the
    current result into update() as found_item.

    Example centred sweep:

        angles_deg=(
            +30.0,   # centre -> +30
            -60.0,   # +30 -> -30
            +30.0,   # -30 -> centre
        )

    Each angle is relative to the end of the previous leg.

    Search motion is continuous within each leg. Direction changes
    only between legs.

    Returns:
      RUNNING
        while the sweep is active

      SUCCEEDED
        as soon as found_item is not None

      FAILED
        when all sweep legs are exhausted without finding anything,
        or when the overall timeout is exhausted
    """

    def __init__(
        self,
        *,
        angles_deg: Sequence[float],
        timeout_s: float,
        config=CONFIG,
        label: str = "SEARCH_SWEEP",
    ):
        super().__init__()

        self.angles_deg = tuple(
            float(angle_deg)
            for angle_deg in angles_deg
            if abs(float(angle_deg)) >= 1e-6
        )

        self.timeout_s = float(timeout_s)
        self.config = config
        self.label = label

        self._io = None

        self._start_time: Optional[float] = None

        self._leg_index = 0
        self._leg: Optional[SearchRotate] = None

        self.found_item: Any = None

        self._status = PrimitiveStatus.RUNNING

    def start(
        self,
        *,
        motion_backend,
        **_,
    ):
        self._io = motion_backend.lvl2.io

        self._start_time = float(
            self._io.time()
        )

        self._leg_index = 0
        self._leg = None

        self.found_item = None
        self._status = PrimitiveStatus.RUNNING

        if not self.angles_deg:
            print(
                f"[{self.label}] "
                "no sweep angles -> FAILED"
            )

            self._status = PrimitiveStatus.FAILED
            return self._status

        angular_speed_rad_s = abs(
            float(
                self.config.servoing_angular_max_rad_s
            )
        )

        if angular_speed_rad_s <= 0.0:
            print(
                f"[{self.label}] "
                "invalid servoing_angular_max_rad_s "
                "-> FAILED"
            )

            self._status = PrimitiveStatus.FAILED
            return self._status

        print(
            f"[{self.label}] start "
            f"legs={self.angles_deg} "
            f"speed={angular_speed_rad_s:.3f}rad/s "
            f"timeout={self.timeout_s:.2f}s"
        )

        return self._start_next_leg(
            motion_backend=motion_backend,
        )

    def update(
        self,
        *,
        motion_backend,
        found_item=None,
        **_,
    ):
        if self._status != PrimitiveStatus.RUNNING:
            return self._status

        now = float(
            self._io.time()
        )

        if (
            self._start_time is not None
            and self.timeout_s > 0.0
            and (now - self._start_time)
            >= self.timeout_s
        ):
            self._stop_leg(
                motion_backend=motion_backend,
            )

            print(
                f"[{self.label}] "
                "timeout -> FAILED"
            )

            self._status = PrimitiveStatus.FAILED
            return self._status

        if self._leg is None:
            print(
                f"[{self.label}] "
                "missing active leg -> FAILED"
            )

            self._status = PrimitiveStatus.FAILED
            return self._status

        st = self._leg.update(
            motion_backend=motion_backend,
            found_item=found_item,
        )

        if st == PrimitiveStatus.RUNNING:
            return self._status

        if st == PrimitiveStatus.SUCCEEDED:
            self.found_item = self._leg.found_item

            self._leg = None

            print(
                f"[{self.label}] "
                "found -> SUCCEEDED"
            )

            self._status = PrimitiveStatus.SUCCEEDED
            return self._status

        # A SearchRotate FAILED result means this leg has exhausted
        # its requested angular search without finding the item.
        self._leg = None
        self._leg_index += 1

        if self._leg_index >= len(
            self.angles_deg
        ):
            print(
                f"[{self.label}] "
                "sweep complete -> FAILED"
            )

            self._status = PrimitiveStatus.FAILED
            return self._status

        return self._start_next_leg(
            motion_backend=motion_backend,
        )

    def _start_next_leg(
        self,
        *,
        motion_backend,
    ):
        angle_deg = float(
            self.angles_deg[
                self._leg_index
            ]
        )

        angular_speed_rad_s = abs(
            float(
                self.config.servoing_angular_max_rad_s
            )
        )

        leg_timeout_s = (
            math.radians(
                abs(angle_deg)
            )
            / angular_speed_rad_s
            + 1.0
        )

        print(
            f"[{self.label}] "
            f"leg="
            f"{self._leg_index + 1}/"
            f"{len(self.angles_deg)} "
            f"angle={angle_deg:+.1f}deg"
        )

        self._leg = SearchRotate(
            target_angle_deg=angle_deg,
            timeout_s=leg_timeout_s,
            config=self.config,
            label=(
                f"{self.label}"
                f"][LEG_{self._leg_index + 1}"
            ),
        )

        st = self._leg.start(
            motion_backend=motion_backend,
        )

        if st == PrimitiveStatus.FAILED:
            self._leg = None
            self._status = PrimitiveStatus.FAILED

        return self._status

    def _stop_leg(
        self,
        *,
        motion_backend=None,
    ):
        if self._leg is None:
            return

        try:
            self._leg.stop(
                motion_backend=motion_backend,
            )
        except Exception:
            pass

        self._leg = None

    def stop(
        self,
        *,
        motion_backend=None,
        **_,
    ):
        self._stop_leg(
            motion_backend=motion_backend,
        )

        self._status = PrimitiveStatus.FAILED
        return self._status