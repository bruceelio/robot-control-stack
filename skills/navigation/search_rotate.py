# skills/navigation/search_rotate.py

from __future__ import annotations

import math
from typing import Any, Optional

from config import CONFIG
from motion_backends.velocity import VelocityMotionBackend
from navigation.command.velocity_arbiter import VelocityCommand
from primitives.base import Primitive, PrimitiveStatus


class SearchRotate(Primitive):
    """
    Generic continuous rotational search primitive.

    SearchRotate owns only the rotational search motion.

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

    Rotation direction comes from target_angle_deg:
      positive -> positive angular_z
      negative -> negative angular_z

    Rotation speed is always taken from:
      config.servoing_angular_max_rad_s

    The requested angular sweep is bounded using commanded angular
    speed and io.time(). No localisation or heading estimate is
    required.

    Returns:
      RUNNING
        while the search is active

      SUCCEEDED
        as soon as found_item is not None

      FAILED
        if the requested angular sweep or timeout is exhausted
    """

    def __init__(
        self,
        *,
        target_angle_deg: float,
        timeout_s: float,
        config=CONFIG,
        label: str = "SEARCH_ROTATE",
    ):
        super().__init__()

        self.target_angle_deg = float(target_angle_deg)
        self.timeout_s = float(timeout_s)
        self.config = config
        self.label = label

        self._io = None
        self._velocity_backend: Optional[VelocityMotionBackend] = None

        self._start_time: Optional[float] = None
        self._target_duration_s: Optional[float] = None
        self._angular_speed_rad_s: Optional[float] = None

        self.found_item: Any = None
        self._status = PrimitiveStatus.RUNNING

    def start(self, *, motion_backend, **_):
        self._io = motion_backend.lvl2.io

        self._velocity_backend = None
        self._start_time = None
        self._target_duration_s = None
        self._angular_speed_rad_s = None

        self.found_item = None
        self._status = PrimitiveStatus.RUNNING

        if abs(self.target_angle_deg) < 1e-6:
            print(
                f"[{self.label}] "
                "invalid target_angle_deg -> FAILED"
            )
            self._status = PrimitiveStatus.FAILED
            return self._status

        self._angular_speed_rad_s = abs(
            float(self.config.servoing_angular_max_rad_s)
        )

        if self._angular_speed_rad_s <= 0.0:
            print(
                f"[{self.label}] "
                "invalid servoing_angular_max_rad_s -> FAILED"
            )
            self._status = PrimitiveStatus.FAILED
            return self._status

        self._target_duration_s = (
            math.radians(abs(self.target_angle_deg))
            / self._angular_speed_rad_s
        )

        self._velocity_backend = VelocityMotionBackend(
            lvl2=motion_backend.lvl2,
            config=self.config,
        )

        self._start_time = float(self._io.time())

        print(
            f"[{self.label}] start "
            f"target={self.target_angle_deg:+.1f}deg "
            f"speed={self._signed_angular_speed():+.3f}rad/s "
            f"duration={self._target_duration_s:.2f}s "
            f"timeout={self.timeout_s:.2f}s"
        )

        self._command_rotation(
            now=self._start_time,
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
            self._stop_rotation()

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
            self._stop_rotation()

            print(
                f"[{self.label}] "
                "timeout -> FAILED"
            )

            self._status = PrimitiveStatus.FAILED
            return self._status

        # -------------------------
        # Requested angular sweep complete
        # -------------------------
        if (
            self._start_time is not None
            and self._target_duration_s is not None
            and (now - self._start_time) >= self._target_duration_s
        ):
            self._stop_rotation()

            print(
                f"[{self.label}] "
                f"search sweep complete "
                f"target={self.target_angle_deg:+.1f}deg "
                "-> FAILED"
            )

            self._status = PrimitiveStatus.FAILED
            return self._status

        # Refresh the velocity request while the search remains active.
        self._command_rotation(now=now)

        return self._status

    def _signed_angular_speed(self) -> float:
        if self._angular_speed_rad_s is None:
            return 0.0

        return (
            self._angular_speed_rad_s
            if self.target_angle_deg > 0.0
            else -self._angular_speed_rad_s
        )

    def _command_rotation(self, *, now: float):
        if self._velocity_backend is None:
            return

        command = VelocityCommand(
            linear_x_mps=0.0,
            angular_z_rps=self._signed_angular_speed(),
            lateral_y_mps=0.0,
            timestamp=now,
        )

        self._velocity_backend.update(command)

    def _stop_rotation(self):
        if self._velocity_backend is None:
            return

        try:
            self._velocity_backend.stop()
        except Exception:
            pass

    def stop(
        self,
        *,
        motion_backend=None,
        **_,
    ):
        self._stop_rotation()

        self._status = PrimitiveStatus.FAILED
        return self._status
