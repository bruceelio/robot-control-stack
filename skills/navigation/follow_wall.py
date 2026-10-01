# skills/navigation/follow_wall.py

from __future__ import annotations

import time

from motion_backends.velocity import VelocityMotionBackend
from navigation.command.velocity_arbiter import VelocityCommand
from navigation.wall_geometry.models import WallGeometry
from navigation.wall_following.models import (
    WallFollowResult,
    WallSide,
)
from navigation.wall_following.wall_follower import (
    WallFollower,
    WallFollowMode,
)
from primitives.base import Primitive, PrimitiveStatus


class FollowWall(Primitive):
    """
    Execute continuous wall-following motion.

    This skill does not acquire sensor measurements itself.

    The caller supplies semantic WallGeometry. The underlying
    navigation.wall_following package decides which available
    wall-following algorithm can use that geometry.

    The skill owns:

        - wall-following controller lifecycle;
        - conversion to canonical VelocityCommand;
        - velocity-backend execution;
        - safe stop when wall geometry is unavailable.

    It does not own:

        - sensor selection;
        - ultrasonic / ToF access;
        - wall-geometry estimation;
        - route progress;
        - mission completion.

    Current wall-following controllers assume forward motion.
    """

    def __init__(
        self,
        *,
        config,
        wall_side: WallSide | str,
        desired_distance_mm: float,
        linear_x_mps: float,
    ):
        super().__init__()

        self.config = config

        self.wall_side = (
            wall_side
            if isinstance(wall_side, WallSide)
            else WallSide(str(wall_side).lower())
        )

        self.desired_distance_mm = float(
            desired_distance_mm
        )

        self.linear_x_mps = float(
            linear_x_mps
        )

        if self.desired_distance_mm <= 0.0:
            raise ValueError(
                "desired_distance_mm must be positive"
            )

        if self.linear_x_mps < 0.0:
            raise ValueError(
                "FollowWall currently supports forward motion only"
            )

        self.controller = WallFollower()

        self.velocity_backend = None

        self._last_update_monotonic_s = None
        self._last_mode: WallFollowMode | None = None
        self._last_result: WallFollowResult | None = None

    @property
    def active_mode(self) -> WallFollowMode | None:
        return self.controller.active_mode

    @property
    def last_result(self) -> WallFollowResult | None:
        return self._last_result

    def start(
        self,
        *,
        lvl2,
        **_,
    ) -> PrimitiveStatus:

        self.controller.reset()

        self.velocity_backend = VelocityMotionBackend(
            lvl2=lvl2,
            config=self.config,
        )

        self._last_update_monotonic_s = None
        self._last_mode = None
        self._last_result = None

        self.status = PrimitiveStatus.RUNNING

        print(
            "[FOLLOW_WALL] start "
            f"side={self.wall_side.value} "
            f"distance={self.desired_distance_mm:.0f}mm "
            f"vx={self.linear_x_mps:.3f}m/s"
        )

        return self.status

    def update(
        self,
        *,
        wall: WallGeometry,
        **_,
    ) -> PrimitiveStatus:

        if self.velocity_backend is None:
            self.status = PrimitiveStatus.FAILED
            return self.status

        now_monotonic_s = time.monotonic()

        if self._last_update_monotonic_s is None:
            # The distance-only controller does not use dt on
            # its first sample because no previous range exists.
            dt_s = 1.0
        else:
            dt_s = max(
                now_monotonic_s
                - self._last_update_monotonic_s,
                1e-6,
            )

        self._last_update_monotonic_s = (
            now_monotonic_s
        )

        result = self.controller.update(
            wall=wall,
            wall_side=self.wall_side,
            desired_distance_mm=self.desired_distance_mm,
            linear_x_mps=self.linear_x_mps,
            dt_s=dt_s,
        )

        self._last_result = result

        if result is None:
            self.velocity_backend.stop()

            if self._last_mode is not None:
                print(
                    "[FOLLOW_WALL] "
                    "wall geometry unavailable -> stop"
                )

            self._last_mode = None
            self.status = PrimitiveStatus.RUNNING
            return self.status

        mode = self.controller.active_mode

        if mode != self._last_mode:
            print(
                "[FOLLOW_WALL] "
                f"mode={mode.value}"
            )
            self._last_mode = mode

        command = VelocityCommand(
            linear_x_mps=(
                result.command.linear_x_mps
            ),
            angular_z_rps=(
                result.command.angular_z_rps
            ),
            lateral_y_mps=0.0,
            timestamp=time.time(),
        )

        self.velocity_backend.update(
            command
        )

        self.status = PrimitiveStatus.RUNNING
        return self.status

    def stop(
            self,
            **_,
    ):
        """
        Stop wall-following motion without declaring
        mission-level success.

        The owning behavior decides why wall following
        ended and what navigation mode follows next.
        """

        if self.velocity_backend is not None:
            self.velocity_backend.stop()

        self.controller.reset()

        self.velocity_backend = None

        self._last_update_monotonic_s = None
        self._last_mode = None
        self._last_result = None