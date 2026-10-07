# skills/navigation/servo_to_range.py

from __future__ import annotations



from motion_backends.velocity import VelocityMotionBackend
from navigation.control.range_only_controller import (
    RangeOnlyController,
    RangeOnlyParams,
    RangeOnlyResult,
)
from navigation.command.velocity_arbiter import VelocityCommand
from primitives.base import Primitive, PrimitiveStatus


class ServoToRange(Primitive):
    """
    Execute closed-loop motion to a scalar range setpoint.

    This skill is deliberately agnostic to the source and meaning
    of the supplied range measurement.

    The caller owns:
        - selecting the range source;
        - converting it into the desired control frame;
        - deciding what physical object/surface the range represents;
        - deciding whether this skill is appropriate.

    This skill owns:
        - RangeOnlyController lifecycle;
        - conversion to VelocityCommand;
        - VelocityMotionBackend execution;
        - safe stop when no current range is available;
        - PrimitiveStatus lifecycle.
    """

    def __init__(
        self,
        *,
        config,
        target_range_mm: float,
        params: RangeOnlyParams = RangeOnlyParams(),
    ):
        super().__init__()

        self.config = config
        self.target_range_mm = float(target_range_mm)

        if self.target_range_mm < 0.0:
            raise ValueError(
                "target_range_mm must be >= 0"
            )

        self.controller = RangeOnlyController(params)

        self.velocity_backend = None
        self._io = None
        self._last_result: RangeOnlyResult | None = None

    @property
    def last_result(self) -> RangeOnlyResult | None:
        return self._last_result

    def start(
            self,
            *,
            lvl2,
            io,
            localisation=None,
            **_,
    ) -> PrimitiveStatus:

        self.controller.reset()
        self._io = io

        self.velocity_backend = VelocityMotionBackend(
            lvl2=lvl2,
            config=self.config,
            localisation=localisation,
            io=io,
        )

        self._last_result = None
        self.status = PrimitiveStatus.RUNNING

        print(
            "[SERVO_TO_RANGE] start "
            f"target={self.target_range_mm:.1f}mm"
        )

        return self.status

    def update(
        self,
        *,
        range_mm: float | None,
        **_,
    ) -> PrimitiveStatus:

        if self.velocity_backend is None:
            self.status = PrimitiveStatus.FAILED
            return self.status

        # A missing current reading is not automatically a controller
        # failure. Stop safely and let the caller decide whether to
        # wait, fall back, or abort.
        if range_mm is None:
            self.velocity_backend.stop()
            self._last_result = None
            self.status = PrimitiveStatus.RUNNING
            return self.status

        result = self.controller.update(
            range_mm=float(range_mm),
            target_range_mm=self.target_range_mm,
        )

        self._last_result = result

        if result.reached:
            self.velocity_backend.stop()
            self.status = PrimitiveStatus.SUCCEEDED
            return self.status

        command = VelocityCommand(
            linear_x_mps=result.linear_mps,
            angular_z_rps=0.0,
            lateral_y_mps=0.0,
            timestamp=float(self._io.time()),
        )

        self.velocity_backend.update(command)

        self.status = PrimitiveStatus.RUNNING
        return self.status

    def stop(
        self,
        **_,
    ) -> PrimitiveStatus:

        if self.velocity_backend is not None:
            self.velocity_backend.stop()

        self.controller.reset()
        self.velocity_backend = None

        self._last_result = None
        self._io = None
        self.status = PrimitiveStatus.SUCCEEDED
        return self.status


__all__ = [
    "ServoToRange",
]
