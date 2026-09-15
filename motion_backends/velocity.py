# motion_backends/velocity.py

from __future__ import annotations

from config import CONFIG
from calibration import CALIBRATION

from motion_backends.motor_output_conditioner import (
    MotorOutputConditioner,
)
from motion_backends.vel_diff_2wd import VelocityDiff2WD

from navigation.velocity_arbiter import VelocityCommand


class VelocityMotionBackend:
    """
    Executable velocity-motion backend.

    Accepts canonical robot velocity commands and sends the resulting
    conditioned motor powers to Level2.

    Flow:

        VelocityCommand
            ↓
        drivetrain-specific velocity conversion
            ↓
        MotorPowerCommand
            ↓
        MotorOutputConditioner
            ↓
        Level2.DRIVE_POWER

    The drivetrain-specific converters remain independent of hardware
    output and motor-output policy.
    """

    def __init__(
        self,
        *,
        lvl2,
        config=CONFIG,
        calibration=CALIBRATION,
    ):
        self.lvl2 = lvl2
        self.cfg = config
        self.cal = calibration

        self.velocity_backend = self._resolve_velocity_backend()

        self.output_conditioner = MotorOutputConditioner(
            config=config,
        )

    # --------------------------------------------------
    # Public API
    # --------------------------------------------------

    def update(
        self,
        command: VelocityCommand,
    ):
        """
        Apply one velocity command.

        The motor powers remain active until the next update() or
        stop() call.
        """

        raw_command = self.velocity_backend.update(
            command
        )

        conditioned_command = self.output_conditioner.condition(
            raw_command
        )

        self.lvl2.DRIVE_POWER(
            left_power=conditioned_command.front_left,
            right_power=conditioned_command.front_right,
        )

    def stop(self):
        """
        Stop drivetrain output immediately.
        """

        self.lvl2.DRIVE_STOP()

    # --------------------------------------------------
    # Backend selection
    # --------------------------------------------------

    def _resolve_velocity_backend(self):
        """
        Select the drivetrain-specific velocity converter.

        Additional layouts can be added as their velocity backends
        are implemented.
        """

        layout = str(self.cfg.drive_layout).upper()

        if layout == "2WD":
            return VelocityDiff2WD(
                config=self.cfg,
                calibration=self.cal,
            )

        raise RuntimeError(
            f"No velocity backend implemented for "
            f"DRIVE_LAYOUT={self.cfg.drive_layout!r}"
        )