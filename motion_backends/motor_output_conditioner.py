# motion_backends/motor_output_conditioner.py

from __future__ import annotations

from dataclasses import dataclass

from config import CONFIG


# ==================================================
# Motor power command
# ==================================================

@dataclass(frozen=True)
class MotorPowerCommand:
    """
    Canonical drivetrain motor-power command.

    Power values are normalized to:

        -1.0 ... 0.0 ... +1.0

    Rear motor values are None for 2WD robots.

    timestamp:
        Optional timestamp propagated from the command source.
        For example, servoing may propagate the perception timestamp.
    """

    front_left: float
    front_right: float

    rear_left: float | None = None
    rear_right: float | None = None

    timestamp: float | None = None


# ==================================================
# Motor output conditioner
# ==================================================

class MotorOutputConditioner:
    """
    Applies final drivetrain output constraints before motor commands
    are sent to hardware.

    Current responsibilities:

        - maximum motor power

    Future responsibilities may include:

        - motor deadband
        - slew-rate limiting
        - output normalization
        - e-stop gating

    The conditioner does not decide what dead_reckoning the robot should make.
    It only determines what motor output is permitted.
    """

    def __init__(
        self,
        *,
        config=CONFIG,
    ):
        self.cfg = config

        self._validate_config()

    def condition(
        self,
        command: MotorPowerCommand,
    ) -> MotorPowerCommand:
        """
        Apply configured motor-output conditioning.
        """

        power_min = float(self.cfg.motor_power_min)
        power_max = float(self.cfg.motor_power_max)

        front_left = self._condition_motor(
            command.front_left,
            power_min,
            power_max,
        )

        front_right = self._condition_motor(
            command.front_right,
            power_min,
            power_max,
        )

        # --------------------------------------------------
        # Rear motors
        # --------------------------------------------------

        rear_left = None
        rear_right = None

        if command.rear_left is not None:
            rear_left = self._condition_motor(
                command.rear_left,
                power_min,
                power_max,
            )

        if command.rear_right is not None:
            rear_right = self._condition_motor(
                command.rear_right,
                power_min,
                power_max,
            )

        return MotorPowerCommand(
            front_left=front_left,
            front_right=front_right,
            rear_left=rear_left,
            rear_right=rear_right,
            timestamp=command.timestamp,
        )

    # ==================================================
    # Helpers
    # ==================================================

    @staticmethod
    def _condition_motor(
            power: float,
            power_min: float,
            power_max: float,
    ) -> float:

        power = float(power)

        # Exact zero means stop.
        if power == 0.0:
            return 0.0

        # A non-zero command must be large enough to overcome
        # drivetrain static friction / motor deadband.
        if abs(power) < power_min:
            power = power_min if power > 0.0 else -power_min

        return max(
            -power_max,
            min(power_max, power),
        )

    def _validate_config(self) -> None:
        """
        Validate the minimum configuration required by the conditioner.
        """

        power_max = float(self.cfg.motor_power_max)

        if not 0.0 < power_max <= 1.0:
            raise RuntimeError(
                "CONFIG.motor_power_max must be > 0.0 and <= 1.0."
            )

        power_min = float(self.cfg.motor_power_min)

        if not 0.0 <= power_min <= power_max:
            raise RuntimeError(
                "CONFIG.motor_power_min must be >= 0.0 "
                "and <= CONFIG.motor_power_max."
            )