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

        - motor polarity
        - maximum motor power

    Future responsibilities may include:

        - motor deadband
        - slew-rate limiting
        - output normalization
        - e-stop gating

    The conditioner does not decide what motion the robot should make.
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

        polarity = self.cfg.motor_polarity
        power_max = float(self.cfg.motor_power_max)

        # --------------------------------------------------
        # Front motors
        # --------------------------------------------------

        front_left = self._condition_motor(
            command.front_left,
            polarity[0],
            power_max,
        )

        front_right = self._condition_motor(
            command.front_right,
            polarity[1],
            power_max,
        )

        # --------------------------------------------------
        # Rear motors
        # --------------------------------------------------

        rear_left = None
        rear_right = None

        if command.rear_left is not None:
            if len(polarity) < 4:
                raise RuntimeError(
                    "Rear-left motor command supplied but "
                    "CONFIG.motor_polarity has fewer than 4 entries."
                )

            rear_left = self._condition_motor(
                command.rear_left,
                polarity[2],
                power_max,
            )

        if command.rear_right is not None:
            if len(polarity) < 4:
                raise RuntimeError(
                    "Rear-right motor command supplied but "
                    "CONFIG.motor_polarity has fewer than 4 entries."
                )

            rear_right = self._condition_motor(
                command.rear_right,
                polarity[3],
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
        polarity: float,
        power_max: float,
    ) -> float:
        """
        Apply polarity and final power clamp to one motor.
        """

        conditioned = float(power) * float(polarity)

        return max(
            -power_max,
            min(power_max, conditioned),
        )

    def _validate_config(self) -> None:
        """
        Validate the minimum configuration required by the conditioner.
        """

        polarity = self.cfg.motor_polarity

        if len(polarity) not in (2, 4):
            raise RuntimeError(
                "CONFIG.motor_polarity must contain either "
                "2 entries for 2WD or 4 entries for 4WD."
            )

        for value in polarity:
            if value not in (-1, 1):
                raise RuntimeError(
                    "CONFIG.motor_polarity entries must be -1 or +1."
                )

        power_max = float(self.cfg.motor_power_max)

        if not 0.0 < power_max <= 1.0:
            raise RuntimeError(
                "CONFIG.motor_power_max must be > 0.0 and <= 1.0."
            )