# motion_backends/vel_diff_2wd.py

from __future__ import annotations

from calibration import CALIBRATION
from config import CONFIG
from motion_backends.motor_output_conditioner import MotorPowerCommand
from navigation.velocity_arbiter import VelocityCommand


class VelocityDiff2WD:
    """
    Feed-forward velocity backend for a 2WD differential-drive robot.

    Input:
        VelocityCommand
            linear_x_mps
            angular_z_rps
            lateral_y_mps
            timestamp

    Output:
        MotorPowerCommand
            front_left
            front_right
            timestamp

    Responsibilities:
        1. Convert robot linear/angular velocity into left/right
           wheel velocities.
        2. Convert each wheel velocity into motor power using the
           calibrated DRIVE_VELOCITY_CURVE.

    This backend does NOT:
        - apply motor polarity;
        - enforce MOTOR_POWER_MAX;
        - apply deadband;
        - apply slew limiting;
        - handle e-stop;
        - write to hardware.

    Those belong downstream in MotorOutputConditioner / motor output.

    Reverse velocity currently assumes that the forward velocity
    calibration curve is symmetric.
    """

    def __init__(
        self,
        *,
        config=CONFIG,
        calibration=CALIBRATION,
    ):
        self.cfg = config
        self.cal = calibration

        self._velocity_curve = self._prepare_velocity_curve(
            self.cal.drive_velocity_curve
        )

    def update(
        self,
        command: VelocityCommand,
    ) -> MotorPowerCommand:
        """
        Convert one canonical robot velocity command into raw
        left/right motor-power commands.
        """

        # --------------------------------------------------
        # Differential drive cannot strafe
        # --------------------------------------------------

        if abs(command.lateral_y_mps) > 1e-9:
            raise ValueError(
                "VelocityDiff2WD does not support lateral dead_reckoning "
                f"(lateral_y_mps={command.lateral_y_mps})."
            )

        # --------------------------------------------------
        # Robot velocity -> wheel velocity
        # --------------------------------------------------

        linear_mm_s = command.linear_x_mps * 1000.0

        half_track_mm = (
            float(self.cfg.drive_track_width_mm) / 2.0
        )

        rotational_mm_s = (
            command.angular_z_rps * half_track_mm
        )

        # Differential-drive kinematics:
        #
        #     v_left  = v - omega * track_width / 2
        #     v_right = v + omega * track_width / 2

        left_velocity_mm_s = (
            linear_mm_s - rotational_mm_s
        )

        right_velocity_mm_s = (
            linear_mm_s + rotational_mm_s
        )

        # --------------------------------------------------
        # Wheel velocity -> motor power
        # --------------------------------------------------

        left_power = self._power_for_velocity_mm_s(
            left_velocity_mm_s
        )

        right_power = self._power_for_velocity_mm_s(
            right_velocity_mm_s
        )

        # --------------------------------------------------
        # Raw motor-power command
        # --------------------------------------------------

        return MotorPowerCommand(
            front_left=left_power,
            front_right=right_power,
            timestamp=command.timestamp,
        )

    # ==================================================
    # Velocity calibration
    # ==================================================

    @staticmethod
    def _prepare_velocity_curve(
        curve,
    ) -> tuple[tuple[float, float], ...]:
        """
        Convert DRIVE_VELOCITY_CURVE into lookup order:

            (velocity_mm_s, motor_power)

        The calibration profile stores:

            (motor_power, velocity_mm_s)

        A (0 velocity, 0 power) point is added automatically if one
        is not already present.
        """

        points = [
            (float(velocity_mm_s), float(power))
            for power, velocity_mm_s in curve
        ]

        if not points:
            raise RuntimeError(
                "DRIVE_VELOCITY_CURVE contains no calibration points."
            )

        points.sort(
            key=lambda point: point[0]
        )

        if points[0][0] > 0.0:
            points.insert(
                0,
                (0.0, 0.0),
            )

        return tuple(points)

    def _power_for_velocity_mm_s(
        self,
        velocity_mm_s: float,
    ) -> float:
        """
        Convert requested wheel velocity to raw motor power.

        Linear interpolation is used between calibration points.

        Requested velocities above the calibrated range are limited
        to the highest calibrated power rather than extrapolated.

        Reverse dead_reckoning uses the same curve with reversed sign.
        """

        if abs(velocity_mm_s) < 1e-9:
            return 0.0

        direction = (
            1.0
            if velocity_mm_s > 0.0
            else -1.0
        )

        requested_velocity = abs(
            velocity_mm_s
        )

        points = self._velocity_curve

        # --------------------------------------------------
        # At or above highest calibrated velocity
        # --------------------------------------------------

        highest_velocity, highest_power = points[-1]

        if requested_velocity >= highest_velocity:
            return direction * highest_power

        # --------------------------------------------------
        # Linear interpolation
        # --------------------------------------------------

        for index in range(1, len(points)):
            velocity_high, power_high = points[index]

            if requested_velocity <= velocity_high:
                velocity_low, power_low = points[index - 1]

                velocity_span = (
                    velocity_high - velocity_low
                )

                if velocity_span <= 0.0:
                    raise RuntimeError(
                        "DRIVE_VELOCITY_CURVE contains duplicate "
                        "or invalid velocity calibration points."
                    )

                fraction = (
                    requested_velocity - velocity_low
                ) / velocity_span

                power = (
                    power_low
                    + fraction
                    * (power_high - power_low)
                )

                return direction * power

        # Defensive fallback. Normally unreachable.
        return 0.0