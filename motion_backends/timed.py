# motion_backends/timed.py

from motion_backends.motor_output_conditioner import (
    MotorOutputConditioner,
    MotorPowerCommand,
)


class TimedMotionBackend:
    """
    Timed (open-loop) dead_reckoning backend.

    Uses:
        - resolved Config (policy, limits)
        - resolved Calibration (physical truth)

    Produces raw motor-power commands which are passed through the
    MotorOutputConditioner before reaching Level2 / hardware.
    """

    def __init__(self, lvl2, config, calibration):
        self.lvl2 = lvl2
        self.cfg = config
        self.cal = calibration

        self.output_conditioner = MotorOutputConditioner(
            config=config,
        )

        self.localisation = None
        self.now_s = None
        self._filtered_battery_voltage = None

    # --------------------------------------------------
    # Voltage compensation
    # --------------------------------------------------

    def _battery_voltage_scale(
        self,
        *,
        power: float,
        motion_kind: str,
    ) -> float:

        reference_v = self.cal.voltage_reference

        try:
            actual_v = self.lvl2.io.voltage["battery"].volts
        except (AttributeError, KeyError):
            actual_v = None

        if actual_v is None or actual_v <= 1.0:
            actual_v = reference_v

        alpha = 0.2

        if self._filtered_battery_voltage is None:
            self._filtered_battery_voltage = actual_v
        else:
            self._filtered_battery_voltage = (
                alpha * actual_v
                + (1.0 - alpha)
                * self._filtered_battery_voltage
            )

        dv = (
            self._filtered_battery_voltage
            - reference_v
        )

        p = abs(power)

        if p <= self.cal.drive_power_short:
            model = self.cal.voltage_low_model
            a = self.cal.voltage_low_a
            b = self.cal.voltage_low_b
        else:
            model = self.cal.voltage_high_model
            a = self.cal.voltage_high_a
            b = self.cal.voltage_high_b

        if model == "linear":
            scale = 1.0 + a * dv

        elif model == "quadratic":
            scale = 1.0 + a * dv + b * dv ** 2

        elif model == "exponential":
            scale = (
                reference_v
                / self._filtered_battery_voltage
            ) ** a

        else:
            raise RuntimeError(
                f"Unknown voltage compensation model: {model}"
            )

        # Voltage compensation itself has sensible bounds.
        # Final motor-power limits are applied downstream by the
        # MotorOutputConditioner.
        scale = min(
            1.50,
            max(0.8, scale),
        )

        print(
            f"[BATTERY_COMP] kind={motion_kind} "
            f"raw={actual_v:.2f}V "
            f"filtered={self._filtered_battery_voltage:.2f}V "
            f"reference={reference_v:.2f}V "
            f"scale={scale:.3f}"
        )

        return scale

    @staticmethod
    def _apply_voltage_compensation(
        power: float,
        scale: float,
    ) -> float:
        """
        Apply voltage compensation only.

        Final motor-power limiting is owned by
        MotorOutputConditioner.
        """
        return power * scale

    # --------------------------------------------------
    # Motor output
    # --------------------------------------------------

    def _run(
        self,
        left: float,
        right: float,
        duration: float,
        *,
        motion_kind: str,
    ):
        if duration <= 0.0:
            return

        # --------------------------------------------------
        # Voltage compensation
        # --------------------------------------------------

        scale = self._battery_voltage_scale(
            power=max(
                abs(left),
                abs(right),
            ),
            motion_kind=motion_kind,
        )

        left = self._apply_voltage_compensation(
            left,
            scale,
        )

        right = self._apply_voltage_compensation(
            right,
            scale,
        )

        # --------------------------------------------------
        # Raw motor-power command
        # --------------------------------------------------

        raw_command = MotorPowerCommand(
            front_left=left,
            front_right=right,
        )

        # --------------------------------------------------
        # Final motor-output conditioning
        # --------------------------------------------------

        command = self.output_conditioner.condition(
            raw_command
        )

        # --------------------------------------------------
        # Existing blocking Level2 execution
        # --------------------------------------------------

        self.lvl2.DRIVE(
            command.front_left,
            command.front_right,
            duration,
        )

    # --------------------------------------------------
    # Primitive compatibility
    # --------------------------------------------------

    def is_busy(self) -> bool:
        """
        Timed backend is blocking.

        When drive() or rotate() returns, dead_reckoning is complete.
        """
        return False

    def stop(self):
        raw_command = MotorPowerCommand(
            front_left=0.0,
            front_right=0.0,
        )

        command = self.output_conditioner.condition(
            raw_command
        )

        self.lvl2.DRIVE_STOP()

    # --------------------------------------------------
    # Duration estimation
    # --------------------------------------------------

    def estimate_drive_duration(
        self,
        *,
        distance_mm: float,
    ) -> tuple[float, float]:
        """
        Return (clamped_distance_mm, expected_duration_s).

        Expected duration is the calibrated nominal time adjusted by
        the resolved drive_factor for the current
        robot/environment/surface.
        """

        if abs(distance_mm) < self.cfg.min_drive_mm:
            return 0.0, 0.0

        d = max(
            -self.cfg.max_drive_mm,
            min(
                self.cfg.max_drive_mm,
                distance_mm,
            ),
        )

        abs_d = abs(d)

        if abs_d < self.cal.drive_switch_mm:
            m = self.cal.drive_m_short
            b = self.cal.drive_b_short
        else:
            m = self.cal.drive_m_long
            b = self.cal.drive_b_long

        duration = (
            m * abs_d + b
        ) * self.cfg.drive_factor

        return d, duration

    def estimate_rotate_duration(
        self,
        *,
        angle_deg: float,
    ) -> tuple[float, float]:
        """
        Return (clamped_angle_deg, expected_duration_s).

        Expected duration is the calibrated nominal time adjusted by
        the resolved rotate_factor for the current
        robot/environment/surface.
        """

        if abs(angle_deg) < self.cfg.min_rotate_deg:
            return 0.0, 0.0

        a = max(
            -self.cfg.max_rotate_deg,
            min(
                self.cfg.max_rotate_deg,
                angle_deg,
            ),
        )

        abs_a = abs(a)

        if abs_a < self.cal.rotate_switch_deg:
            m = self.cal.rotate_m_small
            b = self.cal.rotate_b_small
        else:
            m = self.cal.rotate_m_large
            b = self.cal.rotate_b_large

        duration = (
            m * abs_a + b
        ) * self.cfg.rotate_factor

        return a, duration

    # --------------------------------------------------
    # Public API
    # --------------------------------------------------

    def drive(
        self,
        distance_mm: float,
    ):
        d, duration = self.estimate_drive_duration(
            distance_mm=distance_mm
        )

        if duration <= 0.0:
            return

        abs_d = abs(d)
        direction = (
            1.0
            if d > 0.0
            else -1.0
        )

        if abs_d < self.cal.drive_switch_mm:
            power = self.cal.drive_power_short
        else:
            power = self.cal.drive_power_long

        left = direction * power
        right = direction * power

        print(
            f"[TIMED] DRIVE d={d:.1f}mm "
            f"p={power:.2f} "
            f"t={duration:.3f}s"
        )

        self._run(
            left,
            right,
            duration,
            motion_kind="drive",
        )

    def rotate(
        self,
        angle_deg: float,
    ):
        # angle_deg remains logical for estimation/localisation.
        a, duration = self.estimate_rotate_duration(
            angle_deg=angle_deg
        )

        if duration <= 0.0:
            return

        abs_a = abs(a)

        direction = (
            1.0
            if a > 0.0
            else -1.0
        )

        if abs_a < self.cal.rotate_switch_deg:
            power = self.cal.rotate_power_small
        else:
            power = self.cal.rotate_power_large

        # Canonical rotation convention:
        #   positive angle = left / counter-clockwise
        #   negative angle = right / clockwise
        left = -direction * power
        right = direction * power

        print(
            f"[TIMED] ROTATE logical={a:.1f}deg "
            f"p={power:.2f} t={duration:.3f}s"
        )

        self._run(
            left,
            right,
            duration,
            motion_kind="rotate",
        )