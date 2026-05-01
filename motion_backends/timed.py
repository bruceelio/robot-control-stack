# motion_backends/timed.py

import time


class TimedMotionBackend:
    """
    Timed (open-loop) motion backend.

    Uses:
    - resolved Config (policy, limits)
    - resolved Calibration (physical truth)
    """

    def __init__(self, lvl2, config, calibration):
        self.lvl2 = lvl2
        self.cfg = config
        self.cal = calibration
        self.localisation = None
        self.now_s = None

    # --------------------------------------------------
    # Internal helper
    # --------------------------------------------------

    def _run(self, left: float, right: float, duration: float):
        if duration <= 0:
            return

        # Level2 owns hardware + timing semantics
        self.lvl2.DRIVE(left, right, duration)

    # --------------------------------------------------
    # Primitive compatibility
    # --------------------------------------------------

    def is_busy(self) -> bool:
        """
        Timed backend is blocking.
        When a command returns, motion is complete.
        """
        return False


    # --------------------------------------------------
    # Public API
    # --------------------------------------------------

    def estimate_drive_duration(self, *, distance_mm: float) -> tuple[float, float]:
        """
        Return (clamped_distance_mm, expected_duration_s).

        Expected duration is the calibrated nominal time adjusted by the
        resolved drive_factor for the current robot/environment/surface.
        """
        if abs(distance_mm) < self.cfg.min_drive_mm:
            return 0.0, 0.0

        d = max(
            -self.cfg.max_drive_mm,
            min(self.cfg.max_drive_mm, distance_mm),
        )

        abs_d = abs(d)

        if abs_d < self.cal.drive_switch_mm:
            m = self.cal.drive_m_short
            b = self.cal.drive_b_short
        else:
            m = self.cal.drive_m_long
            b = self.cal.drive_b_long

        duration = (m * abs_d + b) * self.cfg.drive_factor
        return d, duration

    def estimate_rotate_duration(self, *, angle_deg: float) -> tuple[float, float]:
        """
        Return (clamped_angle_deg, expected_duration_s).

        Expected duration is the calibrated nominal time adjusted by the
        resolved rotate_factor for the current robot/environment/surface.
        """
        if abs(angle_deg) < self.cfg.min_rotate_deg:
            return 0.0, 0.0

        a = max(
            -self.cfg.max_rotate_deg,
            min(self.cfg.max_rotate_deg, angle_deg),
        )

        abs_a = abs(a)

        if abs_a < self.cal.rotate_switch_deg:
            m = self.cal.rotate_m_small
            b = self.cal.rotate_b_small
        else:
            m = self.cal.rotate_m_large
            b = self.cal.rotate_b_large

        duration = (m * abs_a + b) * self.cfg.rotate_factor
        return a, duration

    def drive(self, distance_mm: float):
        d, duration = self.estimate_drive_duration(distance_mm=distance_mm)
        if duration <= 0.0:
            return

        abs_d = abs(d)
        direction = 1.0 if d > 0 else -1.0

        if abs_d < self.cal.drive_switch_mm:
            power = self.cal.drive_power_short
        else:
            power = self.cal.drive_power_long

        left = direction * power
        right = direction * power

        print(
            f"[TIMED] DRIVE d={d:.1f}mm "
            f"p={power:.2f} t={duration:.3f}s"
        )

        self._run(left, right, duration)

    def rotate(self, angle_deg: float):
        # angle_deg stays logical for estimate/localisation
        a, duration = self.estimate_rotate_duration(angle_deg=angle_deg)
        if duration <= 0.0:
            return

        abs_a = abs(a)
        direction = 1.0 if a > 0 else -1.0

        if abs_a < self.cal.rotate_switch_deg:
            power = self.cal.rotate_power_small
        else:
            power = self.cal.rotate_power_large

        motor_direction = self.cfg.rotation_sign * direction

        left = motor_direction * power
        right = -motor_direction * power

        print(
            f"[TIMED] ROTATE logical={a:.1f}deg "
            f"motor_sign={self.cfg.rotation_sign:+d} "
            f"p={power:.2f} t={duration:.3f}s"
        )

        self._run(left, right, duration)
