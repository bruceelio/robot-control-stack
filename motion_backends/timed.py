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

    def drive(self, distance_mm: float):
        if abs(distance_mm) < self.cfg.min_drive_mm:
            return

        d = max(
            -self.cfg.max_drive_mm,
            min(self.cfg.max_drive_mm, distance_mm),
        )

        abs_d = abs(d)
        direction = 1.0 if d > 0 else -1.0

        if abs_d < self.cal.drive_switch_mm:
            power = self.cal.drive_power_short
            m = self.cal.drive_m_short
            b = self.cal.drive_b_short
        else:
            power = self.cal.drive_power_long
            m = self.cal.drive_m_long
            b = self.cal.drive_b_long

        duration = m * abs_d + b

        left = direction * power
        right = direction * power

        print(
            f"[TIMED] DRIVE d={d:.1f}mm "
            f"p={power:.2f} t={duration:.3f}s"
        )

        self._run(left, right, duration)

    def rotate(self, angle_deg: float):
        if abs(angle_deg) < self.cfg.min_rotate_deg:
            return

        a = max(
            -self.cfg.max_rotate_deg,
            min(self.cfg.max_rotate_deg, angle_deg),
        )

        abs_a = abs(a)
        direction = 1.0 if a > 0 else -1.0

        if abs_a < self.cal.rotate_switch_deg:
            power = self.cal.rotate_power_small
            m = self.cal.rotate_m_small
            b = self.cal.rotate_b_small
        else:
            power = self.cal.rotate_power_large
            m = self.cal.rotate_m_large
            b = self.cal.rotate_b_large

        duration = m * abs_a + b

        left = direction * power
        right = -direction * power

        print(
            f"[TIMED] ROTATE a={a:.1f}deg "
            f"p={power:.2f} t={duration:.3f}s"
        )

        self._run(left, right, duration)
