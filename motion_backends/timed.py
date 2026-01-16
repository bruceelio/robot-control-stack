# motion_backends/timed.py

"""
Timed motion backend.

Open-loop, time-based motion execution using calibration profiles.

Responsibilities:
- Track motion state (busy / idle)
- Convert semantic commands (drive, rotate) into timed motor actions
- Delegate hardware execution to Level2
- Use calibration constants (no fitting, no data collection)

This backend is intentionally simple and deterministic.
"""

import time
from enum import Enum, auto

from calibration import CALIBRATION


# --------------------------------------------------
# Motion state helper
# --------------------------------------------------

class MotionState(Enum):
    IDLE = auto()
    DRIVING = auto()
    ROTATING = auto()


# --------------------------------------------------
# Timed motion backend
# --------------------------------------------------

class TimedMotionBackend:
    def __init__(self, lvl2, config):
        self.lvl2 = lvl2
        self.config = config

        self._state = MotionState.IDLE
        self._busy_until = 0.0

    # --------------------------------------------------
    # State helpers
    # --------------------------------------------------

    def is_busy(self) -> bool:
        """
        Return True while a motion is still executing.
        """
        if self._state == MotionState.IDLE:
            return False

        if time.monotonic() >= self._busy_until:
            self._state = MotionState.IDLE
            return False

        return True

    def stop(self):
        """
        Immediately stop all motion.
        """
        self.lvl2.DRIVE(0.0, 0.0, duration=0.0)
        self._state = MotionState.IDLE
        self._busy_until = 0.0

    def _start_motion(self, *, duration: float, state: MotionState):
        self._state = state
        self._busy_until = time.monotonic() + duration

    # --------------------------------------------------
    # Public motion API (used by primitives)
    # --------------------------------------------------

    def drive(self, *, distance_mm: float):
        """
        Drive forward/backward by distance_mm.
        Positive = forward, negative = backward.
        """

        # Apply surface / policy scaling ONCE
        distance_mm = distance_mm * self.config.drive_factor

        duration, power = self._compute_drive(distance_mm)

        if duration <= 0.0 or power <= 0.0:
            self._state = MotionState.IDLE
            return

        direction = 1.0 if distance_mm >= 0 else -1.0

        self.lvl2.DRIVE(
            left_power=direction * power,
            right_power=direction * power,
            duration=duration,
        )

        self._start_motion(duration=duration, state=MotionState.DRIVING)

    def rotate(self, angle_deg: float):
        """
        Rotate in place by angle_deg.

        SYSTEM CONVENTION:
          +angle = clockwise (right)
          -angle = counter-clockwise (left)
        """

        duration, power = self._compute_rotate(
            angle_deg,
            rotate_factor=self.config.rotate_factor,
        )

        if duration <= 0.0 or power <= 0.0:
            self._state = MotionState.IDLE
            return

        # Clockwise (right): left forward, right backward
        direction = 1.0 if angle_deg >= 0 else -1.0

        left_power = direction * power
        right_power = -direction * power

        print(
            f"[ROTATE][CMD] angle={angle_deg:.2f}° "
            f"left={left_power:.2f} right={right_power:.2f} "
            f"dur={duration:.3f}s"
        )

        self.lvl2.DRIVE(
            left_power=left_power,
            right_power=right_power,
            duration=duration,
        )

        self._start_motion(duration=duration, state=MotionState.ROTATING)

    # --------------------------------------------------
    # Calibration-backed motion math
    # --------------------------------------------------

    def _compute_drive(self, distance_mm: float):
        """
        Convert distance (mm) to (duration_s, power).
        Uses calibrated timing only — no scaling here.
        """
        d = abs(distance_mm)

        if d < CALIBRATION.drive_switch_mm:
            duration = (
                    CALIBRATION.drive_m_short * d
                    + CALIBRATION.drive_b_short
            )
            power = CALIBRATION.drive_power_short
        else:
            duration = (
                    CALIBRATION.drive_m_long * d
                    + CALIBRATION.drive_b_long
            )
            power = CALIBRATION.drive_power_long

        return max(0.0, duration), power

    def _compute_rotate(self, angle_deg: float, *, rotate_factor: float):
        """
        Convert angle (deg) to (duration_s, power).
        """
        a = abs(angle_deg)

        duration = (
            CALIBRATION.rotate_m * a
            + CALIBRATION.rotate_b
        )

        duration = max(0.0, duration * rotate_factor)
        power = CALIBRATION.rotate_power

        return duration, power
