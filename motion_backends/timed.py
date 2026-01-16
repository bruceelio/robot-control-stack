# motion_backends/timed.py

import time
from calibration.base import drive_duration, rotate_duration


class TimedMotionBackend:
    """
    Motion backend using time-based movement (no encoders).
    Compatible with Primitive-based architecture.
    """

    def __init__(
        self,
        lvl2,
        *,
        min_drive_mm: float,
        min_rotate_deg: float,
        drive_factor: float,
        rotate_factor: float,
    ):
        self.lvl2 = lvl2

        # Injected tuning parameters
        self.min_drive_mm = min_drive_mm
        self.min_rotate_deg = min_rotate_deg
        self.drive_factor = drive_factor
        self.rotate_factor = rotate_factor

        self.end_time = None
        self.mode = None


    # ---------------------
    # Public API
    # ---------------------

    def drive(self, *, distance_mm: float):
        if abs(distance_mm) < self.min_drive_mm:
            self.stop()
            return

        duration, power = drive_duration(
            abs(distance_mm),
            drive_factor=self.drive_factor,
        )

        direction = 1 if distance_mm >= 0 else -1

        self.end_time = time.time() + duration
        self.mode = "drive"

        self.lvl2.DRIVE(
            direction * power,
            direction * power,
            duration
        )

    def rotate(self, angle_deg: float):
        if abs(angle_deg) < self.min_rotate_deg:
            self.stop()
            return

        duration, power = rotate_duration(
            abs(angle_deg),
            rotate_factor=self.rotate_factor,
        )

        self.end_time = time.time() + duration
        self.mode = "rotate"

        if angle_deg > 0:
            self.lvl2.DRIVE(power, -power, duration)
        else:
            self.lvl2.DRIVE(-power, power, duration)

    # ---------------------
    # State helpers
    # ---------------------

    def is_busy(self) -> bool:
        if self.mode is None:
            return False

        if time.time() >= self.end_time:
            self.stop()
            return False

        return True

    def is_motion_complete(self) -> bool:
        return not self.is_busy()

    def stop(self):
        self.lvl2.DRIVE(0, 0, 0)
        self.mode = None
        self.end_time = None
