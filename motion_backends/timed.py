import time
from calibration import drive_duration, rotate_duration
from config import MIN_ROTATE_DEG, MIN_DRIVE_MM

class TimedMotionBackend:
    """
    Motion backend using time-based movement (no encoders).
    Compatible with Primitive-based architecture.
    """

    def __init__(self, lvl2):
        self.lvl2 = lvl2
        self.end_time = None
        self.mode = None

    # ---------------------
    # Public API (NEW)
    # ---------------------

    def drive(self, *, distance_mm: float):
        # Ignore tiny moves
        if abs(distance_mm) < MIN_DRIVE_MM:
            self.mode = None
            self.end_time = None
            return

        duration, power = drive_duration(abs(distance_mm))

        self.end_time = time.time() + duration
        self.mode = "drive"

        # Forward or backward
        direction = 1 if distance_mm >= 0 else -1
        self.lvl2.DRIVE(
            direction * power,
            direction * power,
            duration
        )

    def rotate(self, angle_deg: float):
        if abs(angle_deg) < MIN_ROTATE_DEG:
            self.mode = None
            self.end_time = None
            return

        duration, power = rotate_duration(abs(angle_deg))
        self.end_time = time.time() + duration
        self.mode = "rotate"

        if angle_deg > 0:
            self.lvl2.DRIVE(power, -power, duration)
        else:
            self.lvl2.DRIVE(-power, power, duration)

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
