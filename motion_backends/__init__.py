from config import CONFIG
from motion_backends.timed import TimedMotionBackend


def create_motion_backend(name, lvl2):
    if name == "timed":
        return TimedMotionBackend(
            lvl2,
            min_drive_mm=CONFIG.min_drive_mm,
            min_rotate_deg=CONFIG.min_rotate_deg,
            drive_factor=CONFIG.drive_factor,
            rotate_factor=CONFIG.rotate_factor,
        )

    raise ValueError(f"Unknown motion backend: {name}")
