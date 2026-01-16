from config import CONFIG
from motion_backends.timed import TimedMotionBackend


def create_motion_backend(name, lvl2):
    if name == "timed":
        return TimedMotionBackend(lvl2, CONFIG)

    raise ValueError(f"Unknown motion backend: {name}")
