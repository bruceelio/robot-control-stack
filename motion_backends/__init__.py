from .timed import TimedMotionBackend

def create_motion_backend(name, lvl2):
    if name == "timed":
        return TimedMotionBackend(lvl2)
    raise ValueError(f"Unknown motion backend '{name}'")
