# motion_backends/__init__.py

from motion_backends.timed import TimedMotionBackend
from motion_backends.encoder import EncoderMotionBackend


def create_motion_backend(name, lvl2, config, calibration):
    if name == "timed":
        return TimedMotionBackend(
            lvl2,
            config,
            calibration,
        )

    if name == "encoder":
        return EncoderMotionBackend(
            lvl2=lvl2,
            config=config,
            calibration=calibration,
        )

    raise ValueError(f"Unknown motion backend: {name}")