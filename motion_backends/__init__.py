# motion_backends/__init__.py

from motion_backends.timed import TimedMotionBackend
from motion_backends.odometry import OdometryMotionBackend
from navigation.odometry import resolve_odometry


def create_motion_backend(name, lvl2, config, calibration):
    if name == "timed":
        return TimedMotionBackend(
            lvl2,
            config,
            calibration,
        )

    if name == "odometry":
        odometry = resolve_odometry(
            io=lvl2.io,
            config=config,
        )

        return OdometryMotionBackend(
            lvl2,
            config,
            calibration,
            odometry=odometry,
        )

    raise ValueError(f"Unknown motion backend: {name}")