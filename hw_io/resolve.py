# checkout/resolve.py

from typing import Any

from hw_io.base import IOMap


def resolve_io(*, robot: Any, hardware_profile: str, camera_manager=None) -> IOMap:
    if hardware_profile == "sr1":
        from hw_io.sr1 import SR1IO
        return SR1IO(robot)

    if hardware_profile == "mega2560":
        from hw_io.hw_mega2560 import Mega2560IO
        return Mega2560IO(robot, camera_manager=camera_manager)

    raise RuntimeError(
        f"No IOMap implementation for hardware_profile={hardware_profile!r}"
    )