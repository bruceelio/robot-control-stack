# hw_io/resolve.py

from typing import Any

from hw_io.base import IOMap


def resolve_io(*, robot: Any, hardware_profile: str, camera_manager=None) -> IOMap:
    if hardware_profile == "sr1":
        from hw_io.sr1 import SR1IO
        return SR1IO(robot)

    if hardware_profile == "bob_bot":
        from hw_io.bob_bot import BobBotIO
        return BobBotIO(robot, camera_manager=camera_manager)

    raise RuntimeError(
        f"No IOMap implementation for hardware_profile={hardware_profile!r}"
    )