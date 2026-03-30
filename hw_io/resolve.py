# hw_io/resolve.py

from typing import Any

from hw_io.base import IOMap
from hw_io.bob_bot import BobBotIO


def resolve_io(*, robot: Any, hardware_profile: str) -> IOMap:

    if hardware_profile == "sr1":
        from hw_io.sr1 import SR1IO   # ← lazy import
        return SR1IO(robot)

    if hardware_profile == "bob_bot":
        return BobBotIO(robot)

    raise RuntimeError(
        f"No IOMap implementation for hardware_profile={hardware_profile!r}"
    )