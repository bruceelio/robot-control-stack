# hw_io/resolve

from typing import Any

from hw_io.base import IOMap
from hw_io.sr1 import SR1IO


def resolve_io(*, robot: Any, hardware_profile: str) -> IOMap:
    """
    Resolve and construct the active IOMap implementation.

    Selection is based ONLY on the hardware profile.
    Environment (real vs simulation) is irrelevant at this layer.
    """

    if hardware_profile == "sr1":
        return SR1IO(robot)

    raise RuntimeError(
        f"No IOMap implementation for hardware_profile={hardware_profile!r}"
    )

