# navigation/velocity_arbiter.py

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class VelocitySource(Enum):
    SERVOING = "servoing"
    PATH_TRACKING = "path_tracking"


@dataclass(frozen=True)
class VelocityCommand:
    """
    Canonical velocity command passed to the velocity backend.

    linear_x_mps:
        Forward/reverse velocity.

    angular_z_rps:
        Rotational velocity.

    lateral_y_mps:
        Lateral velocity for mecanum/strafe drives.
        Zero for differential/tank drive.

    timestamp:
        Timestamp associated with the command source.
    """

    linear_x_mps: float
    angular_z_rps: float
    lateral_y_mps: float
    timestamp: float


class VelocityArbiter:
    """
    Selects which navigation velocity command is passed to the
    velocity backend.

    The arbiter does not calculate motion.

    The active source is selected explicitly by the Navigation Arbiter.
    """

    def select(
        self,
        *,
        source: VelocitySource,
        servoing_command: VelocityCommand | None = None,
        path_tracking_command: VelocityCommand | None = None,
    ) -> VelocityCommand | None:

        if source == VelocitySource.SERVOING:
            return servoing_command

        if source == VelocitySource.PATH_TRACKING:
            return path_tracking_command

        return None