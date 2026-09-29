# navigation/wall_following/models.py

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class WallSide(Enum):
    LEFT = "left"
    RIGHT = "right"


@dataclass(frozen=True)
class WallFollowCommand:
    """
    Canonical differential-drive / unicycle wall-following command.
    """

    linear_x_mps: float
    angular_z_rps: float


@dataclass(frozen=True)
class WallFollowResult:
    """
    Wall-following command plus useful diagnostic state.
    """

    command: WallFollowCommand

    distance_error_mm: float | None = None
    distance_rate_mm_s: float | None = None
    distance_rate_filtered_mm_s: float | None = None
    inferred_heading_error_rad: float | None = None

    distance_proportional_angular_z_rps: float | None = None
    heading_angular_z_rps: float | None = None
    unclamped_angular_z_rps: float | None = None

    heading_error_rad: float | None = None