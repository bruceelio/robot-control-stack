# navigation/odometry/base.py

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


OdometryCovariance = tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]

@dataclass(frozen=True)
class OdometryDelta:
    forward_mm: float
    lateral_mm: float
    heading_rad: float | None

    # Covariance for:
    # [forward_mm, lateral_mm, heading_rad]
    #
    # Units:
    #   forward/lateral variance -> mm^2
    #   heading variance         -> rad^2
    #   cross terms              -> corresponding mixed units
    #
    # None means this odometry source does not yet
    # provide an uncertainty estimate.
    covariance: OdometryCovariance | None = None



class OdometryConsistencyError(RuntimeError):
    pass


class OdometrySource(Protocol):
    """
    Interface required by consumers of robot-relative odometry.
    """

    def reset(self) -> None:
        ...

    def read(self) -> OdometryDelta:
        ...

