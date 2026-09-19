# localisation/fusion/base.py

from __future__ import annotations

from typing import Protocol

from localisation.providers.base import PoseObservation


class PoseEstimator(Protocol):
    """
    Interface for the final localisation estimation stage.

    Current implementation:
        Arbitrator

    Future implementations may include:
        - EKF
        - other multi-sensor fusion methods
    """

    def estimate(
        self,
        *,
        now_s: float,
    ) -> PoseObservation | None:
        ...