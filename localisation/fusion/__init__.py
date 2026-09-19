# localisation/fusion/__init__.py

from __future__ import annotations

from typing import Iterable

from localisation.arbitration import Arbitrator
from localisation.fusion.base import PoseEstimator
from localisation.providers.base import PoseProvider


def create_default_estimator(
        providers: Iterable[PoseProvider],
) -> PoseEstimator:
    """
    Create the default final localisation estimator.

    Today:
        simple arbitration between family-level pose sources.

    Future:
        sensor fusion / EKF can replace this implementation without
        changing Localisation.
    """
    return Arbitrator(providers)


__all__ = [
    "PoseEstimator",
    "create_default_estimator",
]