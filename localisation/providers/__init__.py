# localisation/providers/__init__.py

"""
Localisation providers subpackage.

Exports:
- PoseObservation value type
- PoseProvider base interface
- Concrete provider implementations
- default_providers(): sensible default provider ordering
"""

from .base import PoseObservation, PoseProvider
from .vision.pose_cam1_markers2 import Cam1Markers2Provider


def default_providers():
    """
    Return providers in priority order (best-first).
    """
    return [
        Cam1Markers2Provider(),
    ]


__all__ = [
    "PoseObservation",
    "PoseProvider",
    "Cam1Markers2Provider",
    "default_providers",
]