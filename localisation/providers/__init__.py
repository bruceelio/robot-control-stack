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

# Vision providers
from .vision.pose_cam1_markers2 import Cam1Markers2Provider
# from .vision.pose_apriltag_pnp import AprilTagPnPPoseProvider
from .vision.vision_arbiter import VisionArbiter

# Motion / fallback providers
from .motion.commanded_motion import CommandedMotionProvider


def default_providers():
    """
    Return providers in priority order (best-first).
    """

    vision_provider = VisionArbiter(
        providers=[
            Cam1Markers2Provider(),
        ]
    )

    return [
        vision_provider,            # Aggregated vision localisation
        CommandedMotionProvider(),  # Dead-reckoning fallback
    ]


__all__ = [
    "PoseObservation",
    "PoseProvider",

    # Vision
    "Cam1Markers2Provider",
    "AprilTagPnPPoseProvider",
    "VisionArbiter",

    # Motion
    "CommandedMotionProvider",

    # Factory
    "default_providers",
]