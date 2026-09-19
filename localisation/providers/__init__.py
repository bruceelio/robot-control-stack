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
from .vision.pose_apriltag_pnp import AprilTagPnPPoseProvider
from .vision.vision_arbiter import VisionArbiter

from config import CONFIG

# Motion / fallback providers
from .dead_reckoning.commanded_motion import CommandedMotionProvider
from .odometry.drive_encoders import DriveEncoderProvider
from .odometry.odometry_arbiter import OdometryArbiter


def default_providers():
    """
    Return providers in priority order (best-first).
    """

    vision_provider = VisionArbiter(
        providers=[
            Cam1Markers2Provider(),
            AprilTagPnPPoseProvider(),
        ]
    )

    providers = [
        vision_provider,
    ]

    odometry_sources = []

    encoders = getattr(CONFIG, "encoders", {})

    if (
            "drive_front_left" in encoders
            and "drive_front_right" in encoders
    ):
        odometry_sources.append(
            DriveEncoderProvider()
        )

    if odometry_sources:
        providers.append(
            OdometryArbiter(
                providers=odometry_sources,
            )
        )

    providers.append(
        CommandedMotionProvider()
    )

    return providers


__all__ = [
    "PoseObservation",
    "PoseProvider",

    # Vision
    "Cam1Markers2Provider",
    "AprilTagPnPPoseProvider",
    "VisionArbiter",

    # Dead Reckoning
    "CommandedMotionProvider",

    # Odometry
    "DriveEncoderProvider",
    "OdometryArbiter",

    # Factory
    "default_providers",
]