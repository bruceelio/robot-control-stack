# localisation/providers/odometry/drive_encoders.py

from __future__ import annotations

from config import CONFIG
from localisation.providers.odometry.base import OdometryPoseProvider
from navigation.odometry.drive_encoders import DriveEncoderOdometry
from navigation.odometry.base import OdometrySource


class DriveEncoderProvider(OdometryPoseProvider):
    """
    Propagated localisation from drivetrain encoders.

    The generic pose integration, reseeding and covariance lifecycle
    are handled by OdometryPoseProvider.

    This class only selects and configures the drive-encoder
    odometry source.
    """

    def __init__(
        self,
        *,
        config=CONFIG,
    ):
        self.config = config

        super().__init__(
            "drive_encoders",
            confidence=0.75,
            base_weight=0.75,
        )

    def _create_odometry_source(
        self,
        io,
    ) -> OdometrySource:
        return DriveEncoderOdometry(
            io=io,
            config=self.config,
        )