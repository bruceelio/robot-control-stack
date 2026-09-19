# navigation/odometry/resolve.py

from __future__ import annotations

from navigation.odometry.base import OdometrySource
from navigation.odometry.drive_encoders import DriveEncoderOdometry



def resolve_odometry(
    *,
    io,
    config,
) -> OdometrySource:
    """
    Build the robot-relative odometry system available
    for the current robot configuration.

    Current implementation:
        drive encoders

    Future implementation may combine:
        drive encoders
        odometry
        IMU
        fused odometry
    """

    required_drive_encoders = (
        "drive_front_left",
        "drive_front_right",
    )

    if all(
        name in config.encoders
        for name in required_drive_encoders
    ):
        return DriveEncoderOdometry(
            io=io,
            config=config,
        )

    raise RuntimeError(
        "No usable odometry configuration is available"
    )