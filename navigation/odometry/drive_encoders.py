# navigation/odometry/drive_encoders.py

from __future__ import annotations

import math

from hw_io.encoder_manager import EncoderManager, make_signals
from navigation.odometry.base import (
    OdometryDelta,
    OdometryConsistencyError,
)


class DriveEncoderOdometry:
    """
    Robot-relative odometry derived from the left and right
    drive-wheel encoders.

    This class measures robot movement only.
    It does not command the drivetrain.
    """

    def __init__(
        self,
        *,
        io,
        config,
    ):
        self.io = io
        self.cfg = config

        drive_encoders = {
            "drive_front_left":
                config.encoders["drive_front_left"],

            "drive_front_right":
                config.encoders["drive_front_right"],
        }

        drive_signs = {
            "drive_front_left":
                config.encoder_sign["drive_front_left"],

            "drive_front_right":
                config.encoder_sign["drive_front_right"],
        }

        self.encoder_manager = EncoderManager(
            drive_encoders,
            drive_signs,
        )

        self.signals = make_signals()

        self._left_mm = 0.0
        self._right_mm = 0.0

    # --------------------------------------------------
    # Diagnostics
    # --------------------------------------------------

    @property
    def left_mm(self) -> float:
        return self._left_mm

    @property
    def right_mm(self) -> float:
        return self._right_mm

    @property
    def wheel_disagreement_mm(self) -> float:
        return abs(
            self._left_mm - self._right_mm
        )

    def check_consistency(
            self,
            *,
            limit_mm: float,
    ) -> None:
        disagreement_mm = abs(
            self._left_mm - self._right_mm
        )

        if disagreement_mm > limit_mm:
            raise OdometryConsistencyError(
                "Drive encoder disagreement: "
                f"left={self._left_mm:.1f}mm "
                f"right={self._right_mm:.1f}mm"
            )

    # --------------------------------------------------
    # Reference
    # --------------------------------------------------

    def reset(self) -> None:
        """
        Establish the current robot position as the
        zero reference for subsequent odometry reads.
        """

        self.encoder_manager.reset()

        self.encoder_manager.update(
            io=self.io,
            signals=self.signals,
        )

        self._left_mm = 0.0
        self._right_mm = 0.0

    # --------------------------------------------------
    # Measurement
    # --------------------------------------------------

    def read(self) -> OdometryDelta:
        """
        Return robot movement since the last reset().
        """

        self.encoder_manager.update(
            io=self.io,
            signals=self.signals,
        )

        left = self.signals.encoder["drive_front_left"]
        right = self.signals.encoder["drive_front_right"]

        if not left.valid:
            raise RuntimeError(
                "Left drive encoder is not valid"
            )

        if not right.valid:
            raise RuntimeError(
                "Right drive encoder is not valid"
            )

        if left.units != "rev":
            raise RuntimeError(
                f"Left drive encoder units must be 'rev', "
                f"got {left.units!r}"
            )

        if right.units != "rev":
            raise RuntimeError(
                f"Right drive encoder units must be 'rev', "
                f"got {right.units!r}"
            )

        left_diameter_mm = float(
            self.cfg.encoder_wheel_diameter_mm[
                "drive_front_left"
            ]
        )

        right_diameter_mm = float(
            self.cfg.encoder_wheel_diameter_mm[
                "drive_front_right"
            ]
        )

        self._left_mm = (
            left.position
            * math.pi
            * left_diameter_mm
        )

        self._right_mm = (
            right.position
            * math.pi
            * right_diameter_mm
        )

        forward_mm = (
            self._left_mm + self._right_mm
        ) / 2.0

        track_width_mm = float(
            self.cfg.drive_track_width_mm
        )

        if track_width_mm <= 0:
            raise RuntimeError(
                f"Invalid drive track width: {track_width_mm}"
            )

        heading_rad = (
                              self._right_mm - self._left_mm
                      ) / track_width_mm

        return OdometryDelta(
            forward_mm=forward_mm,
            lateral_mm=0.0,
            heading_rad=heading_rad,
        )