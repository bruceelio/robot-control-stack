# navigation/odometry/three_deadwheel.py

from __future__ import annotations

from navigation.odometry.base import OdometryDelta
from hw_io.encoder_manager import EncoderManager, make_signals


class ThreeDeadwheelKinematics:
    """
    Robot-relative odometry from three tracking wheels.

    Layout:
        - left parallel wheel
        - right parallel wheel
        - perpendicular/lateral wheel

    Coordinate convention:
        +forward  = robot forward
        +lateral  = robot left
        +heading  = counter-clockwise / left turn

    perpendicular_forward_offset_mm:
        Position of the perpendicular wheel relative to the
        robot rotation centre.

        positive = forward of centre
        negative = behind centre
    """

    def __init__(
        self,
        *,
        parallel_track_width_mm: float,
        perpendicular_forward_offset_mm: float,
    ):
        self.parallel_track_width_mm = float(
            parallel_track_width_mm
        )

        self.perpendicular_forward_offset_mm = float(
            perpendicular_forward_offset_mm
        )

        if self.parallel_track_width_mm <= 0.0:
            raise ValueError(
                "parallel_track_width_mm must be > 0"
            )

    def calculate(
        self,
        *,
        left_mm: float,
        right_mm: float,
        perpendicular_mm: float,
    ) -> OdometryDelta:
        """
        Convert cumulative deadwheel travel since reset into
        cumulative robot-relative odometry.
        """

        left_mm = float(left_mm)
        right_mm = float(right_mm)
        perpendicular_mm = float(perpendicular_mm)

        heading_rad = (
            right_mm - left_mm
        ) / self.parallel_track_width_mm

        forward_mm = (
            left_mm + right_mm
        ) / 2.0

        # A perpendicular wheel displaced forward from the
        # rotation centre also rolls during rotation.
        #
        # measured_perpendicular =
        #     true_lateral + offset * heading
        #
        # therefore:
        lateral_mm = (
            perpendicular_mm
            - self.perpendicular_forward_offset_mm
            * heading_rad
        )

        return OdometryDelta(
            forward_mm=forward_mm,
            lateral_mm=lateral_mm,
            heading_rad=heading_rad,
        )

class ThreeDeadwheelOdometry:
    """
    Robot-relative odometry from three tracking-wheel encoders.

    This class owns encoder processing and returns cumulative
    OdometryDelta values since reset().

    Encoder names and geometry are supplied by configuration;
    nothing robot-specific is hard-coded here.

    Deadwheel encoder profiles are expected to produce position
    values in millimetres.
    """

    def __init__(
        self,
        *,
        io,
        encoder_assignments: dict[str, str],
        encoder_sign: dict[str, int],
        left_name: str,
        right_name: str,
        perpendicular_name: str,
        parallel_track_width_mm: float,
        perpendicular_forward_offset_mm: float,
    ):
        self.io = io

        self.left_name = left_name
        self.right_name = right_name
        self.perpendicular_name = perpendicular_name

        self.encoder_manager = EncoderManager(
            encoder_assignments,
            encoder_sign,
        )

        self.signals = make_signals()

        self.kinematics = ThreeDeadwheelKinematics(
            parallel_track_width_mm=parallel_track_width_mm,
            perpendicular_forward_offset_mm=(
                perpendicular_forward_offset_mm
            ),
        )

        self._left_zero_mm = 0.0
        self._right_zero_mm = 0.0
        self._perpendicular_zero_mm = 0.0

    def reset(self) -> None:
        """
        Establish the current physical position as the
        zero reference for subsequent odometry reads.
        """

        self.encoder_manager.reset()

        self.encoder_manager.update(
            io=self.io,
            signals=self.signals,
        )

        left = self._signal(self.left_name)
        right = self._signal(self.right_name)
        perpendicular = self._signal(
            self.perpendicular_name
        )

        self._left_zero_mm = float(left.position)
        self._right_zero_mm = float(right.position)
        self._perpendicular_zero_mm = float(
            perpendicular.position
        )

    def read(self) -> OdometryDelta:
        """
        Return cumulative robot-relative movement since reset().
        """

        self.encoder_manager.update(
            io=self.io,
            signals=self.signals,
        )

        left = self._signal(self.left_name)
        right = self._signal(self.right_name)
        perpendicular = self._signal(
            self.perpendicular_name
        )

        left_mm = (
            float(left.position)
            - self._left_zero_mm
        )

        right_mm = (
            float(right.position)
            - self._right_zero_mm
        )

        perpendicular_mm = (
            float(perpendicular.position)
            - self._perpendicular_zero_mm
        )

        return self.kinematics.calculate(
            left_mm=left_mm,
            right_mm=right_mm,
            perpendicular_mm=perpendicular_mm,
        )

    def _signal(self, name: str):
        try:
            signal = self.signals.encoder[name]
        except KeyError as exc:
            raise RuntimeError(
                f"Deadwheel encoder {name!r} "
                "did not produce a signal"
            ) from exc

        if not signal.valid:
            raise RuntimeError(
                f"Deadwheel encoder {name!r} is not valid"
            )

        if signal.units != "mm":
            raise RuntimeError(
                f"Deadwheel encoder {name!r} units must be "
                f"'mm', got {signal.units!r}"
            )

        return signal