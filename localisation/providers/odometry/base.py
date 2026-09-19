# localisation/providers/odometry/base.py

from __future__ import annotations

from abc import abstractmethod
import math

from localisation.providers.base import PoseProvider, PoseObservation
from navigation.odometry.base import OdometrySource


class OdometryPoseProvider(PoseProvider):
    """
    Generic localisation provider for robot-relative odometry sources.

    Responsibilities:
    - start from an externally accepted global pose
    - read cumulative robot-relative odometry
    - convert each incremental motion into world coordinates
    - publish a propagated PoseObservation

    Concrete subclasses only need to create the appropriate OdometrySource.
    """

    def __init__(
        self,
        name: str,
        *,
        confidence: float,
        base_weight: float,
    ):
        super().__init__(
            name,
            base_weight=base_weight,
        )

        self.confidence = float(confidence)

        self.io = None
        self.odometry: OdometrySource | None = None

        self._x = 0.0
        self._y = 0.0
        self._heading: float | None = None
        self._covariance = None

        self._position_valid = False
        self._heading_valid = False

        self._last_forward_mm = 0.0
        self._last_lateral_mm = 0.0
        self._last_heading_rad = 0.0

        self._reset_required = True

    # --------------------------------------------------
    # Concrete odometry source
    # --------------------------------------------------

    @abstractmethod
    def _create_odometry_source(
        self,
        io,
    ) -> OdometrySource:
        ...

    # --------------------------------------------------
    # IO
    # --------------------------------------------------

    def set_io(self, io) -> None:
        if io is self.io and self.odometry is not None:
            return

        self.io = io

        if io is None:
            self.odometry = None
            self._reset_required = True
            return

        self.odometry = self._create_odometry_source(io)
        self._reset_required = True

    # --------------------------------------------------
    # Localisation lifecycle
    # --------------------------------------------------

    def reseed(self, pose) -> None:
        if pose.position_valid:
            self._x = float(pose.x)
            self._y = float(pose.y)
            self._position_valid = True
        else:
            self._position_valid = False

        if pose.heading_valid and pose.heading is not None:
            self._heading = float(pose.heading)
            self._heading_valid = True
        else:
            self._heading = None
            self._heading_valid = False

        self._covariance = pose.covariance

        self._last_forward_mm = 0.0
        self._last_lateral_mm = 0.0
        self._last_heading_rad = 0.0

        self._reset_required = True

    def invalidate(self) -> None:
        self._position_valid = False
        self._heading_valid = False
        self._heading = None
        self._covariance = None
        self._reset_required = True

    # --------------------------------------------------
    # Odometry
    # --------------------------------------------------

    def _reset_odometry(self) -> bool:
        if self.odometry is None:
            return False

        try:
            self.odometry.reset()
        except Exception as exc:
            print(
                f"[{self.name.upper()}][RESET] "
                f"failed: {exc}"
            )
            return False

        self._last_forward_mm = 0.0
        self._last_lateral_mm = 0.0
        self._last_heading_rad = 0.0

        self._reset_required = False
        return True

    def _advance(self) -> bool:
        if self.odometry is None:
            return False

        if not self._position_valid:
            return False

        if not self._heading_valid or self._heading is None:
            return False

        if self._reset_required:
            if not self._reset_odometry():
                return False

            # First read after reset establishes the baseline only.
            return True

        try:
            motion = self.odometry.read()
        except Exception as exc:
            print(
                f"[{self.name.upper()}][READ] "
                f"failed: {exc}"
            )
            return False

        if motion.heading_rad is None:
            print(
                f"[{self.name.upper()}][READ] "
                "odometry did not provide heading"
            )
            return False

        forward_mm = float(motion.forward_mm)
        lateral_mm = float(motion.lateral_mm)
        heading_rad = float(motion.heading_rad)

        delta_forward_mm = (
            forward_mm - self._last_forward_mm
        )

        delta_lateral_mm = (
            lateral_mm - self._last_lateral_mm
        )

        delta_heading_rad = (
            heading_rad - self._last_heading_rad
        )

        self._last_forward_mm = forward_mm
        self._last_lateral_mm = lateral_mm
        self._last_heading_rad = heading_rad

        # Proper covariance propagation is not implemented yet.
        if (
            delta_forward_mm != 0.0
            or delta_lateral_mm != 0.0
            or delta_heading_rad != 0.0
        ):
            self._covariance = None

        mid_heading = (
            self._heading
            + 0.5 * delta_heading_rad
        )

        dx_world = (
            delta_forward_mm * math.cos(mid_heading)
            - delta_lateral_mm * math.sin(mid_heading)
        )

        dy_world = (
            delta_forward_mm * math.sin(mid_heading)
            + delta_lateral_mm * math.cos(mid_heading)
        )

        self._x += dx_world
        self._y += dy_world

        self._heading = self._wrap(
            self._heading + delta_heading_rad
        )

        return True

    # --------------------------------------------------
    # Provider output
    # --------------------------------------------------

    def get_observation(
        self,
        now_s: float,
    ) -> PoseObservation | None:

        if not self._advance():
            return None

        if not self._position_valid:
            return None

        if not self._heading_valid or self._heading is None:
            return None

        return PoseObservation(
            x=self._x,
            y=self._y,
            heading=self._heading,
            position_valid=True,
            heading_valid=True,
            confidence=self.confidence,
            source=self.name,
            timestamp=float(now_s),
            is_absolute=False,
            covariance=self._covariance,
            diagnostics={
                "forward_mm": self._last_forward_mm,
                "lateral_mm": self._last_lateral_mm,
                "heading_delta_rad": self._last_heading_rad,
            },
        )

    # --------------------------------------------------
    # Helpers
    # --------------------------------------------------

    @staticmethod
    def _wrap(angle_rad: float) -> float:
        return (
            angle_rad + math.pi
        ) % (2.0 * math.pi) - math.pi