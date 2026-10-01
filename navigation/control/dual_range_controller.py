# navigation/control/dual_range_controller.py

from __future__ import annotations

from dataclasses import dataclass
import math

from navigation.control.range_only_controller import (
    RangeOnlyController,
    RangeOnlyParams,
    RangeOnlyResult,
)


@dataclass(frozen=True)
class RangeSensorPose2D:
    """
    Pose of one range-sensor origin in the controller geometry frame.

    Coordinate convention:
        +x forward
        +y left
        yaw_rad > 0 counter-clockwise

    The geometry-frame origin is chosen by the caller. It may be base_link,
    a gripper frame, or another control reference.
    """

    x_mm: float
    y_mm: float
    yaw_rad: float = 0.0

    def __post_init__(self) -> None:
        for name, value in (
            ("x_mm", self.x_mm),
            ("y_mm", self.y_mm),
            ("yaw_rad", self.yaw_rad),
        ):
            if not math.isfinite(float(value)):
                raise ValueError(f"{name} must be finite")


@dataclass(frozen=True)
class DualRangeGeometry:
    """Geometry of the two range rays."""

    left: RangeSensorPose2D
    right: RangeSensorPose2D

    @property
    def sensor_baseline_mm(self) -> float:
        return math.hypot(
            self.left.x_mm - self.right.x_mm,
            self.left.y_mm - self.right.y_mm,
        )


@dataclass(frozen=True)
class DualRangeParams:
    """
    Tuning for dual-range regulation against one locally planar surface.

    range:
        RangeOnlyParams for perpendicular stand-off regulation.

    angular_kp_s_inv:
        Proportional gain from surface-normal angular error [rad]
        to angular velocity [rad/s].
    """

    range: RangeOnlyParams = RangeOnlyParams()
    angular_kp_s_inv: float = 2.0
    max_angular_rps: float = 0.35
    angular_tolerance_rad: float = math.radians(2.0)

    def __post_init__(self) -> None:
        if not math.isfinite(self.angular_kp_s_inv) or self.angular_kp_s_inv <= 0.0:
            raise ValueError("angular_kp_s_inv must be finite and > 0")
        if not math.isfinite(self.max_angular_rps) or self.max_angular_rps <= 0.0:
            raise ValueError("max_angular_rps must be finite and > 0")
        if (
            not math.isfinite(self.angular_tolerance_rad)
            or self.angular_tolerance_rad < 0.0
        ):
            raise ValueError("angular_tolerance_rad must be finite and >= 0")


@dataclass(frozen=True)
class DualRangeResult:
    """
    Result of one dual-range control evaluation.

    The controller infers the local surface line from the two beam-hit
    points, then derives perpendicular distance and surface-normal angle.
    """

    linear_mps: float
    angular_rps: float

    left_range_mm: float
    right_range_mm: float
    range_delta_mm: float

    surface_distance_mm: float
    target_range_mm: float
    range_error_mm: float

    surface_normal_angle_rad: float
    target_normal_angle_rad: float
    normal_error_rad: float

    left_hit_x_mm: float
    left_hit_y_mm: float
    right_hit_x_mm: float
    right_hit_y_mm: float

    sensor_baseline_mm: float

    range_result: RangeOnlyResult
    reached: bool


class DualRangeController:
    """
    Source-agnostic dual-range controller for one locally planar surface.

    Inputs:
        r_left, r_right
        known sensor-ray geometry
        desired perpendicular stand-off
        desired surface-normal angle

    Outputs:
        v, omega

    IMPORTANT:
        Both returns must correspond to the same locally planar physical
        surface. If one sensor sees a compact object while the other sees a
        background wall, a higher layer must interpret that pair instead.

    The controller deliberately knows nothing about ToF/ultrasonic/lidar,
    cubes/walls/grippers, observation freshness, or drivetrain output.
    """

    _GEOMETRY_EPS_MM = 1e-9

    def __init__(
        self,
        *,
        geometry: DualRangeGeometry,
        params: DualRangeParams = DualRangeParams(),
    ) -> None:
        if geometry.sensor_baseline_mm <= self._GEOMETRY_EPS_MM:
            raise ValueError("dual-range sensors must have distinct origins")

        self.geometry = geometry
        self.params = params
        self.range_controller = RangeOnlyController(params.range)

    def reset(self) -> None:
        self.range_controller.reset()

    def update(
        self,
        *,
        left_range_mm: float,
        right_range_mm: float,
        target_range_mm: float,
        target_normal_angle_rad: float = 0.0,
    ) -> DualRangeResult:
        left_range_mm = self._positive_range(
            left_range_mm,
            name="left_range_mm",
        )
        right_range_mm = self._positive_range(
            right_range_mm,
            name="right_range_mm",
        )
        target_range_mm = self._finite_float(
            target_range_mm,
            name="target_range_mm",
        )
        target_normal_angle_rad = self._finite_float(
            target_normal_angle_rad,
            name="target_normal_angle_rad",
        )

        if target_range_mm < 0.0:
            raise ValueError("target_range_mm must be >= 0")

        left_hit = self._beam_hit(
            self.geometry.left,
            left_range_mm,
        )
        right_hit = self._beam_hit(
            self.geometry.right,
            right_range_mm,
        )

        # Tangent to the inferred local surface, from right hit to left hit.
        tangent_x = left_hit[0] - right_hit[0]
        tangent_y = left_hit[1] - right_hit[1]
        tangent_norm = math.hypot(tangent_x, tangent_y)

        if tangent_norm <= self._GEOMETRY_EPS_MM:
            raise ValueError(
                "dual-range hit points are coincident; "
                "surface orientation is undefined"
            )

        # A normal to tangent (tx, ty) is (ty, -tx).
        normal_x = tangent_y / tangent_norm
        normal_y = -tangent_x / tangent_norm

        # Select the equivalent normal which points generally forward.
        if normal_x < 0.0:
            normal_x = -normal_x
            normal_y = -normal_y

        surface_normal_angle_rad = math.atan2(normal_y, normal_x)

        # Both hit points lie on the inferred line. Average their normal
        # projections to avoid tiny floating-point asymmetry.
        left_surface_distance_mm = (
            normal_x * left_hit[0]
            + normal_y * left_hit[1]
        )
        right_surface_distance_mm = (
            normal_x * right_hit[0]
            + normal_y * right_hit[1]
        )
        surface_distance_mm = 0.5 * (
            left_surface_distance_mm
            + right_surface_distance_mm
        )

        range_result = self.range_controller.update(
            range_mm=surface_distance_mm,
            target_range_mm=target_range_mm,
        )

        normal_error_rad = self._wrap_pi(
            surface_normal_angle_rad
            - target_normal_angle_rad
        )

        angular_aligned = (
            abs(normal_error_rad)
            <= self.params.angular_tolerance_rad
        )

        if angular_aligned:
            angular_rps = 0.0
        else:
            angular_rps = (
                self.params.angular_kp_s_inv
                * normal_error_rad
            )
            angular_rps = self._clamp(
                angular_rps,
                -self.params.max_angular_rps,
                self.params.max_angular_rps,
            )

        reached = range_result.reached and angular_aligned

        return DualRangeResult(
            linear_mps=(0.0 if reached else range_result.linear_mps),
            angular_rps=(0.0 if reached else angular_rps),
            left_range_mm=left_range_mm,
            right_range_mm=right_range_mm,
            range_delta_mm=left_range_mm - right_range_mm,
            surface_distance_mm=surface_distance_mm,
            target_range_mm=target_range_mm,
            range_error_mm=surface_distance_mm - target_range_mm,
            surface_normal_angle_rad=surface_normal_angle_rad,
            target_normal_angle_rad=target_normal_angle_rad,
            normal_error_rad=normal_error_rad,
            left_hit_x_mm=left_hit[0],
            left_hit_y_mm=left_hit[1],
            right_hit_x_mm=right_hit[0],
            right_hit_y_mm=right_hit[1],
            sensor_baseline_mm=self.geometry.sensor_baseline_mm,
            range_result=range_result,
            reached=reached,
        )

    @staticmethod
    def _beam_hit(
        sensor: RangeSensorPose2D,
        range_mm: float,
    ) -> tuple[float, float]:
        return (
            sensor.x_mm + range_mm * math.cos(sensor.yaw_rad),
            sensor.y_mm + range_mm * math.sin(sensor.yaw_rad),
        )

    @classmethod
    def _positive_range(cls, value: float, *, name: str) -> float:
        value = cls._finite_float(value, name=name)
        if value <= 0.0:
            raise ValueError(f"{name} must be > 0")
        return value

    @staticmethod
    def _finite_float(value: float, *, name: str) -> float:
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _wrap_pi(angle_rad: float) -> float:
        return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


__all__ = [
    "DualRangeController",
    "DualRangeGeometry",
    "DualRangeParams",
    "DualRangeResult",
    "RangeSensorPose2D",
]
