# navigation/servoing/range_only_controller.py

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RangeOnlyParams:
    """
    Tuning for one-dimensional range regulation.

    kp_s_inv:
        Proportional gain applied to range error expressed in metres.

    max_linear_mps:
        Absolute linear-velocity limit.

    tolerance_mm:
        Absolute range error considered reached.
    """

    kp_s_inv: float = 1.0
    max_linear_mps: float = 0.30
    tolerance_mm: float = 10.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.kp_s_inv) or self.kp_s_inv <= 0.0:
            raise ValueError("kp_s_inv must be finite and > 0")
        if not math.isfinite(self.max_linear_mps) or self.max_linear_mps <= 0.0:
            raise ValueError("max_linear_mps must be finite and > 0")
        if not math.isfinite(self.tolerance_mm) or self.tolerance_mm < 0.0:
            raise ValueError("tolerance_mm must be finite and >= 0")


@dataclass(frozen=True)
class RangeOnlyResult:
    """
    Result of one range-only control evaluation.

    range_error_mm = range_mm - target_range_mm

    Positive error means the measured range is too large and therefore
    requests positive/forward velocity. Negative error requests reverse.
    """

    linear_mps: float
    range_mm: float
    target_range_mm: float
    range_error_mm: float
    reached: bool


class RangeOnlyController:
    """
    Source-agnostic one-dimensional range controller.

    Inputs:
        r  = current scalar range
        r* = desired scalar range

    Output:
        v = signed linear velocity

    The controller deliberately knows nothing about the source of the range,
    target identity, localisation, observation freshness, drivetrain, or
    behaviour/state-machine policy.
    """

    def __init__(self, params: RangeOnlyParams = RangeOnlyParams()) -> None:
        self.params = params

    def reset(self) -> None:
        # Interface symmetry with stateful controllers.
        return None

    def update(
        self,
        *,
        range_mm: float,
        target_range_mm: float,
    ) -> RangeOnlyResult:
        range_mm = self._finite_float(range_mm, name="range_mm")
        target_range_mm = self._finite_float(
            target_range_mm,
            name="target_range_mm",
        )

        if range_mm < 0.0:
            raise ValueError("range_mm must be >= 0")
        if target_range_mm < 0.0:
            raise ValueError("target_range_mm must be >= 0")

        error_mm = range_mm - target_range_mm

        if abs(error_mm) <= self.params.tolerance_mm:
            return RangeOnlyResult(
                linear_mps=0.0,
                range_mm=range_mm,
                target_range_mm=target_range_mm,
                range_error_mm=error_mm,
                reached=True,
            )

        linear_mps = self.params.kp_s_inv * (error_mm / 1000.0)
        linear_mps = self._clamp(
            linear_mps,
            -self.params.max_linear_mps,
            self.params.max_linear_mps,
        )

        return RangeOnlyResult(
            linear_mps=linear_mps,
            range_mm=range_mm,
            target_range_mm=target_range_mm,
            range_error_mm=error_mm,
            reached=False,
        )

    @staticmethod
    def _finite_float(value: float, *, name: str) -> float:
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))


__all__ = [
    "RangeOnlyController",
    "RangeOnlyParams",
    "RangeOnlyResult",
]
