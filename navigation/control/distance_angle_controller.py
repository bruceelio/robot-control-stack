# navigation/control/distance_angle_controller.py

from __future__ import annotations

from dataclasses import dataclass

from skills.algorithms.pid import (
    DerivativeMode,
    PIDController,
    PIDResult,
    SHARED_PID,
)

# ==================================================
# Distance-angle parameters
# ==================================================

@dataclass(frozen=True)
class DistanceAngleParams:
    """
    Tuning and limits for generic distance-angle control.

    The controller knows nothing about the source of the distance
    or angle measurements, or about the application using them.
    """

    linear_kp: float
    linear_ki: float
    linear_kd: float

    angular_kp: float
    angular_ki: float
    angular_kd: float

    linear_max_mps: float
    angular_max_rad_s: float

    angle_drive_slowdown_start_rad: float | None
    angle_drive_cutoff_rad: float
    distance_tolerance_m: float

    derivative_mode: DerivativeMode


# ==================================================
# Distance-angle result
# ==================================================

@dataclass(frozen=True)
class DistanceAngleResult:
    """
    Result from one distance-angle control update.
    """

    linear_mps: float
    angular_rps: float

    distance_error_m: float
    angle_error_rad: float

    distance_reached: bool

    linear_pid: PIDResult | None = None
    angular_pid: PIDResult | None = None


# ==================================================
# Distance-angle controller
# ==================================================

class DistanceAngleController:
    """
    Generic closed-loop distance-angle controller.

    The caller supplies:

        distance_m
            Current distance measurement.

        target_distance_m
            Desired distance.

        angle_rad
            Current angular measurement.

        target_angle_rad
            Desired angle.

        timestamp
            Timestamp supplied to the shared PID engine.

    The controller owns:

        - distance PID
        - angular PID
        - velocity limiting
        - distance completion tolerance
        - angular drive slowdown/cutoff

    It deliberately knows nothing about:

        - cameras or vision
        - range-sensor type
        - target identity
        - localisation
        - behaviour/state-machine policy
        - observation freshness
        - robot configuration
        - command arbitration

    PID state is separated by controller_id.
    """

    def __init__(
        self,
        *,
        params: DistanceAngleParams,
        controller_id: str,
        pid_controller: PIDController = SHARED_PID,
    ):
        controller_id = str(controller_id).strip()

        if not controller_id:
            raise ValueError(
                "DistanceAngleController controller_id must not be empty"
            )

        if params.derivative_mode not in (
            "error",
            "measurement",
        ):
            raise ValueError(
                "Invalid derivative_mode: "
                f"{params.derivative_mode!r}"
            )

        self.params = params
        self.controller_id = controller_id
        self.pid = pid_controller

        self._linear_loop_id = (
            f"distance_angle.{self.controller_id}.linear"
        )

        self._angular_loop_id = (
            f"distance_angle.{self.controller_id}.angular"
        )

        self.reset()

    # ==================================================
    # Public API
    # ==================================================

    def reset(self) -> None:
        """
        Reset this controller instance's PID history.
        """

        self.pid.reset(self._linear_loop_id)
        self.pid.reset(self._angular_loop_id)

    def update(
            self,
            *,
            distance_m: float,
            target_distance_m: float,
            angle_rad: float,
            timestamp: float,
            target_angle_rad: float = 0.0,
    ) -> DistanceAngleResult:
        """
        Perform one distance-angle control update.

        Observation freshness is deliberately the caller's
        responsibility.
        """

        distance_m = float(distance_m)
        target_distance_m = float(
            target_distance_m
        )

        angle_rad = float(angle_rad)
        target_angle_rad = float(
            target_angle_rad
        )

        timestamp = float(timestamp)

        # --------------------------------------------------
        # Errors
        # --------------------------------------------------

        distance_error_m = (
                distance_m - target_distance_m
        )

        # Canonical robot geometry:
        #
        #     positive angle error = target is left of
        #     the requested bearing.
        #
        # Positive robot angular velocity is also left / CCW.
        angle_error_rad = (
                angle_rad - target_angle_rad
        )

        # --------------------------------------------------
        # Distance completion
        # --------------------------------------------------
        #
        # Preserve the existing one-sided semantics:
        #
        #     current - target <= tolerance
        #
        # Reaching or passing the requested target distance is
        # therefore considered complete.
        # --------------------------------------------------

        if (
                distance_error_m
                <= self.params.distance_tolerance_m
        ):
            self.reset()

            return DistanceAngleResult(
                linear_mps=0.0,
                angular_rps=0.0,
                distance_error_m=distance_error_m,
                angle_error_rad=angle_error_rad,
                distance_reached=True,
            )

        # --------------------------------------------------
        # Linear PID
        # --------------------------------------------------

        linear_pid = self.pid.update(
            loop_id=self._linear_loop_id,
            setpoint=target_distance_m,
            measurement=distance_m,
            timestamp=timestamp,
            kp=self.params.linear_kp,
            ki=self.params.linear_ki,
            kd=self.params.linear_kd,
            derivative_mode=self.params.derivative_mode,
        )

        # Positive forward velocity reduces target range,
        # so the range-control plant has negative direction.
        linear_mps = -linear_pid.output

        # Preserve forward-only behaviour.
        linear_mps = self._clamp(
            linear_mps,
            0.0,
            self.params.linear_max_mps,
        )

        # --------------------------------------------------
        # Angular PID
        # --------------------------------------------------

        angular_pid = self.pid.update(
            loop_id=self._angular_loop_id,
            setpoint=target_angle_rad,
            measurement=angle_rad,
            timestamp=timestamp,
            kp=self.params.angular_kp,
            ki=self.params.angular_ki,
            kd=self.params.angular_kd,
            derivative_mode=self.params.derivative_mode,
        )

        # Positive robot yaw reduces a positive-left
        # relative target bearing.
        #
        # Therefore, like range control, the controlled
        # plant has negative direction and the PID output
        # must be inverted.
        angular_rps = self._clamp(
            -angular_pid.output,
            -self.params.angular_max_rad_s,
            self.params.angular_max_rad_s,
        )

        # --------------------------------------------------
        # Angular-error drive gating
        # --------------------------------------------------

        angle_error_abs_rad = abs(
            angle_error_rad
        )

        slowdown_start_rad = (
            self.params.angle_drive_slowdown_start_rad
        )

        cutoff_rad = (
            self.params.angle_drive_cutoff_rad
        )

        if angle_error_abs_rad >= cutoff_rad:
            linear_mps = 0.0

        elif (
                slowdown_start_rad is not None
                and angle_error_abs_rad > slowdown_start_rad
        ):
            slowdown_span_rad = (
                    cutoff_rad - slowdown_start_rad
            )

            if slowdown_span_rad > 0.0:
                linear_scale = (
                                       cutoff_rad - angle_error_abs_rad
                               ) / slowdown_span_rad

                linear_mps *= linear_scale

        return DistanceAngleResult(
            linear_mps=linear_mps,
            angular_rps=angular_rps,
            distance_error_m=distance_error_m,
            angle_error_rad=angle_error_rad,
            distance_reached=False,
            linear_pid=linear_pid,
            angular_pid=angular_pid,
        )

    # ==================================================
    # Helpers
    # ==================================================



    @staticmethod
    def _clamp(
        value: float,
        minimum: float,
        maximum: float,
    ) -> float:
        return max(
            minimum,
            min(maximum, value),
        )
