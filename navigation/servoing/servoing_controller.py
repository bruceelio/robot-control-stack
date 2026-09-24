# navigation/servoing/servoing_controller.py

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum

from config import CONFIG
from navigation.velocity_arbiter import VelocityCommand
from skills.algorithms.pid import (
    DerivativeMode,
    PIDController,
    PIDResult,
    SHARED_PID,
)

# ==================================================
# Servoing tuning profile
# ==================================================

@dataclass(frozen=True)
class ServoingProfile:
    """
    Resolved tuning for one servoing application.

    Values come from robot/profile configuration and are resolved
    by ServoingController for the requested application.

    The PID algorithm itself remains generic and knows nothing
    about servoing applications.
    """

    linear_kp: float
    linear_ki: float
    linear_kd: float

    angular_kp: float
    angular_ki: float
    angular_kd: float

    linear_max_mm_s: float
    angular_max_rad_s: float

    target_angle_drive_slowdown_start_rad: float | None
    target_angle_drive_cutoff_rad: float
    stop_distance_tolerance_mm: float

    derivative_mode: DerivativeMode


# ==================================================
# Servoing result
# ==================================================

class ServoingStatus(Enum):
    RUNNING = "running"
    SUCCESS = "success"
    FAILED_STALE = "failed_stale"


@dataclass(frozen=True)
class ServoingResult:
    """
    Result from the non-mecanum servoing controller.

    command:
        Canonical velocity command for the Velocity Arbiter.

    status:
        Current state of the servoing operation.

    linear_pid / angular_pid:
        Full PID diagnostics when a running control update was
        performed. They are None for terminal stop results.
    """

    command: VelocityCommand
    status: ServoingStatus

    linear_pid: PIDResult | None = None
    angular_pid: PIDResult | None = None


# ==================================================
# Servoing controller
# ==================================================

class ServoingController:
    """
    Reusable closed-loop non-mecanum velocity servoing controller.

    The caller identifies the application when constructing the
    controller:

        ServoingController(application="approach_target")
        ServoingController(application="return_to_base")

    The application name has two purposes:

        1. Select application-specific servoing tuning.
        2. Give the shared PID engine independent loop IDs.

    The caller supplies the task geometry:

        distance_mm
            Current distance measurement.

        stop_distance_mm
            Desired distance setpoint.

        target_angle_rad
            Current angular measurement.

        target_angle_setpoint_rad
            Desired angular setpoint. Defaults to 0 rad.

        timestamp
            Timestamp of the measurement.

    ServoingController owns:

        - observation freshness
        - completion tolerance
        - PID calls
        - velocity limiting
        - angular drive gating
        - conversion to VelocityCommand

    It remains independent of:

        - target identity
        - AprilTag identity
        - camera implementation
        - localisation
        - path planning
        - behaviour/state-machine logic

    One shared PID engine is used across the robot process.
    Its state remains independent because each application and
    axis uses a different loop_id.
    """

    def __init__(
        self,
        *,
        application: str,
        config=CONFIG,
        pid_controller: PIDController = SHARED_PID,
    ):
        application = str(application).strip()

        if not application:
            raise ValueError(
                "ServoingController application must not be empty"
            )

        self.application = application
        self.config = config
        self.pid = pid_controller

        self.profile = self._resolve_profile(
            application=application,
        )

        self._linear_loop_id = (
            f"servoing.{self.application}.linear"
        )
        self._angular_loop_id = (
            f"servoing.{self.application}.angular"
        )

        # A new servoing controller starts with clean PID history,
        # even though the underlying PID engine is shared.
        self.reset()

    # ==================================================
    # Public API
    # ==================================================

    def reset(self) -> None:
        """
        Reset PID history for this servoing application only.
        """

        self.pid.reset(self._linear_loop_id)
        self.pid.reset(self._angular_loop_id)

    def update(
        self,
        *,
        distance_mm: float,
        target_angle_rad: float,
        timestamp: float,
        stop_distance_mm: float,
        target_angle_setpoint_rad: float = 0.0,
        now: float | None = None,
    ) -> ServoingResult:
        """
        Perform one servoing update.

        Existing object-approach behaviour is represented by:

            target_angle_setpoint_rad = 0.0

        Other applications may request a non-zero angular setpoint.
        For example, return-to-base may deliberately hold an arena
        guide tag toward one side of the camera.
        """

        if now is None:
            now = time.time()

        distance_mm = float(distance_mm)
        stop_distance_mm = float(stop_distance_mm)

        target_angle_rad = float(target_angle_rad)
        target_angle_setpoint_rad = float(
            target_angle_setpoint_rad
        )

        timestamp = float(timestamp)
        now = float(now)

        # --------------------------------------------------
        # Observation freshness
        # --------------------------------------------------

        if (now - timestamp) > self.config.visible_max_age_s:
            self.reset()

            return self._stop(
                status=ServoingStatus.FAILED_STALE,
                timestamp=timestamp,
            )

        # --------------------------------------------------
        # Distance from requested stopping point
        # --------------------------------------------------
        #
        # Keep the existing servoing completion semantics:
        #
        #     remaining = current distance - requested stop
        #
        # If the robot is within the positive tolerance, or has
        # already reached/passed the requested stop distance,
        # servoing is complete.
        # --------------------------------------------------

        remaining_distance_mm = (
            distance_mm - stop_distance_mm
        )

        if (
            remaining_distance_mm
            <= self.profile.stop_distance_tolerance_mm
        ):
            self.reset()

            return self._stop(
                status=ServoingStatus.SUCCESS,
                timestamp=timestamp,
            )

        # --------------------------------------------------
        # Linear PID
        # --------------------------------------------------
        #
        # PID uses the natural process definition:
        #
        #     setpoint    = desired stopping distance
        #     measurement = current distance
        #
        # Therefore the PID error is negative while the robot is
        # too far away:
        #
        #     error = stop_distance - current_distance
        #
        # Positive linear_x means drive forward, which normally
        # REDUCES the distance measurement. The controlled plant
        # therefore has a negative direction, so the PID output is
        # inverted here when converted into forward velocity.
        # --------------------------------------------------

        linear_pid = self.pid.update(
            loop_id=self._linear_loop_id,
            setpoint=stop_distance_mm,
            measurement=distance_mm,
            timestamp=timestamp,
            kp=self.profile.linear_kp,
            ki=self.profile.linear_ki,
            kd=self.profile.linear_kd,
            derivative_mode=self.profile.derivative_mode,
        )

        linear_mm_s = -linear_pid.output

        # Current non-mecanum servoing intentionally drives
        # forward only. Reaching/passing the stop distance is
        # handled as SUCCESS above.
        linear_mm_s = self._clamp(
            linear_mm_s,
            0.0,
            self.profile.linear_max_mm_s,
        )

        # --------------------------------------------------
        # Angular PID
        # --------------------------------------------------
        #
        # The previous controller used:
        #
        #     angular_z = -Kp * target_angle
        #
        # With:
        #
        #     setpoint = 0
        #     measurement = target_angle
        #
        # PID naturally gives:
        #
        #     error = -target_angle
        #
        # so the existing steering sign is preserved.
        #
        # A caller such as ReturnToBaseServo can instead provide a
        # non-zero target_angle_setpoint_rad.
        # --------------------------------------------------

        angular_pid = self.pid.update(
            loop_id=self._angular_loop_id,
            setpoint=target_angle_setpoint_rad,
            measurement=target_angle_rad,
            timestamp=timestamp,
            kp=self.profile.angular_kp,
            ki=self.profile.angular_ki,
            kd=self.profile.angular_kd,
            derivative_mode=self.profile.derivative_mode,
        )

        angular_z_rps = angular_pid.output

        angular_z_rps = self._clamp(
            angular_z_rps,
            -self.profile.angular_max_rad_s,
            self.profile.angular_max_rad_s,
        )

        # --------------------------------------------------
        # Angular drive gating
        # --------------------------------------------------
        #
        # Gate on angular ERROR rather than absolute target bearing.
        # This preserves old behaviour for a 0-rad setpoint and
        # also works correctly for applications which deliberately
        # use a non-zero angular setpoint.
        # --------------------------------------------------

        angular_error_rad = (
                target_angle_setpoint_rad
                - target_angle_rad
        )

        angular_error_abs_rad = abs(
            angular_error_rad
        )

        slowdown_start_rad = (
            self.profile.target_angle_drive_slowdown_start_rad
        )

        cutoff_rad = (
            self.profile.target_angle_drive_cutoff_rad
        )

        if angular_error_abs_rad >= cutoff_rad:
            linear_mm_s = 0.0

        elif (
                slowdown_start_rad is not None
                and angular_error_abs_rad > slowdown_start_rad
        ):
            slowdown_span_rad = (
                    cutoff_rad - slowdown_start_rad
            )

            if slowdown_span_rad > 0.0:
                linear_scale = (
                                       cutoff_rad - angular_error_abs_rad
                               ) / slowdown_span_rad

                linear_mm_s *= linear_scale

        # --------------------------------------------------
        # Canonical velocity command
        # --------------------------------------------------

        command = VelocityCommand(
            linear_x_mps=linear_mm_s / 1000.0,
            angular_z_rps=angular_z_rps,
            lateral_y_mps=0.0,
            timestamp=timestamp,
        )

        return ServoingResult(
            command=command,
            status=ServoingStatus.RUNNING,
            linear_pid=linear_pid,
            angular_pid=angular_pid,
        )

    # ==================================================
    # Profile resolution
    # ==================================================

    def _resolve_profile(
        self,
        *,
        application: str,
    ) -> ServoingProfile:
        """
        Resolve application-specific servoing tuning from CONFIG.

        ServoingController owns the common control algorithm.
        Robot/profile configuration owns the tuning for each
        servoing application.
        """

        if application == "approach_target":
            derivative_mode = (
                self.config.approach_target_servo_derivative_mode
            )

            if derivative_mode not in ("error", "measurement"):
                raise ValueError(
                    "Invalid approach_target servo derivative mode: "
                    f"{derivative_mode!r}"
                )

            return ServoingProfile(
                linear_kp=float(
                    self.config.approach_target_servo_linear_kp
                ),
                linear_ki=float(
                    self.config.approach_target_servo_linear_ki
                ),
                linear_kd=float(
                    self.config.approach_target_servo_linear_kd
                ),

                angular_kp=float(
                    self.config.approach_target_servo_angular_kp
                ),
                angular_ki=float(
                    self.config.approach_target_servo_angular_ki
                ),
                angular_kd=float(
                    self.config.approach_target_servo_angular_kd
                ),

                linear_max_mm_s=float(
                    self.config.approach_target_servo_linear_max_mm_s
                ),
                angular_max_rad_s=float(
                    self.config.approach_target_servo_angular_max_rad_s
                ),

                target_angle_drive_slowdown_start_rad=None,

                target_angle_drive_cutoff_rad=math.radians(
                    float(
                        self.config
                        .approach_target_servo_drive_cutoff_deg
                    )
                ),

                stop_distance_tolerance_mm=float(
                    self.config
                    .approach_target_servo_stop_tolerance_mm
                ),

                derivative_mode=derivative_mode,
            )

        if application == "return_to_base":
            derivative_mode = (
                self.config.return_to_base_servo_derivative_mode
            )

            if derivative_mode not in ("error", "measurement"):
                raise ValueError(
                    "Invalid return_to_base servo derivative mode: "
                    f"{derivative_mode!r}"
                )

            return ServoingProfile(
                linear_kp=float(
                    self.config.return_to_base_servo_linear_kp
                ),
                linear_ki=float(
                    self.config.return_to_base_servo_linear_ki
                ),
                linear_kd=float(
                    self.config.return_to_base_servo_linear_kd
                ),

                angular_kp=float(
                    self.config.return_to_base_servo_angular_kp
                ),
                angular_ki=float(
                    self.config.return_to_base_servo_angular_ki
                ),
                angular_kd=float(
                    self.config.return_to_base_servo_angular_kd
                ),

                linear_max_mm_s=float(
                    self.config.return_to_base_servo_linear_max_mm_s
                ),
                angular_max_rad_s=float(
                    self.config.return_to_base_servo_angular_max_rad_s
                ),

                target_angle_drive_slowdown_start_rad=math.radians(
                    float(
                        self.config
                        .return_to_base_servo_drive_slowdown_start_deg
                    )
                ),

                target_angle_drive_cutoff_rad=math.radians(
                    float(
                        self.config
                        .return_to_base_servo_drive_cutoff_deg
                    )
                ),

                stop_distance_tolerance_mm=float(
                    self.config
                    .return_to_base_servo_stop_tolerance_mm
                ),

                derivative_mode=derivative_mode,
            )

        raise KeyError(
            "No servoing configuration defined for "
            f"application={application!r}"
        )

    # ==================================================
    # Helpers
    # ==================================================

    @staticmethod
    def _stop(
        *,
        status: ServoingStatus,
        timestamp: float,
    ) -> ServoingResult:
        command = VelocityCommand(
            linear_x_mps=0.0,
            angular_z_rps=0.0,
            lateral_y_mps=0.0,
            timestamp=timestamp,
        )

        return ServoingResult(
            command=command,
            status=status,
        )

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
