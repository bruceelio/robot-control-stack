# navigation/servoing/servoing_controller.py

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import Enum

from config import CONFIG
from navigation.velocity_arbiter import VelocityCommand


# ==================================================
# Controller tuning
# ==================================================

# Proportional response to remaining distance.
# 1.0 = 1 mm/s commanded for every 1 mm of distance error.
LINEAR_KP = 1.0

# Proportional response to target angular error.
ANGULAR_KP = 2.0

# If the target is too far from straight ahead, rotate toward it
# before commanding forward motion.
TARGET_ANGLE_DRIVE_CUTOFF_RAD = math.radians(60.0)

STOP_DISTANCE_TOLERANCE_MM = 10.0

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
    """

    command: VelocityCommand
    status: ServoingStatus


# ==================================================
# Servoing controller
# ==================================================

class ServoingController:
    """
    Closed-loop non-mecanum servoing toward a perceived target.

    Inputs:
        distance_mm
            Current perceived distance to the target.

        target_angle_rad
            Current angle from the robot's forward direction
            to the target.

            0 rad = target directly ahead.

        timestamp
            Timestamp of the perception observation.

        stop_distance_mm
            Distance at which the requested servoing movement
            is considered complete.

    Outputs:
        linear_x_mps
        angular_z_rps
        lateral_y_mps = 0.0
        timestamp

    This controller uses forward/reverse movement and rotation only.

    It does not use lateral/strafe motion. A future mecanum-specific
    servoing controller may provide lateral_y_mps independently.

    The controller is deliberately independent of localisation,
    global heading, path planning, trajectory planning, target
    identity, and camera implementation.

    If perception becomes stale, zero velocity is commanded and
    the controller reports FAILED_STALE.
    """

    def update(
        self,
        *,
        distance_mm: float,
        target_angle_rad: float,
        timestamp: float,
        stop_distance_mm: float,
        now: float | None = None,
    ) -> ServoingResult:

        if now is None:
            now = time.time()

        # --------------------------------------------------
        # Perception freshness
        # --------------------------------------------------

        if (now - timestamp) > CONFIG.visible_max_age_s:
            return self._stop(
                status=ServoingStatus.FAILED_STALE,
                timestamp=timestamp,
            )

        # --------------------------------------------------
        # Distance from requested stopping point
        # --------------------------------------------------

        distance_error_mm = distance_mm - stop_distance_mm

        if distance_error_mm <= STOP_DISTANCE_TOLERANCE_MM:
            return self._stop(
                status=ServoingStatus.SUCCESS,
                timestamp=timestamp,
            )

        # --------------------------------------------------
        # Angular control
        # --------------------------------------------------

        angular_z_rps = -ANGULAR_KP * target_angle_rad

        angular_z_rps = self._clamp(
            angular_z_rps,
            -CONFIG.servoing_angular_max_rad_s,
            CONFIG.servoing_angular_max_rad_s,
        )

        # --------------------------------------------------
        # Linear control
        # --------------------------------------------------

        linear_mm_s = LINEAR_KP * distance_error_mm

        linear_mm_s = self._clamp(
            linear_mm_s,
            0.0,
            CONFIG.servoing_linear_max_mm_s,
        )

        # If the target is badly off-centre, rotate toward it
        # before continuing forward.
        if abs(target_angle_rad) >= TARGET_ANGLE_DRIVE_CUTOFF_RAD:
            linear_mm_s = 0.0

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
        return max(minimum, min(maximum, value))