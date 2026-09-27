# navigation/servoing/ibvs_bearing_only_tracking.py

"""
Bearing-Only Image-Based Visual Servoing (IBVS).

Tracks a target using horizontal bearing alone. The caller may provide a
translational velocity, but the visual-servo law controls angular velocity
from bearing error only.

Academic basis:
    Hutchinson, Hager & Corke (1996),
    "A Tutorial on Visual Servo Control"

    Corke, Robotics, Vision & Control, 3rd ed.,
    Chapter 15 - Image-Based Visual Servoing.

    Peter Corke Machine Vision Toolbox:
    CentralCamera.visjac_p()

The law is deliberately independent of:
    - perception implementation
    - cameras and camera calibration
    - AprilTags
    - target identity
    - range/depth estimation
    - observation freshness
    - field of view policy
    - localisation
    - arrival/success logic
    - drivetrain output

Coordinate conventions:
    bearing_rad > 0
        target is to the robot/camera's right

    angular_z_rps > 0
        robot turns left / counter-clockwise

    linear_mps > 0
        robot moves forward
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# ==================================================
# Parameters
# ==================================================

@dataclass(frozen=True)
class BearingOnlyTrackingIBVSParams:
    """
    Parameters for the bearing-only visual-servo control law.

    gain:
        Positive visual-servo gain lambda, in 1/s.
    """

    gain: float = 1.0


# ==================================================
# Result
# ==================================================

@dataclass(frozen=True)
class BearingOnlyTrackingIBVSResult:
    """
    Diagnostic result from one bearing-only IBVS calculation.

    bearing_rad:
        Current horizontal target bearing.

    desired_bearing_rad:
        Desired horizontal target bearing.

    error_rad:
        Wrapped bearing error:
            bearing_rad - desired_bearing_rad

    interaction:
        Reduced interaction term relating robot angular velocity
        to horizontal bearing rate. For rotation-only motion this is 1.

    linear_mps:
        Caller-requested translational velocity, passed through unchanged.

    angular_z_rps:
        Requested robot angular velocity.
        Positive = left / counter-clockwise.
    """

    bearing_rad: float
    desired_bearing_rad: float
    error_rad: float
    interaction: float
    linear_mps: float
    angular_z_rps: float


# ==================================================
# IBVS control law
# ==================================================

class BearingOnlyTrackingIBVS:
    """
    Bearing-only image-based visual-servo control law.

    Starting from the normalized horizontal image coordinate

        x = tan(b)

    and the rotation-only point-feature relation

        x_dot = (1 + x^2) * omega

    gives

        b_dot = omega

    because

        db/dx = 1 / (1 + x^2)

    Therefore the reduced bearing interaction term is

        L_b = 1

    and imposing

        e_dot = -lambda * e

    gives

        omega = -lambda * e

    where

        e = wrap(b - b*)

    Forward/reverse translational velocity may be supplied by the caller and
    is passed through unchanged. Translation also affects bearing according
    to target depth/range. Because this controller is deliberately
    bearing-only, it does not estimate or compensate that depth-dependent
    term; angular feedback rejects it as a disturbance.
    """

    def __init__(
        self,
        params: BearingOnlyTrackingIBVSParams | None = None,
    ):
        self.params = params or BearingOnlyTrackingIBVSParams()

        if not math.isfinite(self.params.gain) or self.params.gain <= 0.0:
            raise ValueError("IBVS gain must be finite and > 0")

    @staticmethod
    def _wrap_to_pi(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def calculate(
        self,
        *,
        bearing_rad: float,
        desired_bearing_rad: float = 0.0,
        linear_mps: float = 0.0,
    ) -> BearingOnlyTrackingIBVSResult:
        """
        Calculate angular velocity from horizontal bearing error.

        linear_mps is a caller-selected translational velocity and is passed
        through unchanged.
        """

        bearing = float(bearing_rad)
        desired = float(desired_bearing_rad)
        linear = float(linear_mps)

        if not math.isfinite(bearing):
            raise ValueError("bearing_rad must be finite")
        if not math.isfinite(desired):
            raise ValueError("desired_bearing_rad must be finite")
        if not math.isfinite(linear):
            raise ValueError("linear_mps must be finite")

        error = self._wrap_to_pi(bearing - desired)

        interaction = 1.0

        angular_z_rps = (
            -self.params.gain
            * error
            / interaction
        )

        return BearingOnlyTrackingIBVSResult(
            bearing_rad=bearing,
            desired_bearing_rad=desired,
            error_rad=error,
            interaction=interaction,
            linear_mps=linear,
            angular_z_rps=angular_z_rps,
        )