# navigation/visual_servoing/ibvs_bearing_feature.py

"""
Bearing-Feature Image-Based Visual Servoing (IBVS).

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
        target is to the robot/camera's left / counter-clockwise

    angular_z_rps > 0
        robot turns left / counter-clockwise

    linear_mps > 0
        robot moves forward

This module uses the robot-wide canonical navigation convention.
Raw image/detector coordinates may use the opposite horizontal sign;
that conversion belongs at the perception boundary.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# ==================================================
# Parameters
# ==================================================

@dataclass(frozen=True)
class BearingFeatureIBVSParams:
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
class BearingFeatureIBVSResult:
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
        to horizontal bearing rate.

        Under the canonical positive-left bearing convention,
        rotation-only motion gives:

            bearing_dot = -angular_z_rps

        so this interaction term is -1.

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

class BearingFeatureIBVS:
    """
    Bearing-feature image-based visual servoing control law

        Using the canonical positive-left horizontal bearing:

        b > 0
            target is left

        omega > 0
            robot turns left

    For rotation-only motion of a stationary target:

        b_dot = -omega

    Therefore the reduced bearing interaction term is

        L_b = -1

    Imposing

        e_dot = -lambda * e

    with

        e = wrap(b - b*)

    gives

        omega = lambda * e

    Equivalently, using the standard interaction-law form:

        omega = -lambda * e / L_b

    with:

        L_b = -1

    where

        e = wrap(b - b*)

    Forward/reverse translational velocity may be supplied by the caller and
    is passed through unchanged.

    Under the canonical positive-left convention, translation contributes:

        b_dot_translation = (sin(b) / range) * v

    Because this controller is deliberately bearing-only, it does not estimate
    target depth/range and therefore does not explicitly compensate this
    translation-dependent term. Angular feedback rejects it as a disturbance.
    """

    def __init__(
        self,
        params: BearingFeatureIBVSParams | None = None,
    ):
        self.params = params or BearingFeatureIBVSParams()

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
    ) -> BearingFeatureIBVSResult:
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

        interaction = -1.0

        angular_z_rps = (
            -self.params.gain
            * error
            / interaction
        )

        return BearingFeatureIBVSResult(
            bearing_rad=bearing,
            desired_bearing_rad=desired,
            error_rad=error,
            interaction=interaction,
            linear_mps=linear,
            angular_z_rps=angular_z_rps,
        )