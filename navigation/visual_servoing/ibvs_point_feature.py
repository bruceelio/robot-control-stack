# navigation/visual_servoing/ibvs_point_feature.py

"""
Target-Centering Image-Based Visual Servoing (IBVS).

Horizontal image-feature centering using a reduced point-feature
interaction matrix.

Academic basis:
    Hutchinson, Hager & Corke (1996),
    "A Tutorial on Visual Servo Control"

    Corke, Robotics, Vision & Control, 3rd ed.,
    Section 15.2.1 - point-feature interaction matrix.

    Peter Corke Machine Vision Toolbox:
    CentralCamera.visjac_p()

The law is deliberately independent of:
    - perception implementation
    - AprilTags
    - target identity
    - observation freshness
    - field of view policy
    - localisation
    - arrival/success logic
    - drivetrain output

Coordinate conventions:
    normalized_x > 0
        target is to the camera's left

    bearing_rad > 0
        target is to the camera's left / counter-clockwise

    angular_z_rps > 0
        robot turns left / counter-clockwise

Raw image/pixel coordinates may use a different convention. Any such
conversion belongs at the perception boundary, not inside this control law.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# ==================================================
# Parameters
# ==================================================

@dataclass(frozen=True)
class PointFeatureIBVSParams:
    """
    Parameters for the image-based visual-servo control law.

    gain:
        Positive visual-servo gain lambda.
    """

    gain: float = 1.0


# ==================================================
# Result
# ==================================================

@dataclass(frozen=True)
class PointFeatureIBVSResult:
    """
    Diagnostic result from one IBVS calculation.

        normalized_x:
        Current canonical normalized horizontal coordinate.
        Positive = target left.

    desired_normalized_x:
        Desired canonical normalized horizontal coordinate.
        Positive = target left.

    error:
        Image-feature error:
            normalized_x - desired_normalized_x

    interaction:
        Reduced interaction-matrix term relating robot angular
        velocity to horizontal image velocity.

    angular_z_rps:
        Requested robot angular velocity.
        Positive = left / counter-clockwise.
    """

    normalized_x: float
    desired_normalized_x: float
    error: float
    interaction: float
    angular_z_rps: float


# ==================================================
# IBVS control law
# ==================================================

class PointFeatureIBVS:
    """
    Target-centering image-based visual-servo control law.

    Controls one visual feature:
    horizontal target position.

    For a stationary target and canonical positive-left bearing:

    x_dot = -(1 + x^2) * omega

    where:

    x > 0       target is left
    omega > 0   robot turns left

    Standard IBVS imposes:

    e_dot = -lambda * e

    so the reduced interaction term is:

    L = -(1 + x^2)

    and:

    omega = -lambda * e / L
          =  lambda * e / (1 + x^2)

    where x is the normalized horizontal image coordinate.
    """

    def __init__(
            self,
            params: PointFeatureIBVSParams | None = None,
    ):
        self.params = params or PointFeatureIBVSParams()

        if self.params.gain <= 0.0:
            raise ValueError("IBVS gain must be > 0")

    @staticmethod
    def normalized_x_from_bearing(
        bearing_rad: float,
    ) -> float:
        """
                Convert canonical horizontal bearing to the normalized horizontal
        coordinate used by this controller.

        Canonical convention:

            bearing > 0
                target is left / counter-clockwise

            normalized_x > 0
                target is left

        Therefore:

            normalized_x = tan(bearing)

        Note that conventional image coordinates and many IBVS references use
        image x positive to the RIGHT. Under that convention this controller's
        normalized_x is the negative of native image x.

        The sign adaptation is intentional so that the public navigation and
        visual-servo interfaces use the robot-wide positive-left convention.
        """

        return math.tan(float(bearing_rad))

    def calculate(
            self,
            *,
            normalized_x: float,
            desired_normalized_x: float = 0.0,
    ) -> PointFeatureIBVSResult:
        """
        Calculate angular velocity for horizontal target centering.

        This law assumes zero commanded translational velocity.
        """

        x = float(normalized_x)
        x_desired = float(desired_normalized_x)

        error = x - x_desired

        interaction = -(1.0 + x * x)

        angular_z_rps = (
                -self.params.gain
                * error
                / interaction
        )

        return PointFeatureIBVSResult(
            normalized_x=x,
            desired_normalized_x=x_desired,
            error=error,
            interaction=interaction,
            angular_z_rps=angular_z_rps,
        )