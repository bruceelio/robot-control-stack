# navigation/servoing/ibvs_target_centering.py

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
        target is to the camera's right

    angular_z_rps > 0
        robot turns left / counter-clockwise
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# ==================================================
# Parameters
# ==================================================

@dataclass(frozen=True)
class TargetCenteringIBVSParams:
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
class TargetCenteringIBVSResult:
    """
    Diagnostic result from one IBVS calculation.

    normalized_x:
        Current normalized horizontal image coordinate.

    desired_normalized_x:
        Desired normalized horizontal image coordinate.

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

class TargetCenteringIBVS:
    """
    Target-centering image-based visual-servo control law.

    Controls one visual feature:
    horizontal target position.

    For a stationary differential-drive robot in translation:

        x_dot = (1 + x^2) * omega

    Standard IBVS imposes:

        e_dot = -lambda * e

    giving:

        omega = -lambda * e / (1 + x^2)

    where x is the normalized horizontal image coordinate.
    """

    def __init__(
            self,
            params: TargetCenteringIBVSParams | None = None,
    ):
        self.params = params or TargetCenteringIBVSParams()

        if self.params.gain <= 0.0:
            raise ValueError("IBVS gain must be > 0")

    @staticmethod
    def normalized_x_from_bearing(
        bearing_rad: float,
    ) -> float:
        """
        Convert horizontal camera bearing to normalized image x.

        For a pinhole camera:

            x = tan(bearing)
        """

        return math.tan(float(bearing_rad))

    def calculate(
            self,
            *,
            normalized_x: float,
            desired_normalized_x: float = 0.0,
    ) -> TargetCenteringIBVSResult:
        """
        Calculate angular velocity for horizontal target centering.

        This law assumes zero commanded translational velocity.
        """

        x = float(normalized_x)
        x_desired = float(desired_normalized_x)

        error = x - x_desired

        interaction = 1.0 + x * x

        angular_z_rps = (
            -self.params.gain
            * error
            / interaction
        )

        return TargetCenteringIBVSResult(
            normalized_x=x,
            desired_normalized_x=x_desired,
            error=error,
            interaction=interaction,
            angular_z_rps=angular_z_rps,
        )