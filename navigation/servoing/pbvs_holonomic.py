# navigation/servoing/pbvs_holonomic.py

"""
Holonomic Position-Based Visual Servoing (PBVS).

This module is reserved for robots capable of independent planar
X, Y and yaw motion, such as a mecanum or omnidirectional drivetrain.

The mathematical implementation is intentionally deferred until
holonomic servoing is required and the non-holonomic PBVS
implementation has been validated.

Selection of this controller belongs to a higher-level controller.
This module does not inspect robot configuration or drivetrain type.
"""

from __future__ import annotations

from navigation.servoing.servoing_types import (
    HolonomicVelocity2D,
    Pose2D,
)


class HolonomicPBVS:
    """
    Placeholder for holonomic PBVS.

    Expected output:
        linear_x_mps
        linear_y_mps
        angular_z_rps
    """

    def calculate(
        self,
        *,
        current: Pose2D,
        desired: Pose2D,
    ) -> HolonomicVelocity2D:
        raise NotImplementedError(
            "Holonomic PBVS has not yet been implemented."
        )