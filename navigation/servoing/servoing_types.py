# navigation/servoing/servoing_types.py

"""
Common mathematical types for servoing control laws.

These types are deliberately independent of:
    - perception
    - localisation
    - robot configuration
    - drivetrain layout
    - hardware
    - motor output
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    """
    Planar pose.

    x_m:
        Position along +x, in metres.

    y_m:
        Position along +y, in metres.

    heading_rad:
        Counter-clockwise rotation from +x, in radians.
    """

    x_m: float
    y_m: float
    heading_rad: float


@dataclass(frozen=True)
class NonHolonomicVelocity2D:
    """
    Planar velocity for a non-holonomic mobile robot.

    linear_mps:
        Forward/reverse velocity.

    angular_rps:
        Counter-clockwise angular velocity.
    """

    linear_mps: float
    angular_rps: float


@dataclass(frozen=True)
class HolonomicVelocity2D:
    """
    Planar velocity for a holonomic mobile robot.

    linear_x_mps:
        Forward/reverse velocity.

    linear_y_mps:
        Left/right lateral velocity.

    angular_z_rps:
        Counter-clockwise angular velocity.
    """

    linear_x_mps: float
    linear_y_mps: float
    angular_z_rps: float