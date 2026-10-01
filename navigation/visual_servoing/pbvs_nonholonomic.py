# navigation/visual_servoing/pbvs_nonholonomic.py

"""
Non-Holonomic Position-Based Visual Servoing (PBVS).

Controls a planar non-holonomic robot toward a desired pose using pose
feedback expressed in polar coordinates.

Academic basis:
    Corke, Robotics, Vision & Control, 3rd ed.
        Chapter 15 - Position-Based Visual Servoing
        Chapter 16 - non-holonomic mobile-robot visual visual_servoing example

The non-holonomic mobile-robot example estimates vehicle pose from vision
and applies the polar pose-control law

    v     = d * K_rho * rho
    omega = K_alpha * alpha + K_beta * beta

where d is a persistent forward/reverse direction selected at the start of
the manoeuvre.

This module implements only the planar pose-control mathematics. It is
deliberately independent of:
    - perception implementation
    - cameras and camera calibration
    - AprilTags
    - target identity
    - observation freshness
    - field-of-view policy
    - localisation source
    - arrival/success logic
    - drivetrain selection
    - motor output
    - robot configuration

Coordinate conventions:
    +x:
        forward in the pose frame

    +y:
        left in the pose frame

    heading_rad > 0:
        counter-clockwise

    linear_mps > 0:
        robot moves forward

    angular_rps > 0:
        robot turns left / counter-clockwise
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from navigation.visual_servoing.servoing_types import (
    NonHolonomicVelocity2D,
    Pose2D,
)


@dataclass(frozen=True)
class NonHolonomicPBVSParams:
    """
    Polar pose-controller gains.

    Academic reference:
        Corke, Robotics, Vision & Control, 3rd ed.
        Non-holonomic PBVS example:

            k_rho   = 1.0
            k_alpha = 5.0
            k_beta  = -2.0

    For the standard linearised polar pose controller:

        lambda^2
        + (k_alpha - k_rho) * lambda
        - k_rho * k_beta
        = 0

    Corke's gains give angular poles approximately:

        lambda = -0.586, -3.414

    while radial error decays with:

        lambda_rho = -k_rho = -1.0

    Therefore one angular mode converges more slowly than the
    radial position error.

    Current experimental gains:

        k_rho   = 1.0
        k_alpha = 3.4
        k_beta  = -1.5

    These give angular poles approximately:

        lambda = -1.20 +/- 0.245j

    so the angular-error envelope decays slightly faster than
    the radial error.

    Stability conditions for the standard polar pose controller:

        k_rho > 0
        k_beta < 0
        k_alpha > k_rho
    """

    k_rho: float = 1.0
    k_alpha: float = 3.6
    k_beta: float = -1.7


@dataclass(frozen=True)
class NonHolonomicPBVSResult:
    """
    One evaluation of the non-holonomic PBVS law.

    direction:
        +1 for forward travel.
        -1 for reverse travel.

    rho_m:
        Euclidean distance to the desired position.

    alpha_rad:
        Heading error between the robot and the selected direction of travel.

    beta_rad:
        Desired final heading relative to the selected direction of travel.

    velocity:
        Requested non-holonomic body velocity.
    """

    direction: int
    rho_m: float
    alpha_rad: float
    beta_rad: float
    velocity: NonHolonomicVelocity2D


class NonHolonomicPBVS:
    """
    Polar pose controller for a non-holonomic planar robot.

    Forward/reverse direction is selected on the first calculate() call and
    retained for the manoeuvre. This mirrors Corke's non-holonomic example
    and prevents the requested travel direction from changing from one
    control update to the next.

    Call reset() before starting a new manoeuvre.
    """

    _RHO_EPSILON_M = 1.0e-12

    def __init__(
        self,
        params: NonHolonomicPBVSParams | None = None,
    ):
        self.params = params or NonHolonomicPBVSParams()
        self._validate_params()
        self._direction: int | None = None

    @property
    def direction(self) -> int | None:
        """
        Current persistent travel direction.

        None:
            No direction has yet been selected.

        +1:
            Forward.

        -1:
            Reverse.
        """

        return self._direction

    def reset(self) -> None:
        """
        Reset manoeuvre-specific state.

        The next calculate() call will choose a new forward/reverse direction.
        """

        self._direction = None

    @staticmethod
    def _wrap_to_pi(angle_rad: float) -> float:
        return math.atan2(
            math.sin(angle_rad),
            math.cos(angle_rad),
        )

    @staticmethod
    def _validate_pose(
        pose: Pose2D,
        name: str,
    ) -> None:
        values = (
            pose.x_m,
            pose.y_m,
            pose.heading_rad,
        )

        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError(
                f"{name} pose values must all be finite"
            )

    def _validate_params(self) -> None:
        p = self.params

        if not math.isfinite(p.k_rho) or p.k_rho <= 0.0:
            raise ValueError("k_rho must be finite and > 0")

        if not math.isfinite(p.k_alpha) or p.k_alpha <= p.k_rho:
            raise ValueError(
                "k_alpha must be finite and > k_rho"
            )

        if not math.isfinite(p.k_beta) or p.k_beta >= 0.0:
            raise ValueError("k_beta must be finite and < 0")

    def calculate(
        self,
        *,
        current: Pose2D,
        desired: Pose2D,
    ) -> NonHolonomicPBVSResult:
        """
        Calculate non-holonomic velocity from current and desired planar pose.

        The current and desired poses must be expressed in the same frame.

        Polar errors:

            rho =
                distance from current position to desired position

            alpha =
                selected travel-heading error relative to current heading

            beta =
                desired final heading relative to selected travel heading

        Direction selection:

            If the desired position lies within +/- 90 degrees of the
            robot's forward heading, forward travel is selected.

            Otherwise reverse travel is selected.

        Once selected, direction remains fixed until reset() is called.

        Control law:

            linear_mps =
                direction * k_rho * rho

            angular_rps =
                k_alpha * alpha + k_beta * beta

        No velocity limits are applied here. Physical velocity constraints
        belong to the layer that executes the returned velocity.

        Arrival/success is also deliberately not decided here.
        """

        self._validate_pose(current, "current")
        self._validate_pose(desired, "desired")

        dx = float(desired.x_m) - float(current.x_m)
        dy = float(desired.y_m) - float(current.y_m)

        rho = math.hypot(dx, dy)

        # Polar coordinates are singular at exactly zero position error.
        # This is only a numerical guard; arrival/success remains the
        # responsibility of the supervising controller.
        if rho <= self._RHO_EPSILON_M:
            direction = (
                self._direction
                if self._direction is not None
                else 1
            )

            return NonHolonomicPBVSResult(
                direction=direction,
                rho_m=0.0,
                alpha_rad=0.0,
                beta_rad=0.0,
                velocity=NonHolonomicVelocity2D(
                    linear_mps=0.0,
                    angular_rps=0.0,
                ),
            )

        line_heading = math.atan2(dy, dx)

        # --------------------------------------------------
        # Persistent forward/reverse selection
        # --------------------------------------------------

        if self._direction is None:
            forward_alpha = self._wrap_to_pi(
                line_heading - float(current.heading_rad)
            )

            if (
                -math.pi / 2.0
                <= forward_alpha
                <= math.pi / 2.0
            ):
                self._direction = 1
            else:
                self._direction = -1

        direction = self._direction

        # --------------------------------------------------
        # Polar pose error
        # --------------------------------------------------

        if direction == 1:
            travel_heading = line_heading
        else:
            travel_heading = self._wrap_to_pi(
                line_heading + math.pi
            )

        alpha = self._wrap_to_pi(
            travel_heading - float(current.heading_rad)
        )

        # Corke's non-holonomic example bounds alpha to the
        # forward/reverse half-plane selected for the manoeuvre.
        alpha = max(
            -math.pi / 2.0,
            min(math.pi / 2.0, alpha),
        )

        beta = self._wrap_to_pi(
            float(desired.heading_rad) - travel_heading
        )

        # --------------------------------------------------
        # Polar control law
        # --------------------------------------------------

        linear_mps = (
            direction
            * self.params.k_rho
            * rho
        )

        angular_rps = (
            self.params.k_alpha * alpha
            + self.params.k_beta * beta
        )

        return NonHolonomicPBVSResult(
            direction=direction,
            rho_m=rho,
            alpha_rad=alpha,
            beta_rad=beta,
            velocity=NonHolonomicVelocity2D(
                linear_mps=linear_mps,
                angular_rps=angular_rps,
            ),
        )