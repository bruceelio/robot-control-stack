# navigation/local_planning/artificial_potential_field.py

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ArtificialPotentialFieldParams:
    """
    Parameters for a two-dimensional artificial potential field.

    Positions and distances are expressed in metres.

    attractive_gain:
        Strength of attraction toward the goal.

    repulsive_gain:
        Strength of repulsion from obstacles.

    obstacle_influence_distance_m:
        Obstacles farther away than this distance contribute no
        repulsive force.
    """

    attractive_gain: float = 1.0
    repulsive_gain: float = 1.0
    obstacle_influence_distance_m: float = 0.5


@dataclass(frozen=True)
class ArtificialPotentialFieldResult:
    """
    Components of an artificial potential field evaluation.
    """

    attractive: np.ndarray
    repulsive: np.ndarray
    resultant: np.ndarray


def attractive_force(
    *,
    position: np.ndarray,
    goal: np.ndarray,
    gain: float = 1.0,
) -> np.ndarray:
    """
    Compute the attractive vector toward a goal.

    The attractive field is linear:

        F_att = k_att * (goal - position)
    """

    position = _vector_2d(
        position,
        name="position",
    )

    goal = _vector_2d(
        goal,
        name="goal",
    )

    gain = float(gain)

    if not np.isfinite(gain):
        raise ValueError(
            "gain must be finite"
        )

    if gain < 0.0:
        raise ValueError(
            "gain must be >= 0"
        )

    return gain * (goal - position)


def repulsive_force(
    *,
    position: np.ndarray,
    obstacle_points: np.ndarray,
    gain: float = 1.0,
    influence_distance_m: float = 0.5,
) -> np.ndarray:
    """
    Compute the summed repulsive vector from nearby obstacle points.

    For obstacle distance rho within influence distance rho_0:

        F_rep =
            k_rep
            * (1/rho - 1/rho_0)
            * (1/rho^2)
            * unit_vector_away_from_obstacle

    Obstacles at or beyond rho_0 contribute zero force.
    """

    position = _vector_2d(
        position,
        name="position",
    )

    obstacle_points = _points_2d(
        obstacle_points,
        name="obstacle_points",
    )

    gain = float(gain)
    influence_distance_m = float(
        influence_distance_m
    )

    if not np.isfinite(gain):
        raise ValueError(
            "gain must be finite"
        )

    if gain < 0.0:
        raise ValueError(
            "gain must be >= 0"
        )

    if (
        not np.isfinite(influence_distance_m)
        or influence_distance_m <= 0.0
    ):
        raise ValueError(
            "influence_distance_m must be finite and > 0"
        )

    if obstacle_points.shape[0] == 0:
        return np.zeros(
            2,
            dtype=float,
        )

    # Vector from each obstacle toward the robot.
    displacement = (
        position[np.newaxis, :]
        - obstacle_points
    )

    distance = np.linalg.norm(
        displacement,
        axis=1,
    )

    # Avoid division by zero if an obstacle point coincides exactly
    # with the robot position.
    epsilon_m = 1e-9

    relevant = (
        distance
        < influence_distance_m
    )

    if not np.any(relevant):
        return np.zeros(
            2,
            dtype=float,
        )

    relevant_distance = np.maximum(
        distance[relevant],
        epsilon_m,
    )

    relevant_displacement = (
        displacement[relevant]
    )

    direction_away = (
        relevant_displacement
        / relevant_distance[:, np.newaxis]
    )

    inverse_distance = (
        1.0 / relevant_distance
    )

    magnitude = (
        gain
        * (
            inverse_distance
            - 1.0 / influence_distance_m
        )
        * inverse_distance**2
    )

    return np.sum(
        direction_away
        * magnitude[:, np.newaxis],
        axis=0,
    )


def artificial_potential_field(
    *,
    position: np.ndarray,
    goal: np.ndarray,
    obstacle_points: np.ndarray,
    params: ArtificialPotentialFieldParams,
) -> ArtificialPotentialFieldResult:
    """
    Evaluate the attractive and repulsive components of a 2-D
    artificial potential field.

    This function performs planning mathematics only.

    It does not:
        - read sensors
        - know obstacle source
        - know robot hardware
        - choose drivetrain commands
        - apply velocity limits
        - perform path tracking
    """

    attractive = attractive_force(
        position=position,
        goal=goal,
        gain=params.attractive_gain,
    )

    repulsive = repulsive_force(
        position=position,
        obstacle_points=obstacle_points,
        gain=params.repulsive_gain,
        influence_distance_m=(
            params.obstacle_influence_distance_m
        ),
    )

    resultant = (
        attractive + repulsive
    )

    return ArtificialPotentialFieldResult(
        attractive=attractive,
        repulsive=repulsive,
        resultant=resultant,
    )


def _vector_2d(
    value,
    *,
    name: str,
) -> np.ndarray:
    vector = np.asarray(
        value,
        dtype=float,
    )

    if vector.shape != (2,):
        raise ValueError(
            f"{name} must have shape (2,)"
        )

    if not np.all(np.isfinite(vector)):
        raise ValueError(
            f"{name} must contain finite values"
        )

    return vector


def _points_2d(
    value,
    *,
    name: str,
) -> np.ndarray:
    points = np.asarray(
        value,
        dtype=float,
    )

    if points.size == 0:
        return np.empty(
            (0, 2),
            dtype=float,
        )

    if (
        points.ndim != 2
        or points.shape[1] != 2
    ):
        raise ValueError(
            f"{name} must have shape (N, 2)"
        )

    if not np.all(np.isfinite(points)):
        raise ValueError(
            f"{name} must contain finite values"
        )

    return points