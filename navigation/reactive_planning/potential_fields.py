# navigation/reactive_planning/potential_fields

# NOT COMPLETE !!!!
# need to update all variables per standards, this is just general format
# code takes advantage of NumPy to reduce load on Raspberry Pi

import numpy as np


def calculate_repulsive_force(robot_pos, obstacle_points, k_repulse=1.0, d0=0.5):
    """
    robot_pos: [x, y]
    obstacle_points: Nx2 array of [x, y] coordinates from LIDAR/Vision
    k_repulse: gain constant
    d0: influence distance (don't push if obstacle is further than this)
    """
    # 1. Calculate distances from robot to ALL obstacles at once
    # Result: [d1, d2, d3... dN]
    diff = robot_pos - obstacle_points
    dist = np.linalg.norm(diff, axis=1)

    # 2. Only consider obstacles within 'd0' distance
    mask = dist < d0
    relevant_dist = dist[mask]
    relevant_diff = diff[mask]

    if len(relevant_dist) == 0:
        return np.array([0.0, 0.0])

    # 3. Apply the Potential Field Formula: F = k * (1/dist - 1/d0) * (1/dist^2)
    # This is done across the whole array in one operation
    inv_dist = 1.0 / relevant_dist
    scalar = k_repulse * (inv_dist - 1.0 / d0) * (inv_dist ** 2)

    # 4. Sum up all individual repulsive vectors into one final 'push'
    force_vector = np.sum(relevant_diff * scalar[:, np.newaxis], axis=0)

    return force_vector
