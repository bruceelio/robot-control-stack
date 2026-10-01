# navigation/geometry/relative_goal_pose.py

from __future__ import annotations

import math

from navigation.geometry.transforms_2d import (
    rotate_vector_2d,
)

"""
Generic construction of a desired relative robot pose.

All geometry supplied to this module is already expressed in the
robot's CURRENT base_link frame.

Coordinate convention:
    +x       = forward
    +y       = left
    +heading = counter-clockwise / left

Units:
    position = metres
    heading  = radians

This module does not know how the reference point or heading were
obtained. They may come from perception, localisation, docking
geometry, object geometry, or any other source.
"""


def _wrap_angle_rad(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi]."""

    return math.atan2(
        math.sin(angle_rad),
        math.cos(angle_rad),
    )


def relative_goal_pose_from_reference(
    *,
    reference_x_m: float,
    reference_y_m: float,
    controlled_heading_rad: float,
    standoff_m: float,
    controlled_frame_x_m: float = 0.0,
    controlled_frame_y_m: float = 0.0,
    controlled_frame_yaw_rad: float = 0.0,
) -> tuple[float, float, float]:
    """
    Construct a desired relative base_link goal pose from a
    geometric reference.

    The reference point defines WHERE the controlled frame is aiming.

    controlled_heading_rad defines the desired heading of the
    controlled frame in the CURRENT base_link coordinate system.

    A positive standoff places the controlled-frame origin behind the
    reference point along the negative controlled-heading direction.

    controlled_frame_x_m / y_m / yaw_rad describe the fixed pose of
    the controlled frame relative to base_link.

    For example, the controlled frame could be:
        - base_link itself
        - a gripper
        - a docking point
        - another tool or mechanism

    Returns:
    (
        goal_base_x_m,
        goal_base_y_m,
        goal_base_heading_rad,
    )

    relative to the robot's CURRENT base_link frame.
    """

    values = (
        reference_x_m,
        reference_y_m,
        controlled_heading_rad,
        standoff_m,
        controlled_frame_x_m,
        controlled_frame_y_m,
        controlled_frame_yaw_rad,
    )

    if not all(
        math.isfinite(float(value))
        for value in values
    ):
        raise ValueError(
            "pose reference inputs must all be finite"
        )

    reference_x_m = float(reference_x_m)
    reference_y_m = float(reference_y_m)

    controlled_heading_rad = _wrap_angle_rad(
        float(controlled_heading_rad)
    )

    standoff_m = float(standoff_m)

    controlled_frame_x_m = float(
        controlled_frame_x_m
    )

    controlled_frame_y_m = float(
        controlled_frame_y_m
    )

    controlled_frame_yaw_rad = float(
        controlled_frame_yaw_rad
    )

    # --------------------------------------------------
    # Desired controlled-frame position
    # --------------------------------------------------
    #
    # Move backwards from the reference point along the
    # desired controlled-frame heading.
    #
    #       controlled frame -----> reference
    #                    standoff
    #

    controlled_goal_x_m = (
            reference_x_m
            - standoff_m
            * math.cos(controlled_heading_rad)
    )

    controlled_goal_y_m = (
            reference_y_m
            - standoff_m
            * math.sin(controlled_heading_rad)
    )

    # --------------------------------------------------
    # Desired base_link heading
    # --------------------------------------------------

    goal_base_heading_rad = _wrap_angle_rad(
        controlled_heading_rad
        - controlled_frame_yaw_rad
    )

    # --------------------------------------------------
    # Controlled-frame mount offset at desired heading
    # --------------------------------------------------

    (
        controlled_offset_x_m,
        controlled_offset_y_m,
    ) = rotate_vector_2d(
        x=controlled_frame_x_m,
        y=controlled_frame_y_m,
        angle_rad=goal_base_heading_rad,
    )

    # --------------------------------------------------
    # Desired base_link position
    # --------------------------------------------------

    goal_base_x_m = (
            controlled_goal_x_m
            - controlled_offset_x_m
    )

    goal_base_y_m = (
            controlled_goal_y_m
            - controlled_offset_y_m
    )

    return (
        goal_base_x_m,
        goal_base_y_m,
        goal_base_heading_rad,
    )


__all__ = [
    "relative_goal_pose_from_reference",
]