# perception/pose_reference.py

from __future__ import annotations

import math


"""
Generic construction of a desired relative robot pose.

All geometry supplied to this module is already expressed in the
robot's CURRENT base_link frame.

Coordinate convention:
    +x       = forward
    +y       = left
    +heading = counter-clockwise / left

Units:
    position = millimetres
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


def base_pose_from_reference(
    *,
    reference_x_mm: float,
    reference_y_mm: float,
    controlled_heading_rad: float,
    standoff_mm: float,
    controlled_frame_x_mm: float = 0.0,
    controlled_frame_y_mm: float = 0.0,
    controlled_frame_yaw_rad: float = 0.0,
) -> tuple[float, float, float]:
    """
    Construct a desired base_link pose from a geometric reference.

    The reference point defines WHERE the controlled frame is aiming.

    controlled_heading_rad defines the desired heading of the
    controlled frame in the CURRENT base_link coordinate system.

    A positive standoff places the controlled-frame origin behind the
    reference point along the negative controlled-heading direction.

    controlled_frame_x_mm / y_mm / yaw_rad describe the fixed pose of
    the controlled frame relative to base_link.

    For example, the controlled frame could be:
        - base_link itself
        - a gripper
        - a docking point
        - another tool or mechanism

    Returns:
        (
            goal_base_x_mm,
            goal_base_y_mm,
            goal_base_heading_rad,
        )

    relative to the robot's CURRENT base_link frame.
    """

    values = (
        reference_x_mm,
        reference_y_mm,
        controlled_heading_rad,
        standoff_mm,
        controlled_frame_x_mm,
        controlled_frame_y_mm,
        controlled_frame_yaw_rad,
    )

    if not all(
        math.isfinite(float(value))
        for value in values
    ):
        raise ValueError(
            "pose reference inputs must all be finite"
        )

    reference_x_mm = float(reference_x_mm)
    reference_y_mm = float(reference_y_mm)
    controlled_heading_rad = _wrap_angle_rad(
        float(controlled_heading_rad)
    )
    standoff_mm = float(standoff_mm)

    controlled_frame_x_mm = float(
        controlled_frame_x_mm
    )
    controlled_frame_y_mm = float(
        controlled_frame_y_mm
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

    controlled_goal_x_mm = (
        reference_x_mm
        - standoff_mm
        * math.cos(controlled_heading_rad)
    )

    controlled_goal_y_mm = (
        reference_y_mm
        - standoff_mm
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

    cos_heading = math.cos(
        goal_base_heading_rad
    )

    sin_heading = math.sin(
        goal_base_heading_rad
    )

    controlled_offset_x_mm = (
        cos_heading * controlled_frame_x_mm
        - sin_heading * controlled_frame_y_mm
    )

    controlled_offset_y_mm = (
        sin_heading * controlled_frame_x_mm
        + cos_heading * controlled_frame_y_mm
    )

    # --------------------------------------------------
    # Desired base_link position
    # --------------------------------------------------

    goal_base_x_mm = (
        controlled_goal_x_mm
        - controlled_offset_x_mm
    )

    goal_base_y_mm = (
        controlled_goal_y_mm
        - controlled_offset_y_mm
    )

    return (
        goal_base_x_mm,
        goal_base_y_mm,
        goal_base_heading_rad,
    )


__all__ = [
    "base_pose_from_reference",
]