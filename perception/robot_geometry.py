# perception/robot_geometry.py

from __future__ import annotations

import math


"""
Target geometry transforms.

Coordinate conventions
----------------------

Robot geometry:
    +x = forward
    +y = left
    +yaw = counter-clockwise / left

Control bearing:
    +bearing = right
    -bearing = left

Vision observations are expected to be CAMERA-RELATIVE and to contain
the logical source camera name.

Supported observation shapes:

Arena detection:
    {
        "distance_mm": ...,
        "bearing_deg": ...,
        "camera": "front",
    }

Perception object:
    {
        "distance": ...,
        "bearing": ...,
        "camera": "front",
    }

Public functions return:

    (distance_mm, bearing_deg)

relative to the requested frame.
"""


# ==================================================
# Observation extraction
# ==================================================

def _camera_name(observation: dict) -> str:
    """
    Return the logical camera which produced the observation.

    Camera identity is mandatory because camera mounting geometry
    may differ between cameras.
    """

    camera_name = observation.get("camera")

    if not camera_name:
        raise ValueError(
            "geometry observation has no source camera"
        )

    return str(camera_name)


def _camera_measurement(
    observation: dict,
) -> tuple[float, float]:
    """
    Extract camera-relative distance and bearing from either
    supported perception observation format.
    """

    if "distance_mm" in observation:
        distance_mm = float(
            observation["distance_mm"]
        )

    elif "distance" in observation:
        distance_mm = float(
            observation["distance"]
        )

    else:
        raise ValueError(
            "geometry observation has no distance"
        )

    if "bearing_deg" in observation:
        bearing_deg = float(
            observation["bearing_deg"]
        )

    elif "bearing" in observation:
        bearing_deg = float(
            observation["bearing"]
        )

    else:
        raise ValueError(
            "geometry observation has no bearing"
        )

    return distance_mm, bearing_deg


# ==================================================
# Polar / Cartesian conversion
# ==================================================

def _polar_to_xy(
    distance_mm: float,
    bearing_deg: float,
) -> tuple[float, float]:
    """
    Convert control bearing into robot-style XY geometry.

    Control:
        +bearing = right

    Geometry:
        +x = forward
        +y = left
    """

    bearing_rad = math.radians(
        float(bearing_deg)
    )

    x_mm = (
        float(distance_mm)
        * math.cos(bearing_rad)
    )

    y_mm = (
        -float(distance_mm)
        * math.sin(bearing_rad)
    )

    return x_mm, y_mm


def _xy_to_polar(
    x_mm: float,
    y_mm: float,
) -> tuple[float, float]:
    """
    Convert robot-style XY geometry back into control
    distance/bearing.
    """

    distance_mm = math.hypot(
        x_mm,
        y_mm,
    )

    bearing_deg = -math.degrees(
        math.atan2(
            y_mm,
            x_mm,
        )
    )

    return distance_mm, bearing_deg


# ==================================================
# Camera -> base_link
# ==================================================

def _target_xy_base_link(
    *,
    observation: dict,
    config,
) -> tuple[float, float]:
    """
    Convert a camera-relative observation into target XY
    relative to base_link.
    """

    camera_name = _camera_name(
        observation
    )

    distance_mm, bearing_deg = (
        _camera_measurement(
            observation
        )
    )

    target_x_cam, target_y_cam = (
        _polar_to_xy(
            distance_mm,
            bearing_deg,
        )
    )

    try:
        mount = config.camera_mounts[
            camera_name
        ]

    except KeyError as exc:
        raise ValueError(
            f"no camera mount configured "
            f"for {camera_name!r}"
        ) from exc

    mount_x_mm = float(
        mount["x_mm"]
    )

    mount_y_mm = float(
        mount["y_mm"]
    )

    mount_yaw_rad = math.radians(
        float(
            mount.get(
                "yaw_deg",
                0.0,
            )
        )
    )

    cos_yaw = math.cos(
        mount_yaw_rad
    )

    sin_yaw = math.sin(
        mount_yaw_rad
    )

    # Camera coordinates -> base_link orientation.
    rotated_x_mm = (
        cos_yaw * target_x_cam
        - sin_yaw * target_y_cam
    )

    rotated_y_mm = (
        sin_yaw * target_x_cam
        + cos_yaw * target_y_cam
    )

    # Camera origin -> base_link origin.
    target_x_base_mm = (
        mount_x_mm
        + rotated_x_mm
    )

    target_y_base_mm = (
        mount_y_mm
        + rotated_y_mm
    )

    return (
        target_x_base_mm,
        target_y_base_mm,
    )

def _wrap_angle_rad(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi]."""

    return math.atan2(
        math.sin(angle_rad),
        math.cos(angle_rad),
    )


# ==================================================
# Public geometry API
# ==================================================

def target_from_camera(
    *,
    observation: dict,
) -> tuple[float, float]:
    """
    Return target distance/bearing relative to the camera
    which produced the observation.

    No robot transform is applied.
    """

    return _camera_measurement(
        observation
    )


def target_xy_from_base_link(
    *,
    observation: dict,
    config,
) -> tuple[float, float]:
    """
    Return target XY position relative to robot base_link.

    Coordinate convention:
        +x = forward
        +y = left

    Units:
        millimetres
    """

    return _target_xy_base_link(
        observation=observation,
        config=config,
    )

def target_from_base_link(
    *,
    observation: dict,
    config,
) -> tuple[float, float]:
    """
    Return target distance/bearing relative to robot base_link.

    base_link is the canonical robot reference frame.
    """

    x_mm, y_mm = _target_xy_base_link(
        observation=observation,
        config=config,
    )

    return _xy_to_polar(
        x_mm,
        y_mm,
    )


def target_from_gripper(
    *,
    observation: dict,
    config,
) -> tuple[float, float]:
    """
    Return target distance/bearing relative to the gripper.

    Transformation path:

        source camera
            ->
        base_link
            ->
        gripper
    """

    target_x_base_mm, target_y_base_mm = (
        _target_xy_base_link(
            observation=observation,
            config=config,
        )
    )

    grip = config.gripper_mount

    grip_x_mm = float(
        grip["x_mm"]
    )

    grip_y_mm = float(
        grip["y_mm"]
    )

    grip_yaw_rad = math.radians(
        float(
            grip.get(
                "yaw_deg",
                0.0,
            )
        )
    )

    # Move origin from base_link to gripper.
    delta_x_mm = (
        target_x_base_mm
        - grip_x_mm
    )

    delta_y_mm = (
        target_y_base_mm
        - grip_y_mm
    )

    # Rotate base_link axes into gripper axes.
    cos_yaw = math.cos(
        grip_yaw_rad
    )

    sin_yaw = math.sin(
        grip_yaw_rad
    )

    target_x_grip_mm = (
        cos_yaw * delta_x_mm
        + sin_yaw * delta_y_mm
    )

    target_y_grip_mm = (
        -sin_yaw * delta_x_mm
        + cos_yaw * delta_y_mm
    )

    return _xy_to_polar(
        target_x_grip_mm,
        target_y_grip_mm,
    )

def gripper_standoff_pose_from_face(
    *,
    observation: dict,
    standoff_mm: float,
    config,
) -> tuple[float, float, float]:
    """
    Return the desired robot base_link pose for a gripper standoff
    from one observed object face.

    This is the geometry bridge between pose-capable perception and
    a generic relative-pose controller such as SmoothControlLaw.

    The observation must describe ONE face and therefore its distance,
    bearing and yaw must all come from the same face observation.

    Returns:

        (goal_x_mm, goal_y_mm, goal_heading_rad)

    relative to the robot's CURRENT base_link frame.

    The returned pose is the desired base_link pose which would place
    the gripper:

        - standoff_mm directly in front of the observed face
        - aligned square to that face

    Coordinate conventions:

        +x       = forward
        +y       = left
        +heading = counter-clockwise / left

    The observed face yaw is camera-relative. Experimental Webots
    validation established that the desired approach heading relative
    to an aligned camera is -face_yaw.

    Camera and gripper mount translation/yaw are included here so the
    generic pose controller does not need to know anything about
    cameras, grippers or AprilTags.

    This function performs geometry only. It does NOT select a face,
    assess whether a face is usable, apply controller limits, or decide
    when pose servoing should hand over to final approach/docking.
    """

    if "yaw_deg" not in observation:
        raise ValueError(
            "face observation has no yaw_deg"
        )

    camera_name = _camera_name(
        observation
    )

    marker_x_base_mm, marker_y_base_mm = (
        _target_xy_base_link(
            observation=observation,
            config=config,
        )
    )

    try:
        camera_mount = config.camera_mounts[
            camera_name
        ]

    except KeyError as exc:
        raise ValueError(
            f"no camera mount configured "
            f"for {camera_name!r}"
        ) from exc

    camera_yaw_rad = math.radians(
        float(
            camera_mount.get(
                "yaw_deg",
                0.0,
            )
        )
    )

    face_yaw_rad = math.radians(
        float(
            observation["yaw_deg"]
        )
    )

    # Heading the gripper must have in the current base_link frame
    # in order to approach square to the observed face.
    desired_gripper_heading_rad = _wrap_angle_rad(
        camera_yaw_rad
        - face_yaw_rad
    )

    # Desired position of the gripper origin: standoff_mm directly
    # in front of the observed face.
    gripper_goal_x_mm = (
        marker_x_base_mm
        - float(standoff_mm)
        * math.cos(
            desired_gripper_heading_rad
        )
    )

    gripper_goal_y_mm = (
        marker_y_base_mm
        - float(standoff_mm)
        * math.sin(
            desired_gripper_heading_rad
        )
    )

    grip = config.gripper_mount

    grip_x_mm = float(
        grip["x_mm"]
    )

    grip_y_mm = float(
        grip["y_mm"]
    )

    grip_yaw_rad = math.radians(
        float(
            grip.get(
                "yaw_deg",
                0.0,
            )
        )
    )

    # base_link heading required for the mounted gripper itself
    # to have desired_gripper_heading_rad.
    goal_heading_rad = _wrap_angle_rad(
        desired_gripper_heading_rad
        - grip_yaw_rad
    )

    cos_heading = math.cos(
        goal_heading_rad
    )

    sin_heading = math.sin(
        goal_heading_rad
    )

    # At the desired robot heading, rotate the fixed base_link ->
    # gripper translation into the CURRENT base_link frame.
    gripper_offset_x_mm = (
        cos_heading * grip_x_mm
        - sin_heading * grip_y_mm
    )

    gripper_offset_y_mm = (
        sin_heading * grip_x_mm
        + cos_heading * grip_y_mm
    )

    # Move from the desired gripper pose back to the desired
    # base_link pose.
    goal_x_mm = (
        gripper_goal_x_mm
        - gripper_offset_x_mm
    )

    goal_y_mm = (
        gripper_goal_y_mm
        - gripper_offset_y_mm
    )

    return (
        goal_x_mm,
        goal_y_mm,
        goal_heading_rad,
    )


__all__ = [
    "target_from_camera",
    "target_xy_from_base_link",
    "target_from_base_link",
    "target_from_gripper",
    "gripper_standoff_pose_from_face",
]