# tools/challenges/servoing_return_to_base.py

from __future__ import annotations

import math
import time

from behaviors.init_escape import InitEscape
from config.arena import return_guide_routes
from navigation.wall_geometry.models import WallGeometry
from navigation.wall_following.models import WallSide
from perception import sense
from perception.robot_geometry import (
    target_from_base_link,
    target_from_camera,
)
from primitives.base import PrimitiveStatus
from primitives.manipulation import LiftDown
from skills.navigation.follow_wall import FollowWall
from skills.navigation.return_to_base_servo import (
    FINAL_GUIDE_DIRECT_APPROACH_MAX_DEG,
    FINAL_GUIDE_STOP_DISTANCE_MM,
    ReturnToBaseServo,
)


# ==================================================
# Challenge configuration
# ==================================================

TARGET_ZONE = 1

# For this test we specifically want the increasing-ID route:
#
#     ... 0 -> 1 -> 2 -> 3 -> 4
#
# so the robot travels along the top wall toward zone 1.
TARGET_ROUTE_INDEX = 0

# Top wall is on the robot's LEFT while travelling toward zone 1.
WALL_SIDE = WallSide.LEFT
WALL_SENSOR_KEY = WALL_SIDE.value

# A side-on final-guide encounter may hand off to wall following,
# but only when the robot is already close enough to the wall and
# sufficiently parallel to it.
#
# This 300 mm value is an ENTRY reference, not the wall-follow
# setpoint. The actual ultrasonic distance is still latched at handoff.
WALL_FOLLOW_ENTRY_DISTANCE_MM = 300.0
WALL_FOLLOW_ENTRY_DISTANCE_TOLERANCE_MM = 150.0
WALL_FOLLOW_ENTRY_PARALLEL_TOLERANCE_DEG = 15.0
FINAL_GUIDE_EDGE_MARGIN_DEG = 3.0

WALL_FOLLOW_LINEAR_X_MPS = 0.25

LOOP_DELAY_S = 0.02
DIAGNOSTIC_LOG_PERIOD_S = 0.25

# Challenge-only emergency guards.
FRONT_STOP_MM = 250.0
MIN_SIDE_CLEARANCE_MM = 80.0


def _read_range_mm(io, key: str) -> float | None:
    try:
        value = io.ultrasonic[key]
    except Exception:
        return None

    if value is None:
        return None

    try:
        value = float(value)
    except (TypeError, ValueError):
        return None

    # In simulation, 0 means no return.
    if value <= 0.0:
        return None

    return value


def _update_localisation(
    controller,
    vision_message,
    now_s: float,
):
    """
    Keep challenge localisation current using the same vision path as
    normal controller operation.
    """

    pose_obs = controller.localisation.estimate(
        vision_message=vision_message,
        now_s=now_s,
    )

    if pose_obs is not None:
        controller.localisation.accept(pose_obs)

    return controller.localisation.pose


def _top_wall_distance_from_base_link_mm(
    controller,
) -> float | None:
    """
    Perpendicular distance from base_link to the arena top wall.

    For this challenge the target wall is known:
        y = +arena_size / 2

    Localisation is being corrected from the arena-tag observations,
    so this gives the tag-derived base_link-to-wall distance.
    """

    pose = controller.localisation.pose

    if pose is None:
        return None

    if not pose.position_valid:
        return None

    top_wall_y_mm = (
        float(controller.config.arena_size)
        / 2.0
    )

    return (
        top_wall_y_mm
        - float(pose.y)
    )


def _left_side_clearance_mm(
    controller,
) -> float | None:
    """
    Estimate top-wall clearance from the robot's LEFT SIDE.

    The current config has drive-track width but no separate body-width /
    footprint-width field, so for this simulation challenge we use
    half the drive track as the lateral base_link-to-side approximation.

    This value is ONLY the handoff trigger.

    The wall-follow setpoint itself is the actual ultrasonic.left
    reading latched at the instant of handoff.
    """

    base_wall_mm = (
        _top_wall_distance_from_base_link_mm(
            controller
        )
    )

    if base_wall_mm is None:
        return None

    half_track_mm = (
        float(
            controller.config.drive_track_width_mm
        )
        / 2.0
    )

    return (
        base_wall_mm
        - half_track_mm
    )

def _top_wall_parallel_error_deg(
    controller,
) -> float | None:
    """
    Absolute robot-heading error from being parallel to the top wall.

    0 deg:
        robot is parallel to the wall in either direction.

    90 deg:
        robot is perpendicular to the wall.
    """

    pose = controller.localisation.pose

    if pose is None:
        return None

    if not pose.heading_valid:
        return None

    heading_rad = float(pose.heading)

    # Wall direction is an axis rather than a directed vector:
    # heading 0 deg and 180 deg are both parallel.
    parallel_error_rad = abs(
        (
            (
                heading_rad
                + math.pi / 2.0
            )
            % math.pi
        )
        - math.pi / 2.0
    )

    return math.degrees(
        parallel_error_rad
    )

def _guide_geometry(
    *,
    guide_id: int,
    arena_observations,
    config,
) -> tuple[float | None, float | None]:
    """
    Return base_link-referenced distance and bearing for one arena guide.
    """

    observation = next(
        (
            obs
            for obs in arena_observations
            if int(obs.get("id", -1))
            == int(guide_id)
        ),
        None,
    )

    if observation is None:
        return None, None

    distance_mm, bearing_deg = (
        target_from_base_link(
            observation=observation,
            config=config,
        )
    )

    return (
        float(distance_mm),
        float(bearing_deg),
    )


def _guide_camera_bearing_deg(
    *,
    guide_id: int,
    arena_observations,
) -> float | None:
    """
    Return camera-relative bearing for one visible arena guide.
    """

    observation = next(
        (
            obs
            for obs in arena_observations
            if int(obs.get("id", -1))
            == int(guide_id)
        ),
        None,
    )

    if observation is None:
        return None

    _, bearing_deg = target_from_camera(
        observation=observation,
    )

    return float(bearing_deg)



def run(controller):
    """
    Return-to-base -> wall-follow integration challenge.

    Test path:

        lower lift
            ↓
        normal InitEscape
            ↓
        ReturnToBaseServo toward zone 1
        using the increasing-ID top-wall route
            ↓
        arena-tag localisation estimates base_link -> top wall
            ↓
        subtract half track width to estimate
        LEFT SIDE -> top wall clearance
            ↓
        when calculated side clearance <= 300 mm:
            read ultrasonic.left
            latch that exact value
            stop ReturnToBaseServo
            start FollowWall(LEFT)
            ↓
        FollowWall maintains the latched ultrasonic distance
            ↓
        stop when final guide reaches the normal
        ReturnToBase final-guide stop distance

    Important:
        300 mm is only the HANDOFF TRIGGER.

        If ultrasonic.left reads 250 mm at handoff,
        FollowWall maintains 250 mm.
    """

    print(
        "\n=== SERVOING RETURN TO BASE "
        "WALL HANDOFF CHALLENGE ==="
    )

    # --------------------------------------------------
    # Test setup only: lower lift
    # --------------------------------------------------

    print(
        "[RETURN_BASE_CHALLENGE] "
        "lowering lift"
    )

    lift_down = LiftDown(
        settle_time=0.5,
    )

    lift_down.start(
        lvl2=controller.lvl2,
    )

    while (
        lift_down.update()
        == PrimitiveStatus.RUNNING
    ):
        controller.io.sleep(0.05)

    print(
        "[RETURN_BASE_CHALLENGE] "
        "lift down"
    )

    # --------------------------------------------------
    # Test setup only: InitEscape
    # --------------------------------------------------

    print(
        "[RETURN_BASE_CHALLENGE] "
        "running InitEscape"
    )

    escape = InitEscape()

    escape.start(
        config=controller.config,
        motion_backend=controller.motion_backend,
    )

    while True:
        status = escape.update(
            lvl2=controller.lvl2,
            localisation=controller.localisation,
            motion_backend=controller.motion_backend,
        )

        if status.name == "SUCCEEDED":
            break

        if status.name == "FAILED":
            raise RuntimeError(
                "InitEscape failed during "
                "return-base challenge"
            )

        controller.io.sleep(
            LOOP_DELAY_S
        )

    print(
        "[RETURN_BASE_CHALLENGE] "
        "InitEscape complete"
    )

    controller.io.sleep(
        float(
            controller.config.camera_settle_time
        )
    )

    # --------------------------------------------------
    # Force the top-wall route toward zone 1
    # --------------------------------------------------

    all_routes = return_guide_routes(
        TARGET_ZONE
    )

    guide_routes = (
        all_routes[
            TARGET_ROUTE_INDEX
        ],
    )

    final_guide_id = int(
        guide_routes[0]["guide_ids"][-1]
    )

    front_camera_calibration = (
        controller.calibration.cameras.get("front")
    )

    if front_camera_calibration is None:
        raise RuntimeError(
            "No calibration for front camera"
        )

    front_camera_fov_deg = float(
        front_camera_calibration.meta.fov_deg
    )

    final_guide_edge_arm_deg = (
            front_camera_fov_deg / 2.0
            - FINAL_GUIDE_EDGE_MARGIN_DEG
    )

    if final_guide_edge_arm_deg <= 0.0:
        raise RuntimeError(
            "Front camera FOV too small for "
            "final-guide edge margin"
        )

    print(
        "[RETURN_BASE_CHALLENGE][FINAL_EDGE] "
        f"fov={front_camera_fov_deg:.1f}deg "
        f"half={front_camera_fov_deg / 2.0:.1f}deg "
        f"margin={FINAL_GUIDE_EDGE_MARGIN_DEG:.1f}deg "
        f"arm={final_guide_edge_arm_deg:.1f}deg"
    )

    print(
        "[RETURN_BASE_CHALLENGE] "
        f"target_zone={TARGET_ZONE} "
        f"forced_route={TARGET_ROUTE_INDEX} "
        f"route={guide_routes[0]} "
        f"final_guide={final_guide_id}"
    )

    # --------------------------------------------------
    # Start normal ReturnToBaseServo
    # --------------------------------------------------

    return_to_base = ReturnToBaseServo(
        config=controller.config,
        guide_routes=guide_routes,
    )

    return_to_base.start(
        lvl2=controller.lvl2,
    )

    follow_wall = None
    latched_wall_distance_mm = None
    final_guide_edge_armed = False

    next_log_s = time.monotonic()

    try:
        while True:
            now_s = time.time()

            final_approach_deg = None

            # ------------------------------------------
            # Vision
            # ------------------------------------------

            vision_message = (
                controller._get_vision_message(
                    camera_name="front",
                    now_s=now_s,
                )
            )

            # ------------------------------------------
            # Perception
            # ------------------------------------------

            (
                arena_observations,
                _,
            ) = sense(
                controller.io,
                controller.perception,
                latest_vision_message=(
                    vision_message
                ),
                stop_robot=False,
            )

            if vision_message is not None:
                observation_timestamp = float(
                    vision_message[
                        "timestamp"
                    ]
                )
            else:
                observation_timestamp = (
                    now_s
                )

            # ------------------------------------------
            # Keep tag-derived localisation current
            # ------------------------------------------

            _update_localisation(
                controller,
                vision_message,
                now_s,
            )

            # ------------------------------------------
            # Current wall / safety measurements
            # ------------------------------------------

            side_mm = _read_range_mm(
                controller.io,
                WALL_SENSOR_KEY,
            )

            front_mm = _read_range_mm(
                controller.io,
                "front",
            )

            base_wall_mm = (
                _top_wall_distance_from_base_link_mm(
                    controller
                )
            )

            side_clearance_mm = (
                _left_side_clearance_mm(
                    controller
                )
            )

            wall_parallel_error_deg = (
                _top_wall_parallel_error_deg(
                    controller
                )
            )

            (
                final_guide_distance_mm,
                final_guide_bearing_deg,
            ) = _guide_geometry(
                guide_id=final_guide_id,
                arena_observations=(
                    arena_observations
                ),
                config=controller.config,
            )

            final_guide_camera_bearing_deg = (
                _guide_camera_bearing_deg(
                    guide_id=final_guide_id,
                    arena_observations=arena_observations,
                )
            )

            # ------------------------------------------
            # Challenge-only emergency guards
            # ------------------------------------------

            if (
                front_mm is not None
                and front_mm
                <= FRONT_STOP_MM
            ):
                print(
                    "[RETURN_BASE_CHALLENGE]"
                    "[SAFETY] "
                    f"front={front_mm:.0f}mm "
                    "-> stop"
                )
                break

            if (
                side_mm is not None
                and side_mm
                <= MIN_SIDE_CLEARANCE_MM
            ):
                print(
                    "[RETURN_BASE_CHALLENGE]"
                    "[SAFETY] "
                    f"{WALL_SENSOR_KEY}="
                    f"{side_mm:.0f}mm "
                    "-> emergency stop"
                )
                break

            # ==========================================
            # MODE 1: ReturnToBaseServo
            # ==========================================

            if follow_wall is None:

                status = (
                    return_to_base.update(
                        arena_observations=(
                            arena_observations
                        ),
                        observation_timestamp=(
                            observation_timestamp
                        ),
                        robot_pose=(
                            controller.localisation.pose
                        ),
                    )
                )

                final_approach_deg = (
                    return_to_base.final_guide_approach_deg
                )

                side_on_final = (
                        return_to_base.final_guide_visible
                        and final_approach_deg is not None
                        and abs(final_approach_deg)
                        > FINAL_GUIDE_DIRECT_APPROACH_MAX_DEG
                )

                wall_distance_ready = (
                        side_mm is not None
                        and abs(
                    float(side_mm)
                    - WALL_FOLLOW_ENTRY_DISTANCE_MM
                )
                        <= WALL_FOLLOW_ENTRY_DISTANCE_TOLERANCE_MM
                )

                wall_heading_ready = (
                        wall_parallel_error_deg is not None
                        and wall_parallel_error_deg
                        <= WALL_FOLLOW_ENTRY_PARALLEL_TOLERANCE_DEG
                )

                # --------------------------------------
                # Wall-follow handoff
                # --------------------------------------

                if (
                        side_on_final
                        and wall_distance_ready
                        and wall_heading_ready
                ):
                    if side_mm is None:
                        return_to_base.stop()

                        raise RuntimeError(
                            "Wall-follow handoff "
                            "reached but "
                            f"ultrasonic."
                            f"{WALL_SENSOR_KEY} "
                            "has no valid reading"
                        )

                    # Latch ONCE.
                    #
                    # Example:
                    # calculated side clearance = 295 mm
                    # ultrasonic.left = 250 mm
                    #
                    # -> wall follower maintains 250 mm.
                    latched_wall_distance_mm = (
                        float(side_mm)
                    )

                    return_to_base.stop()

                    follow_wall = FollowWall(
                        config=(
                            controller.config
                        ),
                        wall_side=WALL_SIDE,
                        desired_distance_mm=(
                            latched_wall_distance_mm
                        ),
                        linear_x_mps=(
                            WALL_FOLLOW_LINEAR_X_MPS
                        ),
                    )

                    follow_wall.start(
                        lvl2=controller.lvl2,
                    )

                    print(
                        "[RETURN_BASE_CHALLENGE]"
                        "[HANDOFF] "
                        f"final_approach="
                        f"{final_approach_deg:.1f}deg "
                        f"direct_max="
                        f"{FINAL_GUIDE_DIRECT_APPROACH_MAX_DEG:.1f}deg "
                        f"parallel_error="
                        f"{wall_parallel_error_deg:.1f}deg "
                        f"parallel_max="
                        f"{WALL_FOLLOW_ENTRY_PARALLEL_TOLERANCE_DEG:.1f}deg "
                        f"base_wall="
                        f"{base_wall_mm:.0f}mm "
                        f"calculated_left_side="
                        f"{side_clearance_mm:.0f}mm "
                        f"ultrasonic."
                        f"{WALL_SENSOR_KEY}="
                        f"{side_mm:.0f}mm "
                        f"entry_ref="
                        f"{WALL_FOLLOW_ENTRY_DISTANCE_MM:.0f}mm "
                        f"entry_tol=+/-"
                        f"{WALL_FOLLOW_ENTRY_DISTANCE_TOLERANCE_MM:.0f}mm "
                        "-> FOLLOW_WALL "
                        f"desired="
                        f"{latched_wall_distance_mm:.0f}mm"
                    )

                    # First wall-follow update uses the
                    # exact same distance that was latched.
                    #
                    # Therefore the initial distance error
                    # is approximately zero.
                    follow_wall.update(
                        wall=WallGeometry(
                            distance_mm=(
                                side_mm
                            ),
                        ),
                    )

                elif (
                    status
                    == PrimitiveStatus.SUCCEEDED
                ):
                    print(
                        "[RETURN_BASE_CHALLENGE] "
                        "ReturnToBaseServo "
                        "SUCCEEDED before wall handoff"
                    )
                    break

                elif (
                    status
                    == PrimitiveStatus.FAILED
                ):
                    raise RuntimeError(
                        "ReturnToBaseServo FAILED"
                    )

            # ==========================================
            # MODE 2: FollowWall
            # ==========================================

            else:
                if side_mm is None:
                    # FollowWall with no wall geometry
                    # stops rather than driving blind.
                    follow_wall.update(
                        wall=WallGeometry(),
                    )

                else:
                    follow_wall.update(
                        wall=WallGeometry(
                            distance_mm=(
                                side_mm
                            ),
                        ),
                    )

                # --------------------------------------
                # Final-guide camera-edge completion
                # --------------------------------------

                if (
                        not final_guide_edge_armed
                        and final_guide_camera_bearing_deg
                        is not None
                        and abs(final_guide_camera_bearing_deg)
                        >= final_guide_edge_arm_deg
                ):
                    final_guide_edge_armed = True

                    print(
                        "[RETURN_BASE_CHALLENGE]"
                        "[FINAL_EDGE][ARMED] "
                        f"guide={final_guide_id} "
                        f"bearing="
                        f"{final_guide_camera_bearing_deg:+.1f}deg "
                        f"threshold="
                        f"{final_guide_edge_arm_deg:.1f}deg"
                    )

                if (
                        final_guide_edge_armed
                        and final_guide_camera_bearing_deg
                        is None
                ):
                    follow_wall.stop()

                    print(
                        "[RETURN_BASE_CHALLENGE]"
                        "[COMPLETE] "
                        f"final_guide={final_guide_id} "
                        "left camera FOV after edge arm "
                        f"threshold="
                        f"{final_guide_edge_arm_deg:.1f}deg "
                        f"wall_desired="
                        f"{latched_wall_distance_mm:.0f}mm"
                    )

                    break

                # --------------------------------------
                # Finish at normal final-guide distance
                # --------------------------------------

                if (
                    final_guide_distance_mm
                    is not None
                    and final_guide_distance_mm
                    <= (
                        FINAL_GUIDE_STOP_DISTANCE_MM
                    )
                ):
                    follow_wall.stop()

                    print(
                        "[RETURN_BASE_CHALLENGE]"
                        "[COMPLETE] "
                        f"final_guide="
                        f"{final_guide_id} "
                        f"distance="
                        f"{final_guide_distance_mm:.0f}mm "
                        f"target="
                        f"{FINAL_GUIDE_STOP_DISTANCE_MM:.0f}mm "
                        f"wall_desired="
                        f"{latched_wall_distance_mm:.0f}mm"
                    )

                    break

            # ------------------------------------------
            # Diagnostics
            # ------------------------------------------

            monotonic_now = (
                time.monotonic()
            )

            if (
                monotonic_now
                >= next_log_s
            ):
                if follow_wall is None:
                    mode = "RETURN_SERVO"
                    wall_mode = "NONE"
                    wall_error = None
                    wall_wz = None
                else:
                    mode = "WALL_FOLLOW"

                    active_mode = (
                        follow_wall.active_mode
                    )

                    wall_mode = (
                        "NONE"
                        if active_mode is None
                        else active_mode.value
                    )

                    result = (
                        follow_wall.last_result
                    )

                    if result is None:
                        wall_error = None
                        wall_wz = None
                    else:
                        wall_error = (
                            result.distance_error_mm
                        )
                        wall_wz = (
                            result.command.angular_z_rps
                        )

                if (
                    return_to_base
                    .selected_route_index
                    is None
                ):
                    active_guide_id = None
                else:
                    active_guide_id = (
                        return_to_base
                        .active_guide_id
                    )

                print(
                    "[RETURN_BASE_CHALLENGE]"
                    "[WALL] "
                    f"mode={mode} "
                    f"guide="
                    f"{active_guide_id if active_guide_id is not None else 'NONE'} "
                    f"final={final_guide_id} "
                    f"final_dist="
                    f"{'NONE' if final_guide_distance_mm is None else f'{final_guide_distance_mm:.0f}mm'} "
                    f"final_bearing="
                    f"final_approach="
                    f"{'NONE' if final_approach_deg is None else f'{final_approach_deg:.1f}deg'} "
                    f"parallel_error="
                    f"{'NONE' if wall_parallel_error_deg is None else f'{wall_parallel_error_deg:.1f}deg'} "
                    f"{'NONE' if final_guide_bearing_deg is None else f'{final_guide_bearing_deg:+.1f}deg'} "
                    f"base_wall="
                    f"{'NONE' if base_wall_mm is None else f'{base_wall_mm:.0f}mm'} "
                    f"calc_left_side="
                    f"{'NONE' if side_clearance_mm is None else f'{side_clearance_mm:.0f}mm'} "
                    f"{WALL_SENSOR_KEY}="
                    f"{'NONE' if side_mm is None else f'{side_mm:.0f}mm'} "
                    f"desired="
                    f"{'NONE' if latched_wall_distance_mm is None else f'{latched_wall_distance_mm:.0f}mm'} "
                    f"wall_error="
                    f"{'NONE' if wall_error is None else f'{wall_error:+.0f}mm'} "
                    f"wall_wz="
                    f"{'NONE' if wall_wz is None else f'{wall_wz:+.3f}'} "
                    f"wall_mode={wall_mode} "
                    f"front="
                    f"{'NONE' if front_mm is None else f'{front_mm:.0f}mm'}"
                )

                next_log_s = (
                    monotonic_now
                    + DIAGNOSTIC_LOG_PERIOD_S
                )

            controller.io.sleep(
                LOOP_DELAY_S
            )

    finally:
        return_to_base.stop()

        if follow_wall is not None:
            follow_wall.stop()

    print(
        "=== SERVOING RETURN TO BASE "
        "WALL HANDOFF CHALLENGE "
        "COMPLETE ===\n"
    )
