# autonomous/SR2026/return_to_base.py

from __future__ import annotations

import math
import time

from autonomous.SR2026.base import Behavior, BehaviorStatus
from calibration import CALIBRATION
from config.arena import marker_poses, return_guide_routes
from navigation.wall_following.models import WallSide
from navigation.wall_geometry.models import WallGeometry
from primitives.base import PrimitiveStatus
from primitives.motion import Drive, Rotate
from skills.navigation.follow_wall import FollowWall
from skills.navigation.search_for_return_guide import (
    SearchForReturnGuide,
)
from skills.navigation.return_to_base_servo import (
    FINAL_GUIDE_DIRECT_APPROACH_MAX_DEG,
    ReturnToBaseServo,
)


# ==================================================
# Return-to-base policy
# ==================================================

STAGE1_RETURN_DISTANCE_MM = 750.0

# A side-on final-guide encounter may hand off from
# ReturnToBaseServo to FollowWall only when the robot is already
# close enough to the wall and sufficiently parallel to it.
#
# These are ENTRY limits, not the wall-follow setpoint. The actual
# side range is latched when FollowWall starts.
WALL_FOLLOW_ENTRY_MIN_DISTANCE_MM = 150.0
WALL_FOLLOW_ENTRY_MAX_DISTANCE_MM = 450.0
WALL_FOLLOW_ENTRY_PARALLEL_TOLERANCE_DEG = 15.0

WALL_FOLLOW_LINEAR_X_MPS = 0.25

# While FollowWall owns the drivetrain, reaching the camera edge
# arms terminal completion. The wall-follow path succeeds only when
# the final guide subsequently leaves the camera FOV.
FINAL_GUIDE_EDGE_MARGIN_DEG = 3.0
FINAL_GUIDE_CAMERA_NAME = "front"

# Wall-follow terminal success is followed by one normal Drive
# primitive. How that distance is executed is owned by the selected
# motion backend.
FINAL_WALL_FOLLOW_DRIVE_MM = 500.0

# Intermediate-guide handoff recovery.
#
# When the robot is already inside the handoff guard and the next
# guide is not visible, only make a small route-forward look. If the
# next guide is still unavailable, back away far enough to leave the
# guard and let a fresh ReturnToBaseServo select whichever route guide
# is actually visible.
GUIDE_HANDOFF_PEEK_DEG = 10.0
GUIDE_HANDOFF_BACKOFF_MM = 500.0


MODE_STAGE1 = "stage1"
MODE_SERVO = "servo"
MODE_GUIDE_SEARCH = "guide_search"
MODE_GUIDE_HANDOFF_PEEK = "guide_handoff_peek"
MODE_GUIDE_BACKOFF = "guide_backoff"
MODE_GUIDE_REALIGN = "guide_realign"
MODE_WALL_FOLLOW = "wall_follow"
MODE_FINAL_DRIVE = "final_drive"


def _geometry_parallel_error_deg(
    *,
    wall: WallGeometry,
    wall_side: WallSide,
) -> float | None:
    """
    Return wall-parallel error from semantic robot-relative wall geometry.

    WallGeometry.heading_rad is the direction from base_link toward the
    wall normal:

        +90 deg -> wall on left
        -90 deg -> wall on right
    """

    if not wall.has_heading:
        return None

    desired_normal_rad = (
        math.pi / 2.0
        if wall_side == WallSide.LEFT
        else -math.pi / 2.0
    )

    error_rad = math.atan2(
        math.sin(
            float(wall.heading_rad)
            - desired_normal_rad
        ),
        math.cos(
            float(wall.heading_rad)
            - desired_normal_rad
        ),
    )

    return abs(math.degrees(error_rad))


def _wall_parallel_error_deg(
    *,
    robot_pose,
    wall_guide_id: int,
    arena_marker_poses,
) -> float | None:
    """
    Absolute robot-heading error from one guide marker's wall axis.

    The active guide identifies the wall the robot is currently
    travelling along. Arena marker yaw is the inward wall normal, so
    a wall-parallel heading is +/- 90 degrees from that normal.
    """

    if robot_pose is None:
        return None

    if not getattr(robot_pose, "heading_valid", False):
        return None

    marker_pose = arena_marker_poses.get(
        int(wall_guide_id)
    )

    if marker_pose is None:
        return None

    robot_heading_rad = float(robot_pose.heading)
    wall_normal_rad = float(marker_pose["yaw_rad"])
    wall_tangent_rad = wall_normal_rad + math.pi / 2.0

    # Smallest difference to an undirected axis: [0, pi/2].
    parallel_error_rad = abs(
        (
            (
                robot_heading_rad
                - wall_tangent_rad
                + math.pi / 2.0
            )
            % math.pi
        )
        - math.pi / 2.0
    )

    return math.degrees(parallel_error_rad)


def _guides_share_wall(
    *,
    first_guide_id: int,
    second_guide_id: int,
    arena_marker_poses,
) -> bool:
    """Return True when two arena guides are mounted on the same wall."""

    first = arena_marker_poses.get(int(first_guide_id))
    second = arena_marker_poses.get(int(second_guide_id))

    if first is None or second is None:
        return False

    first_yaw = float(first["yaw_rad"])
    second_yaw = float(second["yaw_rad"])

    difference = math.atan2(
        math.sin(first_yaw - second_yaw),
        math.cos(first_yaw - second_yaw),
    )

    return abs(difference) <= 1e-6


class ReturnToBase(Behavior):
    """
    Navigate from the collection area back to the delivery position.

    Stage 2 navigation is an owning state machine around:

        SearchForReturnGuide
              |
              | guide recovered
              v
        ReturnToBaseServo
              |
              | intermediate handoff blocked
              v
        Peek next guide (+/-10 deg)
              |
              | still blocked
              v
        Backoff 500 mm
              |
              v
        Guide realign
              |
              | align to current -> next route segment
              v
        Fresh ReturnToBaseServo

        ReturnToBaseServo
              <->
          FollowWall
              |
              | wall-follow terminal success
              v
          Drive(500 mm)
              |
              v
           SUCCEEDED

    SearchForReturnGuide handles generic return-guide loss. An
    intermediate guide-handoff failure instead gets one small
    route-forward peek; if the expected next guide is still not
    visible, the robot backs away 500 mm, realigns to the known
    current-to-next guide route segment, then starts a fresh route
    selection from the new viewpoint.

    ReturnToBaseServo may also complete directly at the normal final
    guide distance, in which case the behavior succeeds immediately.

    arrival_side is latched from the selected return route and is
    preserved across every navigation handoff for DropoffObject.

    Stage 1 remains the existing dead-reckoning fallback when visual_servoing
    is disabled.

    This behavior does not release the carried object.
    """

    def __init__(self):
        super().__init__()

        self.config = None
        self.match_zone = None
        self.guide_routes = None
        self.arrival_side = None

        self._navigation_stage = None
        self._mode = None
        self._mode_started = False

        self.return_servo = None
        self.return_guide_search = None

        self.guide_handoff_peek = None
        self.guide_handoff_peek_settle_until = None
        self.guide_handoff_current_id = None
        self.guide_handoff_required_id = None
        self.guide_handoff_side = None
        self.guide_backoff = None
        self.guide_realign = None
        self.guide_realign_target_heading_deg = None

        self.follow_wall = None
        self.final_drive = None
        self.stage1_drive = None

        self._arena_marker_poses = None
        self._final_guide_edge_arm_deg = None
        self._final_guide_edge_armed = False
        self._latched_wall_distance_mm = None

    def start(
        self,
        *,
        config,
        match_zone,
        **_,
    ):
        print("[RETURN_TO_BASE] start")

        self.config = config
        self.match_zone = int(match_zone)
        self.guide_routes = return_guide_routes(
            self.match_zone
        )
        self.arrival_side = None

        self._navigation_stage = None
        self._mode = None
        self._mode_started = False

        self.return_servo = None
        self.return_guide_search = None

        self.guide_handoff_peek = None
        self.guide_handoff_peek_settle_until = None
        self.guide_handoff_current_id = None
        self.guide_handoff_required_id = None
        self.guide_handoff_side = None
        self.guide_backoff = None
        self.guide_realign = None
        self.guide_realign_target_heading_deg = None

        self.follow_wall = None
        self.final_drive = None
        self.stage1_drive = None

        self._arena_marker_poses = marker_poses(
            int(self.config.arena_size)
        )

        self._final_guide_edge_arm_deg = None
        self._final_guide_edge_armed = False
        self._latched_wall_distance_mm = None

        self.status = BehaviorStatus.RUNNING

        print(
            "[RETURN_TO_BASE] "
            f"zone={self.match_zone} "
            f"routes={self.guide_routes}"
        )

        if self.config.servoing_enabled:
            self._navigation_stage = 2
            self._mode = MODE_SERVO

            camera_calibration = (
                CALIBRATION.cameras.get(
                    FINAL_GUIDE_CAMERA_NAME
                )
            )

            if camera_calibration is None:
                raise RuntimeError(
                    "No calibration for return-to-base "
                    f"camera {FINAL_GUIDE_CAMERA_NAME!r}"
                )

            camera_fov_deg = float(
                camera_calibration.meta.fov_deg
            )

            self._final_guide_edge_arm_deg = (
                camera_fov_deg / 2.0
                - FINAL_GUIDE_EDGE_MARGIN_DEG
            )

            if self._final_guide_edge_arm_deg <= 0.0:
                raise RuntimeError(
                    "Return-to-base camera FOV is too small "
                    "for the configured final-guide edge margin"
                )

            print(
                "[RETURN_TO_BASE][NAV] "
                "Stage 2 available -> RETURN_TO_BASE_SERVO"
            )

            print(
                "[RETURN_TO_BASE][FINAL_EDGE] "
                f"camera={FINAL_GUIDE_CAMERA_NAME} "
                f"fov={camera_fov_deg:.1f}deg "
                f"margin={FINAL_GUIDE_EDGE_MARGIN_DEG:.1f}deg "
                f"arm={self._final_guide_edge_arm_deg:.1f}deg"
            )

        else:
            self._navigation_stage = 1
            self._mode = MODE_STAGE1

            print(
                "[RETURN_TO_BASE][NAV] "
                "Stage 2 unavailable -> Stage 1 DEAD_RECKONING"
            )

        return self.status

    # --------------------------------------------------
    # Mode starts / transitions
    # --------------------------------------------------

    def _start_return_servo(self, *, lvl2):
        self.return_servo = ReturnToBaseServo(
            config=self.config,
            guide_routes=self.guide_routes,
        )

        self.return_servo.start(
            lvl2=lvl2,
        )

        self._mode = MODE_SERVO
        self._mode_started = True

        print(
            "[RETURN_TO_BASE][MODE] "
            "-> RETURN_TO_BASE_SERVO"
        )

    def _start_return_guide_search(
        self,
        *,
        motion_backend,
        localisation,
        required_guide_id=None,
    ):
        """
        Start Stage 2 perception recovery after initial return-guide
        acquisition fails.

        Version 1 delegates the physical search to SearchForReturnGuide,
        which currently falls back to SearchRotate. Localisation is
        passed through now for future localisation-assisted search.
        """
        if self.return_servo is not None:
            self.return_servo.stop()

        self.return_servo = None

        self.return_guide_search = SearchForReturnGuide(
            config=self.config,
            guide_routes=self.guide_routes,
            required_guide_id=required_guide_id,
        )

        st = self.return_guide_search.start(
            motion_backend=motion_backend,
            localisation=localisation,
        )

        if st == PrimitiveStatus.FAILED:
            print(
                "[RETURN_TO_BASE][RECOVERY] "
                "SearchForReturnGuide failed to start"
            )

            self.return_guide_search = None
            self.status = BehaviorStatus.FAILED
            return self.status

        self._mode = MODE_GUIDE_SEARCH
        self._mode_started = True

        print(
            "[RETURN_TO_BASE][MODE] "
            "RETURN_TO_BASE_SERVO -> SEARCH_FOR_RETURN_GUIDE"
        )

        return self.status

    def _start_guide_handoff_peek(
        self,
        *,
        motion_backend,
        current_guide_id: int,
        required_guide_id: int,
        guide_side: str,
    ):
        """
        Make one small route-forward look for the expected next guide.

        This is deliberately not a broad SearchRotate. At this point
        ReturnToBaseServo has already established the route and is
        inside the intermediate-guide handoff guard.
        """
        if self.return_servo is not None:
            self.return_servo.stop()

        guide_side = str(guide_side).lower()

        if guide_side == "left":
            angle_deg = +GUIDE_HANDOFF_PEEK_DEG
        elif guide_side == "right":
            angle_deg = -GUIDE_HANDOFF_PEEK_DEG
        else:
            print(
                "[RETURN_TO_BASE][RECOVERY] "
                f"invalid guide_side={guide_side!r}"
            )
            self.status = BehaviorStatus.FAILED
            return self.status

        self.guide_handoff_current_id = int(
            current_guide_id
        )
        self.guide_handoff_required_id = int(
            required_guide_id
        )
        self.guide_handoff_side = guide_side
        self.guide_handoff_peek_settle_until = None

        self.guide_handoff_peek = Rotate(
            angle_deg=angle_deg,
        )
        self.guide_handoff_peek.start(
            motion_backend=motion_backend,
        )

        self._mode = MODE_GUIDE_HANDOFF_PEEK
        self._mode_started = True

        print(
            "[RETURN_TO_BASE][RECOVERY] "
            f"guide_handoff current={self.guide_handoff_current_id} "
            f"next={self.guide_handoff_required_id} "
            f"side={guide_side} "
            f"-> peek {angle_deg:+.1f}deg"
        )

        return self.status

    def _start_guide_backoff(
        self,
        *,
        motion_backend,
    ):
        """
        Move out of the handoff guard, then restart Stage 2 from a
        clean route-selection state.
        """
        if self.guide_handoff_peek is not None:
            self.guide_handoff_peek.stop(
                motion_backend=motion_backend,
            )

        self.guide_handoff_peek = None
        self.guide_handoff_peek_settle_until = None

        self.return_servo = None
        self.return_guide_search = None

        self.guide_backoff = Drive(
            distance_mm=-GUIDE_HANDOFF_BACKOFF_MM,
        )
        self.guide_backoff.start(
            motion_backend=motion_backend,
        )

        self._mode = MODE_GUIDE_BACKOFF
        self._mode_started = True

        print(
            "[RETURN_TO_BASE][RECOVERY] "
            f"guide={self.guide_handoff_required_id} "
            "still not visible "
            f"-> backoff {GUIDE_HANDOFF_BACKOFF_MM:.0f}mm"
        )

        return self.status

    def _start_guide_realign(
        self,
        *,
        motion_backend,
        robot_pose,
    ):
        """
        Realign the robot with the known route direction after a
        handoff backoff.

        The desired heading is derived from the fixed arena geometry:

            current guide -> next guide

        The commanded rotation is the shortest angular correction from
        the robot's current localisation heading to that route heading.
        """
        if (
            self.guide_handoff_current_id is None
            or self.guide_handoff_required_id is None
        ):
            print(
                "[RETURN_TO_BASE][RECOVERY] "
                "guide realign missing handoff guide ids"
            )
            self.status = BehaviorStatus.FAILED
            return self.status

        if (
            robot_pose is None
            or not getattr(
                robot_pose,
                "heading_valid",
                False,
            )
        ):
            print(
                "[RETURN_TO_BASE][RECOVERY] "
                "guide realign requires valid heading"
            )
            self.status = BehaviorStatus.FAILED
            return self.status

        current_id = int(
            self.guide_handoff_current_id
        )
        next_id = int(
            self.guide_handoff_required_id
        )

        current_marker = self._arena_marker_poses.get(
            current_id
        )
        next_marker = self._arena_marker_poses.get(
            next_id
        )

        if (
            current_marker is None
            or next_marker is None
        ):
            print(
                "[RETURN_TO_BASE][RECOVERY] "
                f"guide realign missing marker geometry "
                f"current={current_id} next={next_id}"
            )
            self.status = BehaviorStatus.FAILED
            return self.status

        route_dx = (
            float(next_marker["x_m"])
            - float(current_marker["x_m"])
        )
        route_dy = (
            float(next_marker["y_m"])
            - float(current_marker["y_m"])
        )

        if math.hypot(route_dx, route_dy) <= 1e-9:
            print(
                "[RETURN_TO_BASE][RECOVERY] "
                f"guide realign invalid marker geometry "
                f"current={current_id} next={next_id}"
            )
            self.status = BehaviorStatus.FAILED
            return self.status

        target_heading_rad = math.atan2(
            route_dy,
            route_dx,
        )

        heading_error_rad = math.atan2(
            math.sin(
                target_heading_rad
                - float(robot_pose.heading)
            ),
            math.cos(
                target_heading_rad
                - float(robot_pose.heading)
            ),
        )

        angle_deg = math.degrees(
            heading_error_rad
        )

        self.guide_realign_target_heading_deg = (
            math.degrees(target_heading_rad)
        )

        self.guide_realign = Rotate(
            angle_deg=angle_deg,
        )
        self.guide_realign.start(
            motion_backend=motion_backend,
        )

        self._mode = MODE_GUIDE_REALIGN
        self._mode_started = True

        print(
            "[RETURN_TO_BASE][RECOVERY] "
            f"guide realign current={current_id} "
            f"next={next_id} "
            f"heading={math.degrees(float(robot_pose.heading)):+.1f}deg "
            f"target={self.guide_realign_target_heading_deg:+.1f}deg "
            f"rotate={angle_deg:+.1f}deg"
        )

        return self.status

    def _latch_arrival_side(self):
        if self.arrival_side is not None:
            return

        if self.return_servo is None:
            return

        side = self.return_servo.selected_guide_side

        if side is None:
            return

        self.arrival_side = str(side).lower()

        print(
            "[RETURN_TO_BASE][ARRIVAL_SIDE] "
            f"latched={self.arrival_side}"
        )

    def _start_wall_follow(
            self,
            *,
            lvl2,
            wall: WallGeometry,
            final_approach_deg: float,
            parallel_error_deg: float,
    ):
        if self.arrival_side is None:
            raise RuntimeError(
                "Cannot start FollowWall before arrival_side "
                "has been selected"
            )

        wall_side = WallSide(
            self.arrival_side
        )

        if not wall.has_distance:
            raise RuntimeError(
                "Cannot start FollowWall without wall distance"
            )

        self._latched_wall_distance_mm = float(
            wall.distance_mm
        )

        self.return_servo.stop()

        self.follow_wall = FollowWall(
            config=self.config,
            wall_side=wall_side,
            desired_distance_mm=(
                self._latched_wall_distance_mm
            ),
            linear_x_mps=WALL_FOLLOW_LINEAR_X_MPS,
        )

        self.follow_wall.start(
            lvl2=lvl2,
        )

        self._final_guide_edge_armed = False
        self._mode = MODE_WALL_FOLLOW
        self._mode_started = True

        print(
            "[RETURN_TO_BASE][HANDOFF] "
            "RETURN_TO_BASE_SERVO -> FOLLOW_WALL "
            f"side={self.arrival_side} "
            f"final_approach={final_approach_deg:.1f}deg "
            f"direct_max="
            f"{FINAL_GUIDE_DIRECT_APPROACH_MAX_DEG:.1f}deg "
            f"parallel_error={parallel_error_deg:.1f}deg "
            f"wall_distance={wall.distance_mm:.0f}mm "
            f"desired={self._latched_wall_distance_mm:.0f}mm"
        )

        # First wall-follow sample uses the exact range that was
        # latched as the desired distance, so initial distance error
        # is approximately zero.
        self.follow_wall.update(
            wall=wall,
        )

    def _resume_return_servo(self):
        if self.follow_wall is not None:
            self.follow_wall.stop()

        self.follow_wall = None
        self._latched_wall_distance_mm = None
        self._final_guide_edge_armed = False

        self.return_servo.resume()

        self._mode = MODE_SERVO
        self._mode_started = True

        print(
            "[RETURN_TO_BASE][HANDOFF] "
            "FOLLOW_WALL -> RETURN_TO_BASE_SERVO"
        )

    def _start_final_drive(self, *, motion_backend):
        if self.follow_wall is not None:
            self.follow_wall.stop()

        self.follow_wall = None

        print(
            "[RETURN_TO_BASE][WALL_FOLLOW] "
            "terminal success"
        )

        self.final_drive = Drive(
            distance_mm=FINAL_WALL_FOLLOW_DRIVE_MM,
        )

        self.final_drive.start(
            motion_backend=motion_backend,
        )

        self._mode = MODE_FINAL_DRIVE
        self._mode_started = True

        print(
            "[RETURN_TO_BASE][MODE] "
            f"FOLLOW_WALL success -> DRIVE "
            f"{FINAL_WALL_FOLLOW_DRIVE_MM:.0f}mm"
        )

    # --------------------------------------------------
    # Mode updates
    # --------------------------------------------------

    def _update_stage1(
        self,
        *,
        motion_backend,
    ):
        if not self._mode_started:
            self.stage1_drive = Drive(
                distance_mm=STAGE1_RETURN_DISTANCE_MM,
            )

            self.stage1_drive.start(
                motion_backend=motion_backend,
            )

            self._mode_started = True

            print(
                "[RETURN_TO_BASE][NAV] "
                f"starting Stage 1 Drive "
                f"{STAGE1_RETURN_DISTANCE_MM:.0f}mm"
            )

        st = self.stage1_drive.update(
            motion_backend=motion_backend,
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            self.status = BehaviorStatus.FAILED
            return self.status

        self.status = BehaviorStatus.SUCCEEDED

        print(
            "[RETURN_TO_BASE] "
            "Stage 1 complete arrival_side=None"
        )

        return self.status

    def _update_servo(
        self,
        *,
        lvl2,
        motion_backend,
        localisation,
        arena_observations,
        observation_timestamp,
        perception,
        delivered_ids,
        wall_geometry,
        robot_pose,
    ):
        if not self._mode_started:
            self._start_return_servo(
                lvl2=lvl2,
            )

        st = self.return_servo.update(
            arena_observations=arena_observations,
            observation_timestamp=observation_timestamp,
            perception=perception,
            delivered_ids=delivered_ids,
            robot_pose=robot_pose,
        )

        self._latch_arrival_side()

        if st == PrimitiveStatus.SUCCEEDED:
            print(
                "[RETURN_TO_BASE][NAV] "
                "ReturnToBaseServo complete"
            )

            print(
                "[RETURN_TO_BASE] "
                f"arrival_side={self.arrival_side}"
            )

            self.status = BehaviorStatus.SUCCEEDED
            return self.status

        if st == PrimitiveStatus.FAILED:
            failure_reason = getattr(
                self.return_servo,
                "failure_reason",
                None,
            )

            if (
                failure_reason
                == ReturnToBaseServo.FAILURE_GUIDE_HANDOFF
            ):
                required_guide_id = getattr(
                    self.return_servo,
                    "recovery_guide_id",
                    None,
                )
                guide_side = (
                    self.return_servo.selected_guide_side
                )

                if (
                    required_guide_id is None
                    or guide_side is None
                ):
                    print(
                        "[RETURN_TO_BASE][RECOVERY] "
                        "guide_handoff missing recovery context"
                    )

                    self.status = BehaviorStatus.FAILED
                    return self.status

                return self._start_guide_handoff_peek(
                    motion_backend=motion_backend,
                    current_guide_id=(
                        self.return_servo.active_guide_id
                    ),
                    required_guide_id=required_guide_id,
                    guide_side=guide_side,
                )

            if (
                failure_reason
                == ReturnToBaseServo.FAILURE_NO_ROUTE_GUIDE
            ):
                print(
                    "[RETURN_TO_BASE][RECOVERY] "
                    "reason=no_route_guide "
                    "-> SearchForReturnGuide(any)"
                )

                return self._start_return_guide_search(
                    motion_backend=motion_backend,
                    localisation=localisation,
                    required_guide_id=None,
                )

            print(
                "[RETURN_TO_BASE][NAV] "
                "ReturnToBaseServo FAILED "
                f"reason={failure_reason}"
            )

            self.status = BehaviorStatus.FAILED
            return self.status

        # A wall-follow handoff is only possible after the servo has
        # selected a route and therefore established arrival_side.
        if self.arrival_side is None:
            return self.status

        final_state = self.return_servo.observe_final_guide(
            arena_observations=arena_observations,
            robot_pose=robot_pose,
        )

        if (
            final_state is None
            or not final_state.visible
            or final_state.approach_deg is None
        ):
            return self.status

        if (
            final_state.approach_deg
            <= FINAL_GUIDE_DIRECT_APPROACH_MAX_DEG
        ):
            return self.status

        wall_side = WallSide(
            self.arrival_side
        )

        if (
                wall_geometry is None
                or not wall_geometry.has_distance
        ):
            return self.status

        wall_distance_mm = float(
            wall_geometry.distance_mm
        )

        wall_distance_ready = (
                WALL_FOLLOW_ENTRY_MIN_DISTANCE_MM
                <= wall_distance_mm
                <= WALL_FOLLOW_ENTRY_MAX_DISTANCE_MM
        )

        parallel_error_deg = (
            _geometry_parallel_error_deg(
                wall=wall_geometry,
                wall_side=wall_side,
            )
        )

        if parallel_error_deg is None:
            parallel_error_deg = (
                _wall_parallel_error_deg(
                    robot_pose=robot_pose,
                    wall_guide_id=(
                        self.return_servo.active_guide_id
                    ),
                    arena_marker_poses=(
                        self._arena_marker_poses
                    ),
                )
            )

        wall_heading_ready = (
            parallel_error_deg is not None
            and parallel_error_deg
            <= WALL_FOLLOW_ENTRY_PARALLEL_TOLERANCE_DEG
        )

        if not (
            wall_distance_ready
            and wall_heading_ready
        ):
            return self.status

        self._start_wall_follow(
            lvl2=lvl2,
            wall=wall_geometry,
            final_approach_deg=(
                final_state.approach_deg
            ),
            parallel_error_deg=parallel_error_deg,
        )

        return self.status

    def _update_guide_handoff_peek(
        self,
        *,
        lvl2,
        motion_backend,
        arena_observations,
    ):
        if (
            self.guide_handoff_peek is None
            or self.guide_handoff_required_id is None
        ):
            print(
                "[RETURN_TO_BASE][RECOVERY] "
                "handoff-peek mode missing state"
            )
            self.status = BehaviorStatus.FAILED
            return self.status

        st = self.guide_handoff_peek.update(
            motion_backend=motion_backend,
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            print(
                "[RETURN_TO_BASE][RECOVERY] "
                "handoff peek rotate FAILED -> backoff"
            )
            return self._start_guide_backoff(
                motion_backend=motion_backend,
            )

        if self.guide_handoff_peek_settle_until is None:
            settle_s = float(
                getattr(
                    self.config,
                    "recover_settle_time",
                    0.5,
                )
            )

            self.guide_handoff_peek_settle_until = (
                time.monotonic() + settle_s
            )

            print(
                "[RETURN_TO_BASE][RECOVERY] "
                f"handoff peek complete "
                f"-> settle {settle_s:.2f}s"
            )
            return self.status

        if (
            time.monotonic()
            < self.guide_handoff_peek_settle_until
        ):
            return self.status

        required_id = int(
            self.guide_handoff_required_id
        )

        found = any(
            int(obs.get("id", -1)) == required_id
            for obs in (arena_observations or ())
        )

        if found:
            print(
                "[RETURN_TO_BASE][RECOVERY] "
                f"guide={required_id} visible after peek "
                "-> restart ReturnToBaseServo"
            )

            self.guide_handoff_peek = None
            self.guide_handoff_peek_settle_until = None
            self.guide_handoff_current_id = None
            self.guide_handoff_required_id = None
            self.guide_handoff_side = None

            self.return_servo = None
            self._mode_started = False

            self._start_return_servo(
                lvl2=lvl2,
            )

            return self.status

        return self._start_guide_backoff(
            motion_backend=motion_backend,
        )

    def _update_guide_backoff(
        self,
        *,
        motion_backend,
        robot_pose,
    ):
        if self.guide_backoff is None:
            print(
                "[RETURN_TO_BASE][RECOVERY] "
                "guide-backoff mode missing Drive"
            )
            self.status = BehaviorStatus.FAILED
            return self.status

        st = self.guide_backoff.update(
            motion_backend=motion_backend,
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            print(
                "[RETURN_TO_BASE][RECOVERY] "
                "guide backoff FAILED"
            )
            self.status = BehaviorStatus.FAILED
            return self.status

        print(
            "[RETURN_TO_BASE][RECOVERY] "
            f"backoff {GUIDE_HANDOFF_BACKOFF_MM:.0f}mm complete "
            "-> guide realign"
        )

        self.guide_backoff = None

        # The old route is no longer latched for navigation, but retain
        # the current/next guide ids long enough to derive the route
        # direction for the realignment.
        self.arrival_side = None
        self.return_servo = None

        return self._start_guide_realign(
            motion_backend=motion_backend,
            robot_pose=robot_pose,
        )

    def _update_guide_realign(
        self,
        *,
        lvl2,
        motion_backend,
    ):
        if self.guide_realign is None:
            print(
                "[RETURN_TO_BASE][RECOVERY] "
                "guide-realign mode missing Rotate"
            )
            self.status = BehaviorStatus.FAILED
            return self.status

        st = self.guide_realign.update(
            motion_backend=motion_backend,
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            print(
                "[RETURN_TO_BASE][RECOVERY] "
                "guide realign FAILED"
            )
            self.status = BehaviorStatus.FAILED
            return self.status

        print(
            "[RETURN_TO_BASE][RECOVERY] "
            f"guide realign complete "
            f"target={self.guide_realign_target_heading_deg:+.1f}deg "
            "-> reset route selection"
        )

        self.guide_realign = None
        self.guide_realign_target_heading_deg = None

        self.guide_handoff_current_id = None
        self.guide_handoff_required_id = None
        self.guide_handoff_side = None

        self.return_servo = None
        self._mode_started = False

        # Fresh servo instance: whichever valid return guide is now
        # visible may establish the route.
        self._start_return_servo(
            lvl2=lvl2,
        )

        return self.status

    def _update_return_guide_search(
        self,
        *,
        lvl2,
        motion_backend,
        arena_observations,
        localisation,
    ):
        if self.return_guide_search is None:
            print(
                "[RETURN_TO_BASE][RECOVERY] "
                "guide-search mode has no active search"
            )

            self.status = BehaviorStatus.FAILED
            return self.status

        st = self.return_guide_search.update(
            motion_backend=motion_backend,
            arena_observations=arena_observations,
            localisation=localisation,
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            print(
                "[RETURN_TO_BASE][RECOVERY] "
                "return-guide search exhausted -> FAILED"
            )

            self.return_guide_search = None
            self.status = BehaviorStatus.FAILED
            return self.status

        found_guide_id = (
            self.return_guide_search.found_guide_id
        )

        print(
            "[RETURN_TO_BASE][RECOVERY] "
            f"return guide recovered id={found_guide_id} "
            "-> restart ReturnToBaseServo"
        )

        self.return_guide_search = None
        self._mode_started = False

        # Use a fresh servo instance so route selection is performed
        # again from the newly-visible guide.
        self._start_return_servo(
            lvl2=lvl2,
        )

        return self.status

    def _update_wall_follow(
            self,
            *,
            arena_observations,
            wall_geometry,
            robot_pose,
            motion_backend,
    ):
        final_state = self.return_servo.observe_final_guide(
            arena_observations=arena_observations,
            robot_pose=robot_pose,
        )

        # --------------------------------------------------
        # Wall -> direct final servo
        # --------------------------------------------------

        if (
            final_state is not None
            and final_state.visible
            and final_state.approach_deg is not None
            and final_state.approach_deg
            <= FINAL_GUIDE_DIRECT_APPROACH_MAX_DEG
        ):
            print(
                "[RETURN_TO_BASE][FINAL_DECISION] "
                f"guide={final_state.guide_id} "
                f"approach={final_state.approach_deg:.1f}deg "
                f"direct_max="
                f"{FINAL_GUIDE_DIRECT_APPROACH_MAX_DEG:.1f}deg "
                "-> return to direct servo"
            )

            self._resume_return_servo()
            return self.status

        terminal_wall_path = (
            final_state is not None
            and _guides_share_wall(
                first_guide_id=(
                    self.return_servo.active_guide_id
                ),
                second_guide_id=final_state.guide_id,
                arena_marker_poses=(
                    self._arena_marker_poses
                ),
            )
        )

        # --------------------------------------------------
        # Arm expected camera-edge loss
        # --------------------------------------------------
        #
        # This is only a terminal condition when wall following is
        # already taking us along the SAME wall as the final guide.
        # On the opposite route the robot may be following the
        # adjacent wall; there, final-guide visibility is used only
        # to decide when to hand back to ReturnToBaseServo.

        if (
            terminal_wall_path
            and not self._final_guide_edge_armed
            and final_state is not None
            and final_state.visible
            and final_state.camera_bearing_deg is not None
            and abs(final_state.camera_bearing_deg)
            >= self._final_guide_edge_arm_deg
        ):
            self._final_guide_edge_armed = True

            print(
                "[RETURN_TO_BASE][FINAL_EDGE][ARMED] "
                f"guide={final_state.guide_id} "
                f"bearing="
                f"{final_state.camera_bearing_deg:+.1f}deg "
                f"threshold="
                f"{self._final_guide_edge_arm_deg:.1f}deg"
            )

        # --------------------------------------------------
        # Wall-follow terminal success
        # --------------------------------------------------

        if (
            self._final_guide_edge_armed
            and (
                final_state is None
                or not final_state.visible
            )
        ):
            print(
                "[RETURN_TO_BASE][FINAL_EDGE][LOST] "
                "armed final guide left camera FOV"
            )

            self._start_final_drive(
                motion_backend=motion_backend,
            )

            return self.status

        # --------------------------------------------------
        # Continue wall following
        # --------------------------------------------------

        self.follow_wall.update(
            wall=(
                wall_geometry
                if wall_geometry is not None
                else WallGeometry()
            ),
        )

        return self.status

    def _update_final_drive(
        self,
        *,
        motion_backend,
    ):
        st = self.final_drive.update(
            motion_backend=motion_backend,
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            print(
                "[RETURN_TO_BASE][FINAL_DRIVE] FAILED"
            )

            self.status = BehaviorStatus.FAILED
            return self.status

        print(
            "[RETURN_TO_BASE][FINAL_DRIVE] "
            f"complete {FINAL_WALL_FOLLOW_DRIVE_MM:.0f}mm"
        )

        print(
            "[RETURN_TO_BASE] "
            f"arrival_side={self.arrival_side}"
        )

        self.status = BehaviorStatus.SUCCEEDED
        return self.status

    # --------------------------------------------------
    # Public update
    # --------------------------------------------------

    def update(
        self,
        *,
        lvl2,
        motion_backend,
        io,
        localisation,
        wall_geometry: WallGeometry | None = None,
        arena_observations=None,
        observation_timestamp=None,
        perception=None,
        delivered_ids=None,
        **_,
    ):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        if self._navigation_stage == 1:
            return self._update_stage1(
                motion_backend=motion_backend,
            )

        if (
            arena_observations is None
            or observation_timestamp is None
        ):
            print(
                "[RETURN_TO_BASE][NAV] "
                "Stage 2 missing vision input"
            )

            self.status = BehaviorStatus.FAILED
            return self.status

        robot_pose = localisation.pose

        if self._mode == MODE_SERVO:
            return self._update_servo(
                lvl2=lvl2,
                motion_backend=motion_backend,
                localisation=localisation,
                arena_observations=arena_observations,
                observation_timestamp=(
                    observation_timestamp
                ),
                perception=perception,
                delivered_ids=delivered_ids,
                wall_geometry=wall_geometry,
                robot_pose=robot_pose,
            )

        if self._mode == MODE_GUIDE_SEARCH:
            return self._update_return_guide_search(
                lvl2=lvl2,
                motion_backend=motion_backend,
                arena_observations=arena_observations,
                localisation=localisation,
            )

        if self._mode == MODE_GUIDE_HANDOFF_PEEK:
            return self._update_guide_handoff_peek(
                lvl2=lvl2,
                motion_backend=motion_backend,
                arena_observations=arena_observations,
            )

        if self._mode == MODE_GUIDE_BACKOFF:
            return self._update_guide_backoff(
                motion_backend=motion_backend,
                robot_pose=robot_pose,
            )

        if self._mode == MODE_GUIDE_REALIGN:
            return self._update_guide_realign(
                lvl2=lvl2,
                motion_backend=motion_backend,
            )

        if self._mode == MODE_WALL_FOLLOW:
            return self._update_wall_follow(
                arena_observations=arena_observations,
                wall_geometry=wall_geometry,
                robot_pose=robot_pose,
                motion_backend=motion_backend,
            )

        if self._mode == MODE_FINAL_DRIVE:
            return self._update_final_drive(
                motion_backend=motion_backend,
            )

        print(
            "[RETURN_TO_BASE] "
            f"unknown navigation mode={self._mode!r}"
        )

        self.status = BehaviorStatus.FAILED
        return self.status
