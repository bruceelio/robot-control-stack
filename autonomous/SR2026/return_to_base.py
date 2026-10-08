# autonomous/SR2026/return_to_base.py

from __future__ import annotations

import math


from autonomous.SR2026.base import Behavior, BehaviorStatus
from calibration import CALIBRATION
from config.arena import marker_poses, return_guide_routes
from navigation.wall_following.models import WallSide
from navigation.wall_geometry.models import WallGeometry
from perception import get_visible_targets
from perception.height_model import HeightModel
from skills.perception.select_target_stack import SelectTargetStack
from perception.providers.local_obstacle_field import (
    local_obstacle_field_from_objects,
)
from perception.robot_geometry import (
    relative_target_from_base_link,
)
from primitives.base import PrimitiveStatus
from primitives.motion import Drive, Rotate
from skills.navigation.follow_wall import FollowWall
from skills.navigation.local_avoidance import (
    LocalAvoidance,
    LocalObstacleObservation,
)
from skills.navigation.search_for_return_guide import (
    SearchForReturnGuide,
)
from skills.navigation.search_rotate import SearchRotate
from skills.navigation.return_to_base_servo import (
    FINAL_GUIDE_DIRECT_APPROACH_MAX_DEG,
    ReturnToBaseServo,
)
from skills.navigation.approach_target_servo import ApproachTargetServo
from skills.perception.select_target_pickup import ApproachServoMethod
from skills.perception.home_base import HomeBase

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

# Local planning protects the nominal guide-route direction.
#
# The guide servo remains project-agnostic: SR2026 owns the decision
# to interrupt it, the object model used for obstacles, and the
# post-avoidance recovery policy.
RETURN_LOCAL_AVOIDANCE_LOOKAHEAD_MM = 1600.0
SR2026_OBJECT_RADIUS_MM = 65.0


MODE_STAGE1 = "stage1"
MODE_SERVO = "servo"
MODE_LOCAL_AVOIDANCE = "local_avoidance"
MODE_GUIDE_SEARCH = "guide_search"
MODE_GUIDE_HANDOFF_PEEK = "guide_handoff_peek"
MODE_GUIDE_BACKOFF = "guide_backoff"
MODE_GUIDE_REALIGN = "guide_realign"
MODE_WALL_FOLLOW = "wall_follow"
MODE_FINAL_DRIVE = "final_drive"
MODE_TERMINAL = "terminal"
MODE_STACK_APPROACH = "stack_approach"


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
        self.calibration = None
        self.match_zone = None
        self.guide_routes = None
        self.arrival_side = None

        self._navigation_stage = None
        self._mode = None
        self._mode_started = False

        self.return_servo = None
        self.return_guide_search = None
        self.local_avoidance = None
        self.local_avoidance_route_from_id = None
        self.local_avoidance_route_to_id = None
        self._terminal_return_armed = False
        self._terminal_return_from_id = None
        self._terminal_return_final_id = None

        self.home_base = None
        self.home_base_state = None
        self._last_home_base_inside = None

        self.stack_selector = None
        self.stack_approach = None

        self.stack_target_id = None
        self.stack_target_kind = None
        self.stack_target_ignore_ids = ()

        self.stack_distance_mm = None
        self.stack_bearing_deg = None

        self._stack_local_avoidance_active = False

        self.guide_handoff_peek = None

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
            calibration=None,
            **_,
    ):
        print("[RETURN_TO_BASE] start")

        self.config = config
        self.calibration = (
            CALIBRATION
            if calibration is None
            else calibration
        )
        self.match_zone = int(match_zone)
        self.home_base = HomeBase(
            arena_size_mm=float(
                self.config.arena_size
            ),
            entry_margin_mm=float(
                self.config.home_base_entry_margin_mm
            ),
        )

        self.home_base_state = None
        self._last_home_base_inside = None
        self.guide_routes = return_guide_routes(
            self.match_zone
        )
        self.arrival_side = None

        self._navigation_stage = None
        self._mode = None
        self._mode_started = False

        self.return_servo = None
        self.return_guide_search = None
        self.local_avoidance = None
        self.local_avoidance_route_from_id = None
        self.local_avoidance_route_to_id = None

        self._terminal_return_armed = False
        self._terminal_return_from_id = None
        self._terminal_return_final_id = None

        self.stack_selector = SelectTargetStack(
            config=self.config,
            max_age_s=float(
                self.config.visible_max_age_s
            ),
            object_radius_mm=(
                SR2026_OBJECT_RADIUS_MM
            ),
        )

        self.stack_approach = None

        self.stack_target_id = None
        self.stack_target_kind = None
        self.stack_target_ignore_ids = ()

        self.stack_distance_mm = None
        self.stack_bearing_deg = None

        self._stack_local_avoidance_active = False

        self.guide_handoff_peek = None

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
                self.calibration.cameras.get(
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

            self.local_avoidance = LocalAvoidance(
                config=self.config,
                field_fov_rad=math.radians(
                    camera_fov_deg
                ),
                lookahead_distance_mm=(
                    RETURN_LOCAL_AVOIDANCE_LOOKAHEAD_MM
                ),
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

    def _update_home_base_state(
        self,
        *,
        localisation,
    ):
        if (
            self.home_base is None
            or localisation is None
        ):
            return

        state = self.home_base.evaluate(
            localisation=localisation,
            match_zone=self.match_zone,
        )

        self.home_base_state = state

        if (
            state.inside
            == self._last_home_base_inside
        ):
            return

        self._last_home_base_inside = (
            state.inside
        )

        (
            xmin,
            xmax,
            ymin,
            ymax,
        ) = state.safe_bounds_mm

        if state.position_valid:
            print(
                "[HOME_BASE] "
                f"zone={self.match_zone} "
                f"base_link=("
                f"{state.base_link_x_mm:+.0f},"
                f"{state.base_link_y_mm:+.0f})mm "
                f"safe_x=("
                f"{xmin:+.0f},{xmax:+.0f}) "
                f"safe_y=("
                f"{ymin:+.0f},{ymax:+.0f}) "
                f"inside={state.inside} "
                f"source={state.source}"
            )

        else:
            print(
                "[HOME_BASE] "
                f"zone={self.match_zone} "
                "position=UNKNOWN "
                "inside=UNKNOWN"
            )

    # --------------------------------------------------
    # Mode starts / transitions
    # --------------------------------------------------

    def _start_return_servo(
            self,
            *,
            lvl2,
            localisation,
            io,
    ):
        self.return_servo = ReturnToBaseServo(
            config=self.config,
            guide_routes=self.guide_routes,
        )

        self.return_servo.start(
            lvl2=lvl2,
            localisation=localisation,
            io=io,
        )

        self._mode = MODE_SERVO
        self._mode_started = True

        print(
            "[RETURN_TO_BASE][MODE] "
            "-> RETURN_TO_BASE_SERVO"
        )

    def _visible_object_observations(
        self,
        perception,
    ):
        if perception is None:
            return []

        objects = []

        for kind in (
            "acidic",
            "basic",
        ):
            objects.extend(
                get_visible_targets(
                    perception,
                    kind,
                    max_age_s=float(
                        self.config.visible_max_age_s
                    ),
                )
            )

        return objects

    def _local_avoidance_exclude_ids(
        self,
        *,
        carried_target_id,
    ) -> tuple[int, ...]:
        excluded = set()

        if carried_target_id is not None:
            excluded.add(
                int(carried_target_id)
            )

        # The selected LOW stacking base is a navigation target,
        # not an obstacle.
        if self.stack_target_id is not None:
            excluded.add(
                int(self.stack_target_id)
            )

        # HIGH cubes physically sitting on the selected LOW base
        # are deliberately expendable for the SR stacking policy.
        for target_id in self.stack_target_ignore_ids:
            excluded.add(
                int(target_id)
            )

        return tuple(
            sorted(excluded)
        )

    def _local_obstacle_observations(
        self,
        *,
        visible_objects,
        exclude_ids,
    ) -> tuple[
        LocalObstacleObservation,
        ...
    ]:
        excluded = {
            int(target_id)
            for target_id in exclude_ids
        }

        observations = []

        for obj in visible_objects:
            try:
                object_id = int(
                    obj["id"]
                )

                target = (
                    relative_target_from_base_link(
                        observation=obj,
                        config=self.config,
                    )
                )

                distance_mm = (
                    float(
                        target.distance_m
                    )
                    * 1000.0
                )

                bearing_rad = float(
                    target.bearing_rad
                )

            except (
                KeyError,
                TypeError,
                ValueError,
            ):
                continue

            if object_id in excluded:
                continue

            if distance_mm <= 0.0:
                continue

            observations.append(
                LocalObstacleObservation(
                    obstacle_id=object_id,
                    distance_mm=distance_mm,
                    bearing_rad=bearing_rad,
                    radius_mm=(
                        SR2026_OBJECT_RADIUS_MM
                    ),
                )
            )

        return tuple(
            observations
        )

    def _current_return_route_segment(
        self,
    ) -> tuple[int, int] | None:
        """
        Return one ordered guide pair describing the established
        route direction.

        It is latched before LocalAvoidance takes over so that,
        if no guide is visible afterwards, the existing route
        realignment can point the robot back toward the base route.
        """

        if (
            self.return_servo is None
            or self.return_servo.selected_route_index
            is None
            or not self.return_servo.guide_ids
        ):
            return None

        guide_ids = tuple(
            int(guide_id)
            for guide_id
            in self.return_servo.guide_ids
        )

        active_index = int(
            self.return_servo.active_index
        )

        if active_index < len(guide_ids) - 1:
            return (
                guide_ids[active_index],
                guide_ids[active_index + 1],
            )

        if active_index > 0:
            return (
                guide_ids[active_index - 1],
                guide_ids[active_index],
            )

        return None

    def _arm_terminal_return_if_ready(self):
        """
        Latch terminal return once the selected route has progressed
        onto its penultimate -> final guide segment.

        Once armed, losing the final guide after LocalAvoidance must
        not send the robot back into generic route recovery.
        """

        if self._terminal_return_armed:
            return

        if (
            self.return_servo is None
            or self.return_servo.selected_route_index is None
            or not self.return_servo.guide_ids
        ):
            return

        route_segment = (
            self._current_return_route_segment()
        )

        if route_segment is None:
            return

        (
            route_from_id,
            route_to_id,
        ) = route_segment

        final_guide_id = int(
            self.return_servo.final_guide_id
        )

        if int(route_to_id) != final_guide_id:
            return

        self._terminal_return_armed = True
        self._terminal_return_from_id = int(
            route_from_id
        )
        self._terminal_return_final_id = int(
            final_guide_id
        )

        print(
            "[RETURN_TO_BASE][TERMINAL] "
            "armed "
            f"segment="
            f"{self._terminal_return_from_id}->"
            f"{self._terminal_return_final_id}"
        )

    def _try_start_stack_approach(
            self,
            *,
            io,
            perception,
            delivered_ids,
            carried_target_id,
    ) -> bool:
        """
        Prefer an eligible delivered LOW cube once terminal return
        has been established.

        False means no stacking target is currently available and
        normal return-to-base navigation should continue unchanged.
        """

        if not self._terminal_return_armed:
            return False

        if (
                self.stack_selector is None
                or perception is None
                or not delivered_ids
        ):
            return False

        if self.stack_target_id is not None:
            return False

        ignore_ids = ()

        if carried_target_id is not None:
            ignore_ids = (
                int(carried_target_id),
            )

        selection = self.stack_selector.select(
            perception=perception,
            delivered_ids=delivered_ids,
            now_s=float(
                io.time()
            ),
            ignore_ids=ignore_ids,
        )

        if selection is None:
            return False

        self.stack_target_id = int(
            selection.target_id
        )

        self.stack_target_kind = str(
            selection.kind
        )

        self.stack_target_ignore_ids = tuple(
            int(target_id)
            for target_id in getattr(
                selection,
                "ignored_obstacle_ids",
                (),
            )
        )

        self.stack_distance_mm = None
        self.stack_bearing_deg = None
        self.stack_approach = None

        self._stack_local_avoidance_active = False

        if self.return_servo is not None:
            self.return_servo.stop()

        if self.local_avoidance is not None:
            self.local_avoidance.stop()

        if self.follow_wall is not None:
            self.follow_wall.stop()

        self.follow_wall = None

        self._mode = MODE_STACK_APPROACH
        self._mode_started = True

        print(
            "[RETURN_TO_BASE][STACK] "
            f"selected LOW delivered cube "
            f"id={self.stack_target_id} "
            f"kind={self.stack_target_kind} "
            f"distance={selection.distance_mm:.0f}mm "
            f"blockers={selection.blocking_count} "
            f"intrusion="
            f"{selection.total_intrusion_mm:.0f}mm "
            "-> STACK_APPROACH"
        )

        return True

    def _visible_return_guide_ids(
        self,
        *,
        arena_observations,
    ) -> set[int]:
        route_guide_ids = {
            int(guide_id)
            for route in self.guide_routes
            for guide_id in route["guide_ids"]
        }

        return {
            int(obs.get("id", -1))
            for obs in (arena_observations or ())
            if int(obs.get("id", -1))
            in route_guide_ids
        }

    def _start_local_avoidance(
            self,
            *,
            lvl2,
            localisation,
            io,
            now_s: float,
            preferred_bearing_rad: float,
    ):
        if self.local_avoidance is None:
            print(
                "[RETURN_TO_BASE][LOCAL_AVOIDANCE] "
                "planner unavailable"
            )
            return self.status

        route_segment = (
            self._current_return_route_segment()
        )

        if route_segment is None:
            self.local_avoidance_route_from_id = None
            self.local_avoidance_route_to_id = None
        else:
            (
                self.local_avoidance_route_from_id,
                self.local_avoidance_route_to_id,
            ) = route_segment

        if self.return_servo is not None:
            self.return_servo.stop()

        self.local_avoidance.start(
            lvl2=lvl2,
            calibration=self.calibration,
            now_s=now_s,
            localisation=localisation,
            io=io,
        )

        self._mode = MODE_LOCAL_AVOIDANCE
        self._mode_started = True

        print(
            "[RETURN_TO_BASE][LOCAL_AVOIDANCE] "
            "nominal corridor blocked -> takeover "
            f"bearing="
            f"{math.degrees(preferred_bearing_rad):+.1f}deg "
            f"route_segment="
            f"{self.local_avoidance_route_from_id}->"
            f"{self.local_avoidance_route_to_id}"
        )

        return self.status

    def _finish_local_avoidance(
            self,
            *,
            lvl2,
            motion_backend,
            io,
            localisation,
            arena_observations,
            robot_pose,
    ):
        if self.local_avoidance is not None:
            self.local_avoidance.stop()

        visible_guides = (
            self._visible_return_guide_ids(
                arena_observations=(
                    arena_observations
                ),
            )
        )

        # --------------------------------------------------
        # Terminal return
        # --------------------------------------------------
        #
        # Once the penultimate -> final route segment has been
        # established, LocalAvoidance may legitimately carry the
        # robot through delivered-cube clutter close enough to the
        # base that the final guide becomes occluded or leaves the
        # camera FOV.
        #
        # Do not fall back to generic route realignment here.
        # The route has already established that we are in the
        # terminal return region.

        if self._terminal_return_armed:
            final_guide_id = (
                self._terminal_return_final_id
            )

            if (
                final_guide_id is not None
                and final_guide_id in visible_guides
            ):
                print(
                    "[RETURN_TO_BASE][TERMINAL] "
                    f"final guide={final_guide_id} visible "
                    "after avoidance "
                    "-> resume established route"
                )

                self.local_avoidance_route_from_id = None
                self.local_avoidance_route_to_id = None

                self.return_servo.resume()

                self._mode = MODE_SERVO
                self._mode_started = True

                return self.status

            if self._home_base_inside():
                return self._complete_home_base_fallback(
                    reason="final_guide_lost_after_avoidance",
                )

            print(
                "[RETURN_TO_BASE][TERMINAL] "
                "final guide not visible after avoidance "
                f"-> search final guide={final_guide_id}"
            )

            self.local_avoidance_route_from_id = None
            self.local_avoidance_route_to_id = None

            if final_guide_id is not None:
                return self._start_return_guide_search(
                    motion_backend=motion_backend,
                    localisation=localisation,
                    required_guide_id=(
                        int(final_guide_id)
                    ),
                )

            return self._start_terminal_hold(
                reason=(
                    "terminal_final_guide_unknown_after_avoidance"
                ),
            )


        # Preferred handoff:
        #
        #     LocalAvoidance -> fresh ReturnToBaseServo
        #
        # Let the normal guide-route algorithm choose the best
        # currently-visible guide from the new viewpoint.
        if visible_guides:
            print(
                "[RETURN_TO_BASE][LOCAL_AVOIDANCE] "
                f"release -> ReturnToBaseServo "
                f"visible_guides="
                f"{sorted(visible_guides)}"
            )

            self.arrival_side = None
            self.return_servo = None

            self.local_avoidance_route_from_id = None
            self.local_avoidance_route_to_id = None

            self._mode_started = False

            self._start_return_servo(
                lvl2=lvl2,
                localisation=localisation,
                io=io,
            )

            return self.status

        # No useful marker can currently be seen. Only now use route
        # realignment, as discussed.
        route_from_id = (
            self.local_avoidance_route_from_id
        )
        route_to_id = (
            self.local_avoidance_route_to_id
        )

        self.local_avoidance_route_from_id = None
        self.local_avoidance_route_to_id = None

        self.arrival_side = None
        self.return_servo = None

        if (
            route_from_id is not None
            and route_to_id is not None
            and robot_pose is not None
            and getattr(
                robot_pose,
                "heading_valid",
                False,
            )
        ):
            self.guide_handoff_current_id = int(
                route_from_id
            )
            self.guide_handoff_required_id = int(
                route_to_id
            )
            self.guide_handoff_side = None

            print(
                "[RETURN_TO_BASE][LOCAL_AVOIDANCE] "
                "release with no return guide visible "
                "-> guide realign "
                f"{route_from_id}->{route_to_id}"
            )

            return self._start_guide_realign(
                motion_backend=motion_backend,
                robot_pose=robot_pose,
            )

        print(
            "[RETURN_TO_BASE][LOCAL_AVOIDANCE] "
            "release with no return guide visible "
            "and no usable route realign "
            "-> SearchForReturnGuide(any)"
        )

        return self._start_return_guide_search(
            motion_backend=motion_backend,
            localisation=localisation,
            required_guide_id=None,
        )

    def _update_local_avoidance(
        self,
        *,
        lvl2,
        motion_backend,
        io,
        localisation,
        arena_observations,
        perception,
        carried_target_id,
        robot_pose,
    ):

        if (
            self.local_avoidance is None
            or self.return_servo is None
        ):
            print(
                "[RETURN_TO_BASE][LOCAL_AVOIDANCE] "
                "missing active supervisor state"
            )
            self.status = BehaviorStatus.FAILED
            return self.status

        now_s = float(
            io.time()
        )

        # If the guide is still visible, continually correct the
        # preferred route direction. If it is occluded, pass None
        # and LocalAvoidance propagates its latched direction through
        # robot rotation while continuing to propagate blocker tracks.
        preferred_bearing_rad = (
            self.return_servo
            .observe_nominal_travel_bearing(
                arena_observations=(
                    arena_observations
                ),
            )
        )

        visible_objects = (
            self._visible_object_observations(
                perception
            )
        )

        exclude_ids = (
            self._local_avoidance_exclude_ids(
                carried_target_id=(
                    carried_target_id
                ),
            )
        )

        obstacle_observations = (
            self._local_obstacle_observations(
                visible_objects=(
                    visible_objects
                ),
                exclude_ids=exclude_ids,
            )
        )

        obstacle_field = (
            local_obstacle_field_from_objects(
                config=self.config,
                visible_objects=(
                    visible_objects
                ),
                field_fov_rad=(
                    self.local_avoidance
                    .field_fov_rad
                ),
                scan_sectors=(
                    self.local_avoidance
                    .scan_sectors
                ),
                clear_range_mm=(
                    self.local_avoidance
                    .lookahead_distance_mm
                    * 1.5
                ),
                timestamp_s=now_s,
                exclude_ids=exclude_ids,
                default_obstacle_radius_mm=(
                    SR2026_OBJECT_RADIUS_MM
                ),
                max_obstacle_distance_mm=(
                    RETURN_LOCAL_AVOIDANCE_LOOKAHEAD_MM
                ),
            )
        )

        st = self.local_avoidance.update(
            now_s=now_s,
            preferred_bearing_rad=(
                preferred_bearing_rad
            ),
            obstacle_observations=(
                obstacle_observations
            ),
            obstacle_field=(
                obstacle_field
            ),
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            print(
                "[RETURN_TO_BASE][LOCAL_AVOIDANCE] "
                f"FAILED "
                f"reason={self.local_avoidance.reason} "
                "-> route recovery"
            )
        else:
            print(
                "[RETURN_TO_BASE][LOCAL_AVOIDANCE] "
                f"complete "
                f"reason={self.local_avoidance.reason}"
            )

        return self._finish_local_avoidance(
            lvl2=lvl2,
            motion_backend=motion_backend,
            io=io,
            localisation=localisation,
            arena_observations=(
                arena_observations
            ),
            robot_pose=robot_pose,
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
        angular_speed_rad_s = abs(
            float(
                self.config.servoing_angular_max_rad_s
            )
        )

        timeout_s = 0.0

        if angular_speed_rad_s > 0.0:
            timeout_s = (
                    math.radians(
                        abs(angle_deg)
                    )
                    / angular_speed_rad_s
                    + 1.0
            )

        self.guide_handoff_peek = SearchRotate(
            target_angle_deg=angle_deg,
            timeout_s=timeout_s,
            config=self.config,
            label="RETURN_GUIDE_HANDOFF_PEEK",
        )

        st = self.guide_handoff_peek.start(
            motion_backend=motion_backend,
        )

        if st == PrimitiveStatus.FAILED:
            print(
                "[RETURN_TO_BASE][RECOVERY] "
                "handoff peek search failed to start "
                "-> backoff"
            )

            return self._start_guide_backoff(
                motion_backend=motion_backend,
            )

        self._mode = MODE_GUIDE_HANDOFF_PEEK
        self._mode_started = True

        print(
            "[RETURN_TO_BASE][RECOVERY] "
            f"guide_handoff current={self.guide_handoff_current_id} "
            f"next={self.guide_handoff_required_id} "
            f"side={guide_side} "
            f"-> search {angle_deg:+.1f}deg"
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
            localisation,
            io,
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
            localisation=localisation,
            io=io,
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

    def _home_base_inside(self) -> bool:
        return (
            self.home_base_state is not None
            and self.home_base_state.inside is True
        )

    def _complete_home_base_fallback(
        self,
        *,
        reason: str,
    ):
        """
        We cannot improve the return using current perception/navigation,
        but localisation says base_link is safely inside the home base.

        Complete ReturnToBase and let the normal DropoffObject behaviour
        release the carried object.
        """

        if self.local_avoidance is not None:
            self.local_avoidance.stop()

        if self.return_servo is not None:
            self.return_servo.stop()

        if self.follow_wall is not None:
            self.follow_wall.stop()

        if self.stack_approach is not None:
            self.stack_approach.stop()

        self.follow_wall = None
        self.final_drive = None

        print(
            "[RETURN_TO_BASE][HOME_BASE] "
            "inside=True "
            f"reason={reason} "
            "-> ordinary dropoff"
        )

        self.status = BehaviorStatus.SUCCEEDED
        return self.status

    def _start_terminal_hold(
        self,
        *,
        reason: str,
    ):
        if self._home_base_inside():
            return self._complete_home_base_fallback(
                reason=reason,
            )
        if self.local_avoidance is not None:
            self.local_avoidance.stop()

        if self.return_servo is not None:
            self.return_servo.stop()

        if self.follow_wall is not None:
            self.follow_wall.stop()

        self.follow_wall = None
        self.final_drive = None

        self._mode = MODE_TERMINAL
        self._mode_started = True

        print(
            "[RETURN_TO_BASE][TERMINAL] "
            f"reason={reason} "
            "-> HOLD"
        )

        return self.status

    def _start_terminal_drive(
        self,
        *,
        motion_backend,
        reason: str,
    ):
        if self.local_avoidance is not None:
            self.local_avoidance.stop()

        if self.return_servo is not None:
            self.return_servo.stop()

        if self.follow_wall is not None:
            self.follow_wall.stop()

        self.follow_wall = None

        self.final_drive = Drive(
            distance_mm=FINAL_WALL_FOLLOW_DRIVE_MM,
        )

        self.final_drive.start(
            motion_backend=motion_backend,
        )

        self._mode = MODE_FINAL_DRIVE
        self._mode_started = True

        print(
            "[RETURN_TO_BASE][TERMINAL] "
            f"reason={reason} "
            "-> DRIVE "
            f"{FINAL_WALL_FOLLOW_DRIVE_MM:.0f}mm"
        )

        return self.status

    def _start_final_drive(
        self,
        *,
        motion_backend,
    ):

        print(
            "[RETURN_TO_BASE][WALL_FOLLOW] "
            "terminal success"
        )

        return self._start_terminal_drive(
            motion_backend=motion_backend,
            reason="wall_follow_complete",
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
            io,
            localisation,
            arena_observations,
            observation_timestamp,
            perception,
            delivered_ids,
            carried_target_id,
            wall_geometry,
            robot_pose,
    ):
        if not self._mode_started:
            self._start_return_servo(
                lvl2=lvl2,
                localisation=localisation,
                io=io,
            )

        self._arm_terminal_return_if_ready()

        if self._try_start_stack_approach(
                io=io,
                perception=perception,
                delivered_ids=delivered_ids,
                carried_target_id=carried_target_id,
        ):
            return self.status

        # --------------------------------------------------
        # Local-planning supervisor
        # --------------------------------------------------

        if (
            self.local_avoidance is not None
            and self.return_servo is not None
        ):
            nominal_bearing_rad = (
                self.return_servo
                .observe_nominal_travel_bearing(
                    arena_observations=(
                        arena_observations
                    ),
                )
            )



            if nominal_bearing_rad is not None:
                visible_objects = (
                    self._visible_object_observations(
                        perception
                    )
                )

                exclude_ids = (
                    self._local_avoidance_exclude_ids(
                        carried_target_id=(
                            carried_target_id
                        ),
                    )
                )

                obstacle_observations = (
                    self._local_obstacle_observations(
                        visible_objects=(
                            visible_objects
                        ),
                        exclude_ids=exclude_ids,
                    )
                )

                if (
                    self.local_avoidance
                    .nominal_corridor_blocked(
                        preferred_bearing_rad=(
                            nominal_bearing_rad
                        ),
                        lookahead_distance_mm=(
                            RETURN_LOCAL_AVOIDANCE_LOOKAHEAD_MM
                        ),
                        obstacle_observations=(
                            obstacle_observations
                        ),
                    )
                ):
                    return self._start_local_avoidance(
                        lvl2=lvl2,
                        localisation=localisation,
                        io=io,
                        now_s=float(
                            io.time()
                        ),
                        preferred_bearing_rad=(
                            nominal_bearing_rad
                        ),
                    )

        st = self.return_servo.update(
            arena_observations=arena_observations,
            observation_timestamp=observation_timestamp,
            io=io,
            perception=perception,
            delivered_ids=delivered_ids,
            robot_pose=robot_pose,
        )

        self._latch_arrival_side()
        self._arm_terminal_return_if_ready()

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
                if self._home_base_inside():
                    return self._complete_home_base_fallback(
                        reason="no_route_guide",
                    )

                if self._terminal_return_armed:
                    final_guide_id = (
                        self._terminal_return_final_id
                    )

                    if final_guide_id is not None:
                        print(
                            "[RETURN_TO_BASE][TERMINAL] "
                            "final route guide lost "
                            "after terminal arm "
                            f"-> search final guide="
                            f"{final_guide_id}"
                        )

                        return (
                            self._start_return_guide_search(
                                motion_backend=motion_backend,
                                localisation=localisation,
                                required_guide_id=(
                                    int(final_guide_id)
                                ),
                            )
                        )

                    return self._start_terminal_hold(
                        reason=(
                            "terminal_final_guide_unknown"
                        ),
                    )

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
            localisation=localisation,
            io=io,
            final_approach_deg=(
                final_state.approach_deg
            ),
            parallel_error_deg=parallel_error_deg,
        )

        return self.status

    def _update_stack_approach(
            self,
            *,
            lvl2,
            io,
            localisation,
            perception,
            carried_target_id,
    ):
        if (
                self.stack_target_id is None
                or self.stack_target_kind is None
        ):
            print(
                "[RETURN_TO_BASE][STACK] "
                "missing selected target "
                "-> resume normal terminal return"
            )

            if self.return_servo is not None:
                self.return_servo.resume()

                self._mode = MODE_SERVO
                self._mode_started = True

                return self.status

            return self._start_terminal_hold(
                reason="stack_target_missing"
            )

        now_s = float(
            io.time()
        )

        fresh_targets = get_visible_targets(
            perception,
            self.stack_target_kind,
            max_age_s=float(
                self.config.visible_max_age_s
            ),
        )

        target_observation = next(
            (
                target
                for target in fresh_targets
                if int(target.get("id", -1))
                   == self.stack_target_id
            ),
            None,
        )

        goal_distance_mm = None
        goal_bearing_rad = None

        if target_observation is not None:
            target_geometry = (
                relative_target_from_base_link(
                    observation=target_observation,
                    config=self.config,
                )
            )

            goal_distance_mm = (
                    float(
                        target_geometry.distance_m
                    )
                    * 1000.0
            )

            goal_bearing_rad = float(
                target_geometry.bearing_rad
            )

        visible_objects = (
            self._visible_object_observations(
                perception
            )
        )

        exclude_ids = (
            self._local_avoidance_exclude_ids(
                carried_target_id=(
                    carried_target_id
                ),
            )
        )

        obstacle_observations = (
            self._local_obstacle_observations(
                visible_objects=visible_objects,
                exclude_ids=exclude_ids,
            )
        )

        # --------------------------------------------------
        # LocalAvoidance currently owns motion
        # --------------------------------------------------

        if self._stack_local_avoidance_active:
            obstacle_range_limit_mm = (
                goal_distance_mm
            )

            if obstacle_range_limit_mm is None:
                obstacle_range_limit_mm = getattr(
                    self.local_avoidance,
                    "goal_distance_mm",
                    None,
                )

            if obstacle_range_limit_mm is None:
                obstacle_range_limit_mm = (
                    RETURN_LOCAL_AVOIDANCE_LOOKAHEAD_MM
                )

            obstacle_field = (
                local_obstacle_field_from_objects(
                    config=self.config,
                    visible_objects=visible_objects,
                    field_fov_rad=(
                        self.local_avoidance
                        .field_fov_rad
                    ),
                    scan_sectors=(
                        self.local_avoidance
                        .scan_sectors
                    ),
                    clear_range_mm=(
                            self.local_avoidance
                            .lookahead_distance_mm
                            * 1.5
                    ),
                    timestamp_s=now_s,
                    exclude_ids=exclude_ids,
                    default_obstacle_radius_mm=(
                        SR2026_OBJECT_RADIUS_MM
                    ),
                    max_obstacle_distance_mm=(
                        obstacle_range_limit_mm
                    ),
                )
            )

            st = self.local_avoidance.update(
                now_s=now_s,
                goal_distance_mm=(
                    goal_distance_mm
                ),
                goal_bearing_rad=(
                    goal_bearing_rad
                ),
                obstacle_observations=(
                    obstacle_observations
                ),
                obstacle_field=obstacle_field,
            )

            if st == PrimitiveStatus.RUNNING:
                return self.status

            if st == PrimitiveStatus.FAILED:
                print(
                    "[RETURN_TO_BASE][STACK]"
                    "[LOCAL_AVOIDANCE] "
                    f"FAILED "
                    f"reason={self.local_avoidance.reason} "
                    "-> normal terminal return"
                )

                self._stack_local_avoidance_active = False
                self.stack_approach = None
                self.stack_target_id = None
                self.stack_target_kind = None
                self.stack_target_ignore_ids = ()

                self.return_servo.resume()

                self._mode = MODE_SERVO
                self._mode_started = True

                return self.status

            print(
                "[RETURN_TO_BASE][STACK]"
                "[LOCAL_AVOIDANCE] "
                "complete -> stack servo"
            )

            self.local_avoidance.stop()

            self._stack_local_avoidance_active = False
            self.stack_approach = None

            return self.status

        # --------------------------------------------------
        # Nominal stack approach owns motion
        # --------------------------------------------------

        if (
                goal_distance_mm is not None
                and goal_bearing_rad is not None
                and self.local_avoidance is not None
                and self.local_avoidance
                .nominal_corridor_blocked(
            goal_distance_mm=(
                    goal_distance_mm
            ),
            goal_bearing_rad=(
                    goal_bearing_rad
            ),
            obstacle_observations=(
                    obstacle_observations
            ),
        )
        ):
            if self.stack_approach is not None:
                self.stack_approach.stop()

            self.stack_approach = None

            self.local_avoidance.start(
                lvl2=lvl2,
                calibration=self.calibration,
                now_s=now_s,
            )

            self._stack_local_avoidance_active = True

            print(
                "[RETURN_TO_BASE][STACK]"
                "[LOCAL_AVOIDANCE] "
                "stack corridor blocked "
                "-> takeover"
            )

            return self.status

        if self.stack_approach is None:
            self.stack_approach = (
                ApproachTargetServo(
                    config=self.config,
                    kind=self.stack_target_kind,
                    height_model=HeightModel(),
                    locked_target_id=(
                        self.stack_target_id
                    ),
                    fixed_target_is_high=False,
                    servo_method=(
                        ApproachServoMethod
                        .TARGET_BEARING
                    ),
                    pose_bearing_allowed=False,
                )
            )

            self.stack_approach.start(
                lvl2=lvl2,
                localisation=localisation,
                io=io,
            )

            print(
                "[RETURN_TO_BASE][STACK] "
                f"servo start "
                f"id={self.stack_target_id} "
                "fixed_height=LOW"
            )

        st = self.stack_approach.update(
            perception=perception
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            print(
                "[RETURN_TO_BASE][STACK] "
                "servo FAILED "
                f"reason="
                f"{self.stack_approach.failure_reason} "
                "-> normal terminal return"
            )

            self.stack_approach.stop()
            self.stack_approach = None

            self.stack_target_id = None
            self.stack_target_kind = None
            self.stack_target_ignore_ids = ()

            self.return_servo.resume()

            self._mode = MODE_SERVO
            self._mode_started = True

            return self.status

        self.stack_distance_mm = (
            self.stack_approach.final_distance_mm
        )

        self.stack_bearing_deg = (
            self.stack_approach.final_bearing_deg
        )

        if (
                self.stack_distance_mm is None
                or self.stack_bearing_deg is None
                or self.stack_approach.target_is_high
                is not False
        ):
            print(
                "[RETURN_TO_BASE][STACK] "
                "invalid LOW stack handoff "
                "-> normal terminal return"
            )

            self.stack_approach.stop()
            self.stack_approach = None

            self.stack_target_id = None
            self.stack_target_kind = None
            self.stack_target_ignore_ids = ()

            self.return_servo.resume()

            self._mode = MODE_SERVO
            self._mode_started = True

            return self.status

        print(
            "[RETURN_TO_BASE][STACK] "
            f"handoff ready "
            f"id={self.stack_target_id} "
            f"distance="
            f"{self.stack_distance_mm:.0f}mm "
            f"bearing="
            f"{self.stack_bearing_deg:+.1f}deg"
        )

        self.status = (
            BehaviorStatus.SUCCEEDED
        )

        return self.status

    def _update_guide_handoff_peek(
            self,
            *,
            lvl2,
            motion_backend,
            io,
            localisation,
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

        required_id = int(
            self.guide_handoff_required_id
        )

        found = next(
            (
                obs
                for obs in (arena_observations or ())
                if int(obs.get("id", -1))
                   == required_id
            ),
            None,
        )

        st = self.guide_handoff_peek.update(
            motion_backend=motion_backend,
            found_item=found,
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            print(
                "[RETURN_TO_BASE][RECOVERY] "
                "handoff peek search exhausted "
                "-> backoff"
            )

            return self._start_guide_backoff(
                motion_backend=motion_backend,
            )

        print(
            "[RETURN_TO_BASE][RECOVERY] "
            f"guide={required_id} visible during peek "
            "-> restart ReturnToBaseServo"
        )

        self.guide_handoff_peek = None
        self.guide_handoff_current_id = None
        self.guide_handoff_required_id = None
        self.guide_handoff_side = None

        self.return_servo = None
        self._mode_started = False

        self._start_return_servo(
            lvl2=lvl2,
            localisation=localisation,
            io=io,
        )

        return self.status

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
            io,
            localisation,
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
            localisation=localisation,
            io=io,
        )

        return self.status

    def _update_return_guide_search(
            self,
            *,
            lvl2,
            motion_backend,
            io,
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

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            print(
                "[RETURN_TO_BASE][RECOVERY] "
                "return-guide search exhausted"
            )

            self.return_guide_search = None

            if self._home_base_inside():
                return self._complete_home_base_fallback(
                    reason="return_guide_search_exhausted",
                )

            print(
                "[RETURN_TO_BASE][RECOVERY] "
                "outside/unknown home base -> FAILED"
            )

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

        self._start_return_servo(
            lvl2=lvl2,
            localisation=localisation,
            io=io,
        )

        return self.status

    def _update_wall_follow(
            self,
            *,
            lvl2,
            io,
            perception,
            delivered_ids,
            carried_target_id,
            arena_observations,
            wall_geometry,
            robot_pose,
            motion_backend,
    ):
        if self._try_start_stack_approach(
                io=io,
                perception=perception,
                delivered_ids=delivered_ids,
                carried_target_id=carried_target_id,
        ):
            return self.status

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

    def _update_terminal(
            self,
            *,
            io,
            perception,
            delivered_ids,
            carried_target_id,
    ):
        if self._try_start_stack_approach(
                io=io,
                perception=perception,
                delivered_ids=delivered_ids,
                carried_target_id=carried_target_id,
        ):
            return self.status

        if self._home_base_inside():
            return self._complete_home_base_fallback(
                reason="terminal_no_stack_target",
            )

        # Outside or localisation unknown:
        # HOLD remains stationary while waiting for useful
        # terminal perception.
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
        carried_target_id=None,
        **_,
    ):
        if self.status != BehaviorStatus.RUNNING:
            return self.status
        self._update_home_base_state(
            localisation=localisation,
        )

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
                io=io,
                localisation=localisation,
                arena_observations=arena_observations,
                observation_timestamp=(
                    observation_timestamp
                ),
                perception=perception,
                delivered_ids=delivered_ids,
                carried_target_id=(
                    carried_target_id
                ),
                wall_geometry=wall_geometry,
                robot_pose=robot_pose,
            )

        if self._mode == MODE_LOCAL_AVOIDANCE:
            return self._update_local_avoidance(
                lvl2=lvl2,
                motion_backend=motion_backend,
                io=io,
                localisation=localisation,
                arena_observations=(
                    arena_observations
                ),
                perception=perception,
                carried_target_id=(
                    carried_target_id
                ),
                robot_pose=robot_pose,
            )

        if self._mode == MODE_STACK_APPROACH:
            return self._update_stack_approach(
                lvl2=lvl2,
                io=io,
                localisation=localisation,
                perception=perception,
                carried_target_id=(
                    carried_target_id
                ),
            )

        if self._mode == MODE_GUIDE_SEARCH:
            return self._update_return_guide_search(
                lvl2=lvl2,
                motion_backend=motion_backend,
                io=io,
                arena_observations=arena_observations,
                localisation=localisation,
            )

        if self._mode == MODE_GUIDE_HANDOFF_PEEK:
            return self._update_guide_handoff_peek(
                lvl2=lvl2,
                motion_backend=motion_backend,
                io=io,
                localisation=localisation,
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
                io=io,
                localisation=localisation,
            )

        if self._mode == MODE_WALL_FOLLOW:
            return self._update_wall_follow(
                lvl2=lvl2,
                io=io,
                perception=perception,
                delivered_ids=delivered_ids,
                carried_target_id=(
                    carried_target_id
                ),
                arena_observations=arena_observations,
                wall_geometry=wall_geometry,
                robot_pose=robot_pose,
                motion_backend=motion_backend,
            )

        if self._mode == MODE_FINAL_DRIVE:
            return self._update_final_drive(
                motion_backend=motion_backend,
            )

        if self._mode == MODE_TERMINAL:
            return self._update_terminal(
                io=io,
                perception=perception,
                delivered_ids=delivered_ids,
                carried_target_id=(
                    carried_target_id
                ),
            )

        print(
            "[RETURN_TO_BASE] "
            f"unknown navigation mode={self._mode!r}"
        )

        self.status = BehaviorStatus.FAILED
        return self.status
