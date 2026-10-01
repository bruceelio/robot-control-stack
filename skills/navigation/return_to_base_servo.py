# skills/navigation/return_to_base_servo.py

from __future__ import annotations

import math
import time

from dataclasses import dataclass
from typing import Optional, Sequence

from policies.vision_grace_period import VisionGracePeriod
from motion_backends.velocity import VelocityMotionBackend
from perception.robot_geometry import (
    relative_target_from_base_link,
    relative_target_from_camera,
)
from navigation.command.velocity_arbiter import VelocityCommand
from navigation.control.distance_angle_controller import (
    DistanceAngleController,
    DistanceAngleParams,
)
from primitives.base import Primitive, PrimitiveStatus
from config.arena import marker_poses


GUIDE_BEARING_OFFSET_RAD = math.radians(20.0)

# Virtual forward error supplied to DistanceAngleController.
# This controls cruise speed; it is NOT tag distance.
GUIDE_FORWARD_ERROR_MM = 250.0
GUIDE_STOP_DISTANCE_MM = 0.0

FINAL_GUIDE_BEARING_RAD = 0.0
FINAL_GUIDE_STOP_DISTANCE_MM = 762.5

# Maximum absolute arena approach angle for direct final-guide servo.
#
# 0 deg  = directly in front of marker
# 90 deg = alongside the marker's wall
#
# More side-on approaches are left to the higher-level return behaviour,
# which may hand off to wall following when wall-entry geometry is suitable.
FINAL_GUIDE_DIRECT_APPROACH_MAX_DEG = 75.0

GUIDE_HANDOFF_GUARD_DISTANCE_MM = 700.0

@dataclass(frozen=True)
class FinalGuideState:
    guide_id: int
    visible: bool

    distance_mm: Optional[float]
    camera_bearing_deg: Optional[float]
    approach_deg: Optional[float]

class ReturnToBaseServo(Primitive):
    """
    Vision-servo along an ordered sequence of arena guide tags.

    Initial Base 0 sequence:

        16 -> 17 -> 18 -> 19

    The active tag is deliberately held toward the left side
    of the camera so that the camera looks ahead toward the
    next tag.

    DistanceAngleController owns the generic closed-loop
    distance-angle calculation. This skill owns guide-tag
    selection, freshness policy, command construction, and handoff.
    """

    FAILURE_PERCEPTION = "perception"
    FAILURE_GUIDE_HANDOFF = "guide_handoff"
    FAILURE_NO_ROUTE_GUIDE = "no_route_guide"

    def __init__(
            self,
            *,
            config,
            guide_routes: Sequence[dict],
    ):
        super().__init__()

        if not guide_routes:
            raise ValueError("guide_routes must not be empty")

        self.config = config
        self.arena_marker_poses = marker_poses(
            int(self.config.arena_size)
        )

        self.guide_routes = []

        for route in guide_routes:
            guide_ids = tuple(
                int(tag_id)
                for tag_id in route["guide_ids"]
            )

            guide_side = str(
                route["guide_side"]
            ).lower()

            if not guide_ids:
                raise ValueError(
                    "guide route must contain guide_ids"
                )

            if guide_side not in ("left", "right"):
                raise ValueError(
                    f"Unknown guide_side: {guide_side}"
                )

            self.guide_routes.append(
                {
                    "guide_ids": guide_ids,
                    "guide_side": guide_side,
                }
            )

        # Guides from which a delivered cube may replace
        # the arena navigation target:
        #
        #   - the final guide
        #   - the guide immediately before it on either route
        #
        # Derived from the zone-specific return routes rather
        # than hard-coded arena marker IDs.
        self.terminal_replacement_guide_ids = frozenset(
            guide_id
            for route in self.guide_routes
            for guide_id in (
                route["guide_ids"][-2],
                route["guide_ids"][-1],
            )
        )

        self.controller = DistanceAngleController(
            controller_id="return_to_base",
            params=DistanceAngleParams(
                linear_kp=float(
                    self.config.return_to_base_servo_linear_kp
                ),
                linear_ki=float(
                    self.config.return_to_base_servo_linear_ki
                ),
                linear_kd=float(
                    self.config.return_to_base_servo_linear_kd
                ),

                angular_kp=float(
                    self.config.return_to_base_servo_angular_kp
                ),
                angular_ki=float(
                    self.config.return_to_base_servo_angular_ki
                ),
                angular_kd=float(
                    self.config.return_to_base_servo_angular_kd
                ),

                linear_max_mps=(
                        float(
                            self.config.return_to_base_servo_linear_max_mm_s
                        )
                        / 1000.0
                ),
                angular_max_rad_s=float(
                    self.config.return_to_base_servo_angular_max_rad_s
                ),

                angle_drive_slowdown_start_rad=math.radians(
                    float(
                        self.config
                        .return_to_base_servo_drive_slowdown_start_deg
                    )
                ),

                angle_drive_cutoff_rad=math.radians(
                    float(
                        self.config
                        .return_to_base_servo_drive_cutoff_deg
                    )
                ),

                distance_tolerance_m=(
                        float(
                            self.config
                            .return_to_base_servo_stop_tolerance_mm
                        )
                        / 1000.0
                ),
                derivative_mode=(
                    self.config
                    .return_to_base_servo_derivative_mode
                ),
            ),
        )

        self.velocity_backend = None

        # Not selected until a guide becomes visible.
        self.selected_route_index = None
        self.guide_ids = ()
        self.active_index = 0
        self.desired_bearing_rad = 0.0

        self.failure_reason: Optional[str] = None
        self.terminal_target_id: Optional[int] = None

        # Stage 2 return-guide perception recovery state.
        self.recovery_guide_id: Optional[int] = None

        # Debounce short losses of the currently-active guide.
        self._guide_vision_grace = None
        self._active_guide_last_seen_s: Optional[float] = None

        self.terminal_target_kind: Optional[str] = None
        self.final_guide_visible = False
        self.final_guide_approach_deg: Optional[float] = None

    @property
    def active_guide_id(self) -> int:
        return self.guide_ids[self.active_index]

    @property
    def final_guide_id(self) -> int:
        return self.guide_ids[-1]

    @property
    def selected_guide_side(self):
        if self.selected_route_index is None:
            return None

        return self.guide_routes[
            self.selected_route_index
        ]["guide_side"]

    def observe_final_guide(
        self,
        *,
        arena_observations,
        robot_pose,
    ) -> Optional[FinalGuideState]:
        """
        Observe the selected route's final guide without
        commanding any robot motion.

        This remains usable while another navigation skill,
        such as FollowWall, owns the drivetrain.
        """

        if (
            self.selected_route_index is None
            or not self.guide_ids
        ):
            return None

        guide_id = self.final_guide_id

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
            return FinalGuideState(
                guide_id=guide_id,
                visible=False,
                distance_mm=None,
                camera_bearing_deg=None,
                approach_deg=None,
            )

        base_target = relative_target_from_base_link(
            observation=observation,
            config=self.config,
        )

        camera_target = relative_target_from_camera(
            observation=observation,
        )

        # FinalGuideState is still a legacy external interface.
        distance_mm = (
                base_target.distance_m
                * 1000.0
        )

        camera_bearing_deg = -math.degrees(
            camera_target.bearing_rad
        )

        approach_deg = (
            self._final_guide_approach_angle_deg(
                guide_id=guide_id,
                robot_pose=robot_pose,
            )
        )

        if approach_deg is not None:
            approach_deg = abs(
                float(approach_deg)
            )

        return FinalGuideState(
            guide_id=guide_id,
            visible=True,
            distance_mm=float(distance_mm),
            camera_bearing_deg=float(
                camera_bearing_deg
            ),
            approach_deg=approach_deg,
        )

    def start(self, *, lvl2, **_):
        self.controller.reset()

        self.velocity_backend = VelocityMotionBackend(
            lvl2=lvl2,
            config=self.config,
        )

        self.selected_route_index = None
        self.guide_ids = ()
        self.active_index = 0
        self.desired_bearing_rad = 0.0
        self.failure_reason = None
        self.terminal_target_id = None
        self.terminal_target_kind = None
        self.final_guide_visible = False
        self.final_guide_approach_deg = None

        self.recovery_guide_id = None
        self._active_guide_last_seen_s = None

        self._guide_vision_grace = VisionGracePeriod(
            vision_grace_s=float(
                self.config.vision_grace_period_s
            )
        )

        self.status = PrimitiveStatus.RUNNING

        print(
            "[RETURN_BASE_SERVO] start "
            f"routes={self.guide_routes}"
        )

        return self.status

    def _find_delivered_target(
        self,
        *,
        perception,
        delivered_ids,
        now: float,
    ):
        """
        Find the nearest currently-visible delivered cube.

        This is only used when a terminal arena guide has
        disappeared. Once selected, the cube id is latched.
        """

        if perception is None or not delivered_ids:
            return None

        delivered_ids = {
            int(target_id)
            for target_id in delivered_ids
        }

        max_age_s = float(
            self.config.visible_max_age_s
        )

        candidates = []

        for kind, memory in perception.objects.items():
            for target_id in delivered_ids:

                observation = memory.get(target_id)

                if observation is None:
                    continue

                if (
                    "distance" not in observation
                    or "bearing" not in observation
                ):
                    continue

                last_seen = float(
                    observation.get("last_seen", 0.0)
                )

                if now - last_seen > max_age_s:
                    continue

                candidates.append(
                    (
                        float(observation["distance"]),
                        str(kind),
                        target_id,
                        observation,
                    )
                )

        if not candidates:
            return None

        return min(
            candidates,
            key=lambda item: item[0],
        )

    def _get_terminal_target(
        self,
        *,
        perception,
        now: float,
    ):
        """
        Return the already-latched delivered cube if it is
        still freshly visible.
        """

        if (
            perception is None
            or self.terminal_target_id is None
            or self.terminal_target_kind is None
        ):
            return None

        observation = perception.objects.get(
            self.terminal_target_kind,
            {},
        ).get(
            self.terminal_target_id
        )

        if observation is None:
            return None

        last_seen = float(
            observation.get("last_seen", 0.0)
        )

        if (
            now - last_seen
            > float(self.config.visible_max_age_s)
        ):
            return None

        return observation

    def _update_terminal_target(
        self,
        *,
        observation,
        now: float,
    ) -> PrimitiveStatus:
        """
        Servo directly toward a delivered cube.

        Range and bearing are the cube's camera measurements.
        Completion uses the LOW pickup commit distance.
        """

        target_geometry = relative_target_from_camera(
            observation=observation,
        )

        distance_m = (
            target_geometry.distance_m
        )

        bearing_rad = (
            target_geometry.bearing_rad
        )

        # Legacy values retained for diagnostics only.
        distance_mm = (
                distance_m
                * 1000.0
        )

        bearing_deg = -math.degrees(
            bearing_rad
        )

        timestamp = float(
            observation.get("last_seen", 0.0)
        )

        if (
            now - timestamp
            > float(self.config.visible_max_age_s)
        ):
            self.velocity_backend.stop()

            print(
                "[RETURN_BASE_SERVO][DELIVERED] "
                f"id={self.terminal_target_id} stale"
            )

            self.status = PrimitiveStatus.RUNNING
            return self.status

        stop_distance_m = (
                float(
                    self.config.final_commit_distance_mm
                )
                / 1000.0
        )

        stop_distance_mm = (
            stop_distance_m
            * 1000.0
        )

        result = self.controller.update(
            distance_m=distance_m,
            target_distance_m=stop_distance_m,
            angle_rad=bearing_rad,
            target_angle_rad=0.0,
            timestamp=timestamp,
        )

        command = VelocityCommand(
            linear_x_mps=result.linear_mps,
            angular_z_rps=result.angular_rps,
            lateral_y_mps=0.0,
            timestamp=timestamp,
        )

        self.velocity_backend.update(
            command
        )

        if result.distance_reached:
            self.velocity_backend.stop()

            print(
                "[RETURN_BASE_SERVO][DELIVERED] "
                f"id={self.terminal_target_id} "
                f"distance={distance_mm:.0f}mm "
                f"target={stop_distance_mm:.0f}mm "
                "-> complete"
            )

            self.status = PrimitiveStatus.SUCCEEDED
            return self.status

        print(
            "[RETURN_BASE_SERVO][DELIVERED] "
            f"id={self.terminal_target_id} "
            f"distance={distance_mm:.0f}mm "
            f"stop={stop_distance_mm:.0f}mm "
            f"bearing={bearing_deg:+.1f}deg "
            f"vx={command.linear_x_mps:.3f}m/s "
            f"wz={command.angular_z_rps:+.3f}rad/s"
        )

        self.status = PrimitiveStatus.RUNNING
        return self.status

    def update(
            self,
            *,
            arena_observations,
            observation_timestamp: float,
            perception=None,
            delivered_ids=None,
            robot_pose=None,
            **_,
    ) -> PrimitiveStatus:

        if self.velocity_backend is None:
            self.failure_reason = self.FAILURE_PERCEPTION
            return PrimitiveStatus.FAILED

        now = time.time()

        # --------------------------------------------------
        # Latched delivered-cube terminal target
        # --------------------------------------------------

        if self.terminal_target_id is not None:

            observation = self._get_terminal_target(
                perception=perception,
                now=now,
            )

            if observation is None:
                self.velocity_backend.stop()

                print(
                    "[RETURN_BASE_SERVO][DELIVERED] "
                    f"id={self.terminal_target_id} "
                    "not visible"
                )

                self.status = PrimitiveStatus.RUNNING
                return self.status

            return self._update_terminal_target(
                observation=observation,
                now=now,
            )

        all_guide_ids = {
            tag_id
            for route in self.guide_routes
            for tag_id in route["guide_ids"]
        }

        visible = {
            int(obs["id"]): obs
            for obs in arena_observations
            if int(obs.get("id", -1))
               in all_guide_ids
        }

        # --------------------------------------------------
        # Select route
        # --------------------------------------------------

        if self.selected_route_index is None:

            candidates = []

            for route_index, route in enumerate(
                    self.guide_routes
            ):
                guide_ids = route["guide_ids"]



                visible_indices = [
                    index
                    for index, tag_id in enumerate(guide_ids)
                    if tag_id in visible
                ]

                final_index = len(guide_ids) - 1

                if final_index in visible_indices:
                    final_approach_deg = (
                        self._final_guide_approach_angle_deg(
                            guide_id=guide_ids[final_index],
                            robot_pose=robot_pose,
                        )
                    )

                    if (
                            final_approach_deg is not None
                            and abs(final_approach_deg)
                            > FINAL_GUIDE_DIRECT_APPROACH_MAX_DEG
                    ):
                        visible_indices.remove(final_index)

                if not visible_indices:
                    continue

                # Prefer the furthest-progressed visible guide.
                visible_index = max(visible_indices)

                candidates.append(
                    (
                        visible_index,
                        route_index,
                    )
                )

            if not candidates:
                self.velocity_backend.stop()

                self.failure_reason = (
                    self.FAILURE_NO_ROUTE_GUIDE
                )

                print(
                    "[RETURN_BASE_SERVO] "
                    "no return-route guide visible "
                    "-> FAILED no_route_guide"
                )

                self.status = PrimitiveStatus.FAILED
                return self.status

            visible_index, route_index = max(candidates)

            route = self.guide_routes[route_index]

            self.selected_route_index = route_index
            self.guide_ids = route["guide_ids"]

            self.active_index = visible_index

            if route["guide_side"] == "left":
                self.desired_bearing_rad = (
                    +GUIDE_BEARING_OFFSET_RAD
                )
            else:
                self.desired_bearing_rad = (
                    -GUIDE_BEARING_OFFSET_RAD
                )

            print(
                "[RETURN_BASE_SERVO][ROUTE] "
                f"route={route_index} "
                f"side={route['guide_side']} "
                f"guide={self.active_guide_id} "
                f"target="
                f"{math.degrees(self.desired_bearing_rad):+.1f}deg"
            )

        # --------------------------------------------------
        # Progress through selected tag chain
        # --------------------------------------------------

        final_guide_state = (
            self.observe_final_guide(
                arena_observations=arena_observations,
                robot_pose=robot_pose,
            )
        )

        self.final_guide_visible = (
                final_guide_state is not None
                and final_guide_state.visible
        )

        self.final_guide_approach_deg = (
            None
            if final_guide_state is None
            else final_guide_state.approach_deg
        )

        visible_indices = [
            index
            for index in range(
                self.active_index,
                len(self.guide_ids),
            )
            if self.guide_ids[index] in visible
        ]

        final_index = len(self.guide_ids) - 1

        if (
                final_index in visible_indices
                and self.final_guide_approach_deg is not None
                and self.final_guide_approach_deg
                > FINAL_GUIDE_DIRECT_APPROACH_MAX_DEG
        ):
            visible_indices.remove(final_index)

            print(
                "[RETURN_BASE_SERVO][FINAL_DECISION] "
                f"guide={self.final_guide_id} "
                f"approach="
                f"{self.final_guide_approach_deg:.1f}deg "
                f"direct_max="
                f"{FINAL_GUIDE_DIRECT_APPROACH_MAX_DEG:.1f}deg "
                "-> defer final handoff"
            )

        if visible_indices:
            new_index = max(visible_indices)

            if new_index != self.active_index:

                old_id = self.active_guide_id

                self.active_index = new_index

                print(
                    "[RETURN_BASE_SERVO][HANDOFF] "
                    f"{old_id} -> "
                    f"{self.active_guide_id}"
                )

        guide_id = self.active_guide_id

        # --------------------------------------------------
        # Current guide must be visible
        # --------------------------------------------------

        observation = visible.get(guide_id)

        if observation is not None:
            self._active_guide_last_seen_s = now

            # Reset/refresh the shared vision-grace policy while
            # the required guide is healthy.
            self._guide_vision_grace.evaluate(
                visible_now=True,
                age_s=0.0,
            )

        if observation is None:

            if guide_id in self.terminal_replacement_guide_ids:

                candidate = self._find_delivered_target(
                    perception=perception,
                    delivered_ids=delivered_ids,
                    now=now,
                )

                if candidate is not None:
                    (
                        _,
                        kind,
                        target_id,
                        observation,
                    ) = candidate

                    self.terminal_target_id = target_id
                    self.terminal_target_kind = kind

                    # New physical target and new bearing setpoint:
                    # discard controller history from the arena guide.
                    self.controller.reset()

                    print(
                        "[RETURN_BASE_SERVO][TERMINAL_HANDOFF] "
                        f"guide={guide_id} -> "
                        f"delivered {kind} id={target_id}"
                    )

                    return self._update_terminal_target(
                        observation=observation,
                        now=now,
                    )

            self.velocity_backend.stop()

            if self._active_guide_last_seen_s is None:
                guide_loss_age_s = float("inf")
            else:
                guide_loss_age_s = max(
                    0.0,
                    now - self._active_guide_last_seen_s,
                )

            grace = self._guide_vision_grace.evaluate(
                visible_now=False,
                age_s=guide_loss_age_s,
            )

            if not grace.lost_long_enough:
                print(
                    "[RETURN_BASE_SERVO] "
                    f"guide={guide_id} temporarily not visible "
                    f"age={guide_loss_age_s:.2f}s "
                    f"grace={grace.grace_s:.2f}s"
                )

                self.status = PrimitiveStatus.RUNNING
                return self.status

            self.failure_reason = (
                self.FAILURE_NO_ROUTE_GUIDE
            )

            # We lost the current guide after already establishing the
            # route. For now recovery may accept any useful route guide;
            # ReturnToBaseServo will select the furthest-progressed
            # visible guide when it restarts.
            self.recovery_guide_id = None

            print(
                "[RETURN_BASE_SERVO] "
                f"guide={guide_id} not visible "
                f"for {guide_loss_age_s:.2f}s "
                "-> FAILED no_route_guide"
            )

            self.status = PrimitiveStatus.FAILED
            return self.status



        # --------------------------------------------------
        # Convert navigation policy into visual_servoing error
        # --------------------------------------------------

        camera_target = relative_target_from_camera(
            observation=observation,
        )

        guide_target = relative_target_from_base_link(
            observation=observation,
            config=self.config,
        )

        guide_distance_m = (
            guide_target.distance_m
        )

        camera_bearing_rad = (
            camera_target.bearing_rad
        )

        observed_bearing_rad = (
            guide_target.bearing_rad
        )

        # Legacy display/safety values.
        guide_distance_mm = (
                guide_distance_m
                * 1000.0
        )

        camera_bearing_deg = math.degrees(
            camera_bearing_rad
        )

        observed_bearing_deg = math.degrees(
            observed_bearing_rad
        )

        # --------------------------------------------------
        # Intermediate guide safety boundary
        # --------------------------------------------------

        if guide_id != self.final_guide_id:

            next_id = self.guide_ids[
                self.active_index + 1
                ]

            if (
                    guide_id
                    not in self.terminal_replacement_guide_ids
                    and guide_distance_mm
                    <= GUIDE_HANDOFF_GUARD_DISTANCE_MM
                    and next_id not in visible
            ):
                self.velocity_backend.stop()

                print(
                    "[RETURN_BASE_SERVO] "
                    f"guide={guide_id} "
                    f"distance={guide_distance_mm:.0f}mm "
                    f"next={next_id} not visible "
                    "-> FAILED guide_handoff"
                )

                self.failure_reason = (
                    self.FAILURE_GUIDE_HANDOFF
                )

                # Unlike generic perception loss, this failure tells recovery
                # exactly which guide Stage 2 needs next.
                self.recovery_guide_id = int(next_id)

                self.status = PrimitiveStatus.FAILED
                return self.status

        if guide_id == self.final_guide_id:
            distance_m = guide_distance_m

            stop_distance_m = (
                    FINAL_GUIDE_STOP_DISTANCE_MM
                    / 1000.0
            )

            steering_bearing_rad = (
                camera_bearing_rad
            )

            target_bearing_rad = (
                FINAL_GUIDE_BEARING_RAD
            )

        else:
            distance_m = (
                    GUIDE_FORWARD_ERROR_MM
                    / 1000.0
            )

            stop_distance_m = (
                    GUIDE_STOP_DISTANCE_MM
                    / 1000.0
            )

            steering_bearing_rad = (
                observed_bearing_rad
            )

            target_bearing_rad = (
                self.desired_bearing_rad
            )

        distance_mm = (
            distance_m * 1000.0
        )

        stop_distance_mm = (
            stop_distance_m * 1000.0
        )

        target_bearing_deg = math.degrees(
            target_bearing_rad
        )

        bearing_error_deg = math.degrees(
            steering_bearing_rad
            - target_bearing_rad
        )


        if (
            now - float(observation_timestamp)
            > float(self.config.visible_max_age_s)
        ):
            self.velocity_backend.stop()

            print(
                "[RETURN_BASE_SERVO] "
                f"guide={guide_id} stale "
                "-> waiting for fresh vision"
            )

            self.status = PrimitiveStatus.RUNNING
            return self.status

        result = self.controller.update(
            distance_m=distance_m,
            target_distance_m=stop_distance_m,
            angle_rad=steering_bearing_rad,
            target_angle_rad=target_bearing_rad,
            timestamp=float(
                observation_timestamp
            ),
        )

        command = VelocityCommand(
            linear_x_mps=result.linear_mps,
            angular_z_rps=result.angular_rps,
            lateral_y_mps=0.0,
            timestamp=float(
                observation_timestamp
            ),
        )

        # --------------------------------------------------
        # Execute DistanceAngleController output
        # --------------------------------------------------

        self.velocity_backend.update(
            command
        )

        if (
                result.distance_reached
                and guide_id == self.final_guide_id
        ):
            self.velocity_backend.stop()

            print(
                "[RETURN_BASE_SERVO] "
                f"final guide={guide_id} "
                f"distance={distance_mm:.0f}mm "
                f"target={stop_distance_mm:.0f}mm "
                "-> complete"
            )

            self.status = PrimitiveStatus.SUCCEEDED
            return self.status

        if guide_id == self.final_guide_id:
            print(
                "[RETURN_BASE_SERVO] "
                f"guide={guide_id} final "
                f"distance={distance_mm:.0f}mm "
                f"stop={stop_distance_mm:.0f}mm "
                f"cam_bearing={camera_bearing_deg:+.1f}deg "
                f"base_bearing={observed_bearing_deg:+.1f}deg "
                f"target={target_bearing_deg:+.1f}deg "
                f"error={bearing_error_deg:+.1f}deg "
                f"vx={command.linear_x_mps:.3f}m/s "
                f"wz={command.angular_z_rps:+.3f}rad/s"
            )



        else:
            next_id = self.guide_ids[
                self.active_index + 1
                ]

            print(
                "[RETURN_BASE_SERVO] "
                f"guide={guide_id} "
                f"next={next_id} "
                f"cam_bearing={camera_bearing_deg:+.1f}deg "
                f"base_bearing={observed_bearing_deg:+.1f}deg "
                f"target={target_bearing_deg:+.1f}deg "
                f"error={bearing_error_deg:+.1f}deg "
                f"vx={command.linear_x_mps:.3f}m/s "
                f"wz={command.angular_z_rps:+.3f}rad/s"
            )

        self.status = PrimitiveStatus.RUNNING
        return self.status

    def _final_guide_approach_angle_deg(
        self,
        *,
        guide_id: int,
        robot_pose,
    ) -> float | None:
        """
        Angle between the marker's inward-facing normal and
        the marker->robot direction.

        0 deg:
            robot is directly in front of the marker.

        90 deg:
            robot is approximately alongside the marker's wall.

        Position is sufficient; robot heading is not required.
        """

        if (
            robot_pose is None
            or not getattr(
                robot_pose,
                "position_valid",
                False,
            )
        ):
            return None

        marker_pose = self.arena_marker_poses.get(
            int(guide_id)
        )

        if marker_pose is None:
            return None

        marker_x_mm = (
            float(marker_pose["x_m"]) * 1000.0
        )
        marker_y_mm = (
            float(marker_pose["y_m"]) * 1000.0
        )

        marker_to_robot_x = (
            float(robot_pose.x)
            - marker_x_mm
        )
        marker_to_robot_y = (
            float(robot_pose.y)
            - marker_y_mm
        )

        distance_mm = math.hypot(
            marker_to_robot_x,
            marker_to_robot_y,
        )

        if distance_mm <= 1e-6:
            return 0.0

        marker_yaw_rad = float(
            marker_pose["yaw_rad"]
        )

        inward_x = math.cos(
            marker_yaw_rad
        )
        inward_y = math.sin(
            marker_yaw_rad
        )

        cosine = (
            marker_to_robot_x * inward_x
            + marker_to_robot_y * inward_y
        ) / distance_mm

        cosine = max(
            -1.0,
            min(1.0, cosine),
        )

        return math.degrees(
            math.acos(cosine)
        )

    def resume(self):
        """
        Resume the existing selected return route after another
        navigation skill has temporarily owned the drivetrain.

        Route selection and guide progress are preserved.
        """

        if self.velocity_backend is None:
            raise RuntimeError(
                "ReturnToBaseServo cannot resume before start()"
            )

        # Discard PID/controller history from before the handoff,
        # but preserve the selected route and active guide.
        self.controller.reset()

        self.status = PrimitiveStatus.RUNNING

        print(
            "[RETURN_BASE_SERVO][RESUME] "
            f"route={self.selected_route_index} "
            f"side={self.selected_guide_side} "
            f"guide={self.active_guide_id}"
        )

        return self.status

    def stop(self, **_):
        if self.velocity_backend is not None:
            self.velocity_backend.stop()