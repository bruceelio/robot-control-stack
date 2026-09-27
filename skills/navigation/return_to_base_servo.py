# skills/navigation/return_to_base_servo.py

from __future__ import annotations

import math
import time
from typing import Optional, Sequence

from motion_backends.velocity import VelocityMotionBackend
from perception.robot_geometry import (
    target_from_camera,
    target_from_base_link,
)
from navigation.servoing.servoing_controller import (
    ServoingController,
    ServoingStatus,
)
from primitives.base import Primitive, PrimitiveStatus
from config.arena import return_guide_routes


GUIDE_BEARING_OFFSET_DEG = 20.0

# Virtual forward error supplied to ServoingController.
# This controls cruise speed; it is NOT tag distance.
GUIDE_FORWARD_ERROR_MM = 250.0
GUIDE_STOP_DISTANCE_MM = 0.0

FINAL_GUIDE_BEARING_DEG = 0.0
FINAL_GUIDE_STOP_DISTANCE_MM = 762.5

GUIDE_HANDOFF_GUARD_DISTANCE_MM = 700.0


class ReturnToBaseServo(Primitive):
    """
    Vision-servo along an ordered sequence of arena guide tags.

    Initial Base 0 sequence:

        16 -> 17 -> 18 -> 19

    The active tag is deliberately held toward the left side
    of the camera so that the camera looks ahead toward the
    next tag.

    ServoingController owns the actual closed-loop velocity
    calculation. This skill owns guide-tag selection and handoff.
    """

    FAILURE_PERCEPTION = "perception"
    FAILURE_GUIDE_HANDOFF = "guide_handoff"

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

        self.controller = ServoingController(
            application="return_to_base",
            config=self.config,
        )

        self.velocity_backend = None

        # Not selected until a guide becomes visible.
        self.selected_route_index = None
        self.guide_ids = ()
        self.active_index = 0
        self.desired_bearing_deg = 0.0

        self.failure_reason: Optional[str] = None
        self.terminal_target_id: Optional[int] = None
        self.terminal_target_kind: Optional[str] = None

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

    def start(self, *, lvl2, **_):
        self.controller.reset()

        self.velocity_backend = VelocityMotionBackend(
            lvl2=lvl2,
            config=self.config,
        )

        self.selected_route_index = None
        self.guide_ids = ()
        self.active_index = 0
        self.desired_bearing_deg = 0.0
        self.failure_reason = None
        self.terminal_target_id = None
        self.terminal_target_kind = None

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

        distance_mm = float(
            observation["distance"]
        )

        bearing_deg = float(
            observation["bearing"]
        )

        timestamp = float(
            observation.get("last_seen", 0.0)
        )

        stop_distance_mm = float(
            self.config.final_commit_distance_mm
        )

        result = self.controller.update(
            distance_mm=distance_mm,
            target_angle_rad=math.radians(
                bearing_deg
            ),
            target_angle_setpoint_rad=0.0,
            timestamp=timestamp,
            stop_distance_mm=stop_distance_mm,
            now=now,
        )

        self.velocity_backend.update(
            result.command
        )

        if result.status == ServoingStatus.FAILED_STALE:
            self.velocity_backend.stop()

            print(
                "[RETURN_BASE_SERVO][DELIVERED] "
                f"id={self.terminal_target_id} stale"
            )

            self.status = PrimitiveStatus.RUNNING
            return self.status

        if result.status == ServoingStatus.SUCCESS:
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
            f"vx={result.command.linear_x_mps:.3f}m/s "
            f"wz={result.command.angular_z_rps:+.3f}rad/s"
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

                print(
                    "[RETURN_BASE_SERVO] "
                    "no return-route guide visible"
                )

                self.status = PrimitiveStatus.RUNNING
                return self.status

            _, route_index = max(candidates)

            route = self.guide_routes[route_index]

            self.selected_route_index = route_index
            self.guide_ids = route["guide_ids"]

            self.active_index = max(
                index
                for index, tag_id in enumerate(
                    self.guide_ids
                )
                if tag_id in visible
            )

            if route["guide_side"] == "left":
                self.desired_bearing_deg = (
                    -GUIDE_BEARING_OFFSET_DEG
                )
            else:
                self.desired_bearing_deg = (
                    +GUIDE_BEARING_OFFSET_DEG
                )

            print(
                "[RETURN_BASE_SERVO][ROUTE] "
                f"route={route_index} "
                f"side={route['guide_side']} "
                f"guide={self.active_guide_id} "
                f"target="
                f"{self.desired_bearing_deg:+.1f}deg"
            )

        # --------------------------------------------------
        # Progress through selected tag chain
        # --------------------------------------------------

        visible_indices = [
            index
            for index in range(
                self.active_index,
                len(self.guide_ids),
            )
            if self.guide_ids[index] in visible
        ]

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

            print(
                "[RETURN_BASE_SERVO] "
                f"guide={guide_id} not visible"
            )

            self.status = PrimitiveStatus.RUNNING
            return self.status



        # --------------------------------------------------
        # Convert navigation policy into servoing error
        # --------------------------------------------------

        camera_distance_mm, camera_bearing_deg = (
            target_from_camera(
                observation=observation,
            )
        )

        guide_distance_mm, observed_bearing_deg = (
            target_from_base_link(
                observation=observation,
                config=self.config,
            )
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

                self.status = PrimitiveStatus.FAILED
                return self.status

        if guide_id == self.final_guide_id:
            # Final guide:
            #
            # Distance remains base_link referenced so the robot stops
            # at the correct delivery distance.
            #
            # Steering is camera referenced so the final guide remains
            # centred in the camera rather than on the robot centreline.
            # This works regardless of which side of the robot the
            # camera is mounted on.
            distance_mm = guide_distance_mm

            stop_distance_mm = (
                FINAL_GUIDE_STOP_DISTANCE_MM
            )

            steering_bearing_deg = camera_bearing_deg
            target_bearing_deg = FINAL_GUIDE_BEARING_DEG

        else:
            # Intermediate guides retain their existing base_link
            # steering policy.
            distance_mm = GUIDE_FORWARD_ERROR_MM
            stop_distance_mm = GUIDE_STOP_DISTANCE_MM

            steering_bearing_deg = observed_bearing_deg
            target_bearing_deg = self.desired_bearing_deg

        bearing_error_deg = (
                steering_bearing_deg
                - target_bearing_deg
        )

        result = self.controller.update(
            distance_mm=distance_mm,

            target_angle_rad=math.radians(
                steering_bearing_deg
            ),

            target_angle_setpoint_rad=math.radians(
                target_bearing_deg
            ),

            timestamp=float(
                observation_timestamp
            ),

            stop_distance_mm=stop_distance_mm,

            now=now,
        )

        # --------------------------------------------------
        # Execute ServoingController output
        # --------------------------------------------------

        self.velocity_backend.update(
            result.command
        )

        if result.status == ServoingStatus.FAILED_STALE:
            self.velocity_backend.stop()

            print(
                "[RETURN_BASE_SERVO] "
                f"guide={guide_id} stale "
                "-> waiting for fresh vision"
            )

            self.status = PrimitiveStatus.RUNNING
            return self.status

        if (
                result.status == ServoingStatus.SUCCESS
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
                f"vx={result.command.linear_x_mps:.3f}m/s "
                f"wz={result.command.angular_z_rps:+.3f}rad/s"
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
                f"vx={result.command.linear_x_mps:.3f}m/s "
                f"wz={result.command.angular_z_rps:+.3f}rad/s"
            )

        self.status = PrimitiveStatus.RUNNING
        return self.status

    def stop(self, **_):
        if self.velocity_backend is not None:
            self.velocity_backend.stop()