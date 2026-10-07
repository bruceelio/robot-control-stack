# skills/navigation/local_avoidance.py

from __future__ import annotations

import math

from dataclasses import dataclass, replace
from typing import Iterable, Optional

from motion_backends.velocity import VelocityMotionBackend
from navigation.command.velocity_arbiter import VelocityCommand

from navigation.local_planning.dynamic_window import (
    DynamicWindowConfig,
)
from navigation.local_planning.follow_the_gap import (
    FollowTheGapConfig,
)
from navigation.local_planning.local_planning_coordinator import (
    CoordinatedLocalPlanningResult,
    LocalPlanningCoordinatorConfig,
    plan_local_motion,
)
from navigation.local_planning.models import (
    LocalObstacleField,
    LocalPlanningRequest,
    PolarRangeSample,
    PreferredDirection,
    RobotCollisionGeometry,
    RobotDynamicLimits,
    RobotMotionState,
)

from primitives.base import (
    Primitive,
    PrimitiveStatus,
)


# ==================================================
# Defaults
# ==================================================

DEFAULT_ANGULAR_ACCEL_RAD_S2 = 4.0

DEFAULT_COLLISION_SAFETY_MARGIN_MM = 75.0
DEFAULT_DIRECT_RELEASE_CONFIRMATIONS = 2
DEFAULT_DIRECT_RELEASE_TOLERANCE_RAD = math.radians(2.0)
DEFAULT_HANDOFF_LATERAL_CLEARANCE_MM = 120.0

DEFAULT_DWA_ANGULAR_SAMPLES = 15
DEFAULT_DWA_CONTROL_DT_S = 0.10
DEFAULT_DWA_INTEGRATION_DT_S = 0.05
DEFAULT_DWA_LINEAR_SAMPLES = 7
DEFAULT_DWA_PREDICTION_HORIZON_S = 1.00

DEFAULT_FTG_EDGE_MARGIN_DEG = 2.0

DEFAULT_GOAL_ESTIMATE_TIMEOUT_S = 2.0

DEFAULT_LINEAR_ACCEL_MM_S2 = 1000.0
DEFAULT_LINEAR_DECEL_MM_S2 = 1200.0

DEFAULT_LOOKAHEAD_DISTANCE_MM = 1600.0
DEFAULT_NO_GAP_RECOVERY_ANGULAR_RAD_S = 0.4

DEFAULT_OBSTACLE_TRACK_MAX_AGE_S = 2.0

DEFAULT_SCAN_SECTORS = 31


# ==================================================
# Public input model
# ==================================================

@dataclass(frozen=True)
class LocalObstacleObservation:
    """
    One local obstacle expressed in base_link coordinates.

    Coordinate convention:

        +x / bearing 0   = forward
        +bearing         = left / counter-clockwise
        -bearing         = right / clockwise

    This is deliberately sensor-agnostic.

    Camera, ToF, LiDAR or fused perception may all produce
    these observations.
    """

    distance_mm: float
    bearing_rad: float

    obstacle_id: Optional[int] = None

    # Physical obstacle radius used for corridor-clearance
    # and temporary polar-field construction.
    radius_mm: float = 0.0


@dataclass
class _ObstacleTrack:
    obstacle_id: int

    x_mm: float
    y_mm: float

    radius_mm: float

    last_seen_s: float


# ==================================================
# Helpers
# ==================================================

def _cfg(
    config,
    name: str,
    default,
):
    value = getattr(
        config,
        name,
        None,
    )

    return (
        default
        if value is None
        else value
    )


def _relative_xy(
    distance_mm: float,
    bearing_rad: float,
) -> tuple[float, float]:
    return (
        float(distance_mm)
        * math.cos(bearing_rad),

        float(distance_mm)
        * math.sin(bearing_rad),
    )


def _propagate_relative_point(
    x_mm: float,
    y_mm: float,
    *,
    linear_mm_s: float,
    angular_rad_s: float,
    dt_s: float,
) -> tuple[float, float]:
    """
    Propagate a stationary world point through robot motion.

    The point remains expressed in the robot's current frame.

    This is robot-relative short-term propagation only.
    It does not require global localisation.
    """

    if dt_s <= 0.0:
        return (
            x_mm,
            y_mm,
        )

    dtheta = (
        angular_rad_s
        * dt_s
    )

    if abs(
        angular_rad_s
    ) < 1e-9:
        robot_dx_mm = (
            linear_mm_s
            * dt_s
        )

        robot_dy_mm = 0.0

    else:
        radius_mm = (
            linear_mm_s
            / angular_rad_s
        )

        robot_dx_mm = (
            radius_mm
            * math.sin(dtheta)
        )

        robot_dy_mm = (
            radius_mm
            * (
                1.0
                - math.cos(dtheta)
            )
        )

    translated_x_mm = (
        x_mm
        - robot_dx_mm
    )

    translated_y_mm = (
        y_mm
        - robot_dy_mm
    )

    c = math.cos(dtheta)
    s = math.sin(dtheta)

    return (
        c * translated_x_mm
        + s * translated_y_mm,

        -s * translated_x_mm
        + c * translated_y_mm,
    )


# ==================================================
# Local Avoidance
# ==================================================

class LocalAvoidance(Primitive):
    """
    Reusable localisation-independent local-avoidance skill.

    Current composition:

        preferred local goal
            ->
        local obstacle observations
            ->
        Follow-the-Gap
            ->
        Dynamic Window Approach
            ->
        VelocityMotionBackend

    The skill owns motion only while the nominal direct corridor
    is obstructed.

    SUCCEEDED means:

        the nominal direct corridor is clear and control may
        return to the nominal navigation controller.

    For Stage 2 acquisition the nominal controller will later be
    ApproachTargetServo.

    For Stage 3 the nominal controller can instead be a path
    tracker such as Pure Pursuit.

    The core local-planning algorithms remain unaware of this
    takeover/release policy.
    """

    def __init__(
        self,
        *,
        config,
        field_fov_rad: float,
        scan_sectors: int = DEFAULT_SCAN_SECTORS,
        lookahead_distance_mm: float = (
            DEFAULT_LOOKAHEAD_DISTANCE_MM
        ),
        obstacle_track_max_age_s: float = (
            DEFAULT_OBSTACLE_TRACK_MAX_AGE_S
        ),
        goal_estimate_timeout_s: float = (
            DEFAULT_GOAL_ESTIMATE_TIMEOUT_S
        ),
    ):
        super().__init__()

        self.config = config

        self.field_fov_rad = float(
            field_fov_rad
        )

        self.scan_sectors = int(
            scan_sectors
        )

        self.lookahead_distance_mm = float(
            lookahead_distance_mm
        )

        self.obstacle_track_max_age_s = float(
            obstacle_track_max_age_s
        )

        self.goal_estimate_timeout_s = float(
            goal_estimate_timeout_s
        )

        if (
            self.scan_sectors < 3
            or self.scan_sectors % 2 == 0
        ):
            raise ValueError(
                "scan_sectors must be odd and >= 3"
            )

        if self.field_fov_rad <= 0.0:
            raise ValueError(
                "field_fov_rad must be > 0"
            )

        # ----------------------------------------------
        # Robot geometry
        # ----------------------------------------------

        drive_track_width_mm = float(
            self.config.drive_track_width_mm
        )

        self.collision_geometry = (
            RobotCollisionGeometry(
                collision_radius_mm=(
                    drive_track_width_mm
                    / 2.0
                ),
                safety_margin_mm=float(
                    _cfg(
                        self.config,
                        "local_planning_safety_margin_mm",
                        DEFAULT_COLLISION_SAFETY_MARGIN_MM,
                    )
                ),
            )
        )

        # ----------------------------------------------
        # Robot dynamic limits
        # ----------------------------------------------

        linear_max_mm_s = float(
            _cfg(
                self.config,
                "local_planning_linear_max_mm_s",
                self.config.servoing_linear_max_mm_s,
            )
        )

        angular_max_rad_s = float(
            _cfg(
                self.config,
                "local_planning_angular_max_rad_s",
                self.config.servoing_angular_max_rad_s,
            )
        )

        self.dynamic_limits = (
            RobotDynamicLimits(
                linear_min_mm_s=0.0,
                linear_max_mm_s=(
                    linear_max_mm_s
                ),
                angular_max_rad_s=(
                    angular_max_rad_s
                ),
                linear_accel_mm_s2=float(
                    _cfg(
                        self.config,
                        "local_planning_linear_accel_mm_s2",
                        DEFAULT_LINEAR_ACCEL_MM_S2,
                    )
                ),
                linear_decel_mm_s2=float(
                    _cfg(
                        self.config,
                        "local_planning_linear_decel_mm_s2",
                        DEFAULT_LINEAR_DECEL_MM_S2,
                    )
                ),
                angular_accel_rad_s2=float(
                    _cfg(
                        self.config,
                        "local_planning_angular_accel_rad_s2",
                        DEFAULT_ANGULAR_ACCEL_RAD_S2,
                    )
                ),
            )
        )

        # ----------------------------------------------
        # Planner composition
        # ----------------------------------------------

        self.planner_config = (
            LocalPlanningCoordinatorConfig(
                follow_the_gap=FollowTheGapConfig(
                    lookahead_distance_mm=(
                        self.lookahead_distance_mm
                    ),
                    min_gap_width_rad=0.0,
                    edge_margin_rad=math.radians(
                        DEFAULT_FTG_EDGE_MARGIN_DEG
                    ),
                    min_confidence=0.0,
                    unknown_is_blocked=True,
                ),
                dynamic_window=DynamicWindowConfig(
                    control_dt_s=(
                        DEFAULT_DWA_CONTROL_DT_S
                    ),
                    prediction_horizon_s=(
                        DEFAULT_DWA_PREDICTION_HORIZON_S
                    ),
                    integration_dt_s=(
                        DEFAULT_DWA_INTEGRATION_DT_S
                    ),
                    linear_samples=(
                        DEFAULT_DWA_LINEAR_SAMPLES
                    ),
                    angular_samples=(
                        DEFAULT_DWA_ANGULAR_SAMPLES
                    ),
                    heading_weight=0.50,
                    clearance_weight=0.35,
                    speed_weight=0.15,
                    clearance_normalization_mm=1000.0,
                    braking_margin_mm=50.0,
                    min_confidence=0.0,
                    unknown_is_blocked=True,
                    unknown_obstacle_distance_mm=0.0,
                ),
                velocity_obstacle=None,
            )
        )

        # ----------------------------------------------
        # Runtime
        # ----------------------------------------------

        self.velocity_backend = None

        self._goal_x_mm: Optional[float] = None
        self._goal_y_mm: Optional[float] = None
        self._goal_last_seen_s: Optional[float] = None
        self._direction_bearing_rad: Optional[float] = None

        self._tracks: dict[
            int,
            _ObstacleTrack,
        ] = {}

        self._previous_update_s: Optional[float] = None

        self._direct_release_count = 0

        self.last_linear_mm_s = 0.0
        self.last_angular_rad_s = 0.0

        self.avoidance_engaged = False

        # Diagnostic only for now.
        # We are deliberately NOT adding side-hysteresis yet.
        self.avoidance_side: Optional[int] = None

        self.blocking_obstacle_id: Optional[int] = None
        self.blocking_lateral_mm: Optional[float] = None

        self.goal_distance_mm: Optional[float] = None
        self.goal_bearing_rad: Optional[float] = None
        self.preferred_bearing_rad: Optional[float] = None
        self.goal_age_s: Optional[float] = None
        self.goal_visible = False

        self.last_plan: Optional[
            CoordinatedLocalPlanningResult
        ] = None

        self.reason: Optional[str] = None

    # ==================================================
    # Lifecycle
    # ==================================================

    def start(
        self,
        *,
        lvl2,
        calibration=None,
        now_s: Optional[float] = None,
        localisation=None,
        io=None,
        **_,
    ):
        self.velocity_backend = (
            VelocityMotionBackend(
                lvl2=lvl2,
                config=self.config,
                calibration=calibration,
                localisation=localisation,
                io=io,
            )
        )

        self._goal_x_mm = None
        self._goal_y_mm = None
        self._goal_last_seen_s = None
        self._direction_bearing_rad = None

        self._tracks.clear()

        self._previous_update_s = (
            float(now_s)
            if now_s is not None
            else None
        )

        self._direct_release_count = 0

        self.last_linear_mm_s = 0.0
        self.last_angular_rad_s = 0.0

        self.avoidance_engaged = False
        self.avoidance_side = None

        self.blocking_obstacle_id = None
        self.blocking_lateral_mm = None

        self.goal_distance_mm = None
        self.goal_bearing_rad = None
        self.preferred_bearing_rad = None
        self.goal_age_s = None
        self.goal_visible = False

        self.last_plan = None
        self.reason = None

        self.status = PrimitiveStatus.RUNNING

        print(
            "[LOCAL_AVOIDANCE] start "
            f"fov="
            f"{math.degrees(self.field_fov_rad):.1f}deg "
            f"collision_radius="
            f"{self.collision_geometry.collision_radius_mm:.0f}mm "
            f"safety_margin="
            f"{self.collision_geometry.safety_margin_mm:.0f}mm "
            f"v_max="
            f"{self.dynamic_limits.linear_max_mm_s:.0f}mm/s "
            f"w_max="
            f"{self.dynamic_limits.angular_max_rad_s:.2f}rad/s"
        )

        return self.status

    def stop(
        self,
        **_,
    ):
        if self.velocity_backend is not None:
            self.velocity_backend.stop()

        self.last_linear_mm_s = 0.0
        self.last_angular_rad_s = 0.0

    # ==================================================
    # Robot-relative propagation
    # ==================================================

    def _propagate_state(
        self,
        *,
        dt_s: float,
    ) -> None:
        if (
            self._goal_x_mm is not None
            and self._goal_y_mm is not None
        ):
            (
                self._goal_x_mm,
                self._goal_y_mm,
            ) = _propagate_relative_point(
                self._goal_x_mm,
                self._goal_y_mm,
                linear_mm_s=(
                    self.last_linear_mm_s
                ),
                angular_rad_s=(
                    self.last_angular_rad_s
                ),
                dt_s=dt_s,
            )

        for track in self._tracks.values():
            (
                track.x_mm,
                track.y_mm,
            ) = _propagate_relative_point(
                track.x_mm,
                track.y_mm,
                linear_mm_s=(
                    self.last_linear_mm_s
                ),
                angular_rad_s=(
                    self.last_angular_rad_s
                ),
                dt_s=dt_s,
            )

        # A direction-only route intent is effectively a point at
        # infinity: robot translation does not consume any distance.
        #
        # Robot rotation does change the same world direction in the
        # current base_link frame.
        if (
            self._direction_bearing_rad is not None
            and dt_s > 0.0
        ):
            dtheta = (
                self.last_angular_rad_s
                * dt_s
            )

            self._direction_bearing_rad = math.atan2(
                math.sin(
                    self._direction_bearing_rad
                    - dtheta
                ),
                math.cos(
                    self._direction_bearing_rad
                    - dtheta
                ),
            )

    def _active_preferred_bearing_rad(
        self,
    ) -> Optional[float]:
        if (
            self._goal_x_mm is not None
            and self._goal_y_mm is not None
        ):
            return math.atan2(
                self._goal_y_mm,
                self._goal_x_mm,
            )

        return self._direction_bearing_rad

    def _active_corridor_distance_mm(
        self,
    ) -> Optional[float]:
        if (
            self._goal_x_mm is not None
            and self._goal_y_mm is not None
        ):
            return math.hypot(
                self._goal_x_mm,
                self._goal_y_mm,
            )

        if self._direction_bearing_rad is not None:
            return self.lookahead_distance_mm

        return None

    def _corridor_endpoint_xy(
        self,
    ) -> Optional[tuple[float, float]]:
        bearing_rad = (
            self._active_preferred_bearing_rad()
        )

        distance_mm = (
            self._active_corridor_distance_mm()
        )

        if (
            bearing_rad is None
            or distance_mm is None
        ):
            return None

        return _relative_xy(
            distance_mm,
            bearing_rad,
        )

    # ==================================================
    # Obstacle tracking
    # ==================================================

    def _update_tracks(
        self,
        *,
        observations: Iterable[
            LocalObstacleObservation
        ],
        now_s: float,
    ) -> set[int]:
        currently_seen_ids: set[int] = set()

        for observation in observations:
            if observation.obstacle_id is None:
                continue

            if (
                observation.distance_mm <= 0.0
                or not math.isfinite(
                    observation.distance_mm
                )
                or not math.isfinite(
                    observation.bearing_rad
                )
            ):
                continue

            obstacle_id = int(
                observation.obstacle_id
            )

            currently_seen_ids.add(
                obstacle_id
            )

            x_mm, y_mm = _relative_xy(
                observation.distance_mm,
                observation.bearing_rad,
            )

            self._tracks[obstacle_id] = (
                _ObstacleTrack(
                    obstacle_id=obstacle_id,
                    x_mm=x_mm,
                    y_mm=y_mm,
                    radius_mm=max(
                        0.0,
                        float(
                            observation.radius_mm
                        ),
                    ),
                    last_seen_s=now_s,
                )
            )



        return currently_seen_ids

    # ==================================================
    # Corridor release
    # ==================================================



    def nominal_corridor_blocked(
            self,
            *,
            obstacle_observations: Iterable[
                LocalObstacleObservation
            ],
            goal_distance_mm: Optional[float] = None,
            goal_bearing_rad: Optional[float] = None,
            preferred_bearing_rad: Optional[float] = None,
            lookahead_distance_mm: Optional[float] = None,
    ) -> bool:
        """
        Passive test of the nominal corridor.

        A genuine point target may provide both distance and bearing.

        A route controller may instead provide only a preferred
        bearing; in that case lookahead_distance_mm defines how far
        ahead obstacle relevance is tested. It is not a motion goal.
        """

        if (
                goal_distance_mm is not None
                and goal_bearing_rad is not None
        ):
            corridor_distance_mm = float(
                goal_distance_mm
            )
            corridor_bearing_rad = float(
                goal_bearing_rad
            )

        elif preferred_bearing_rad is not None:
            corridor_distance_mm = float(
                self.lookahead_distance_mm
                if lookahead_distance_mm is None
                else lookahead_distance_mm
            )
            corridor_bearing_rad = float(
                preferred_bearing_rad
            )

        else:
            return False

        if (
                corridor_distance_mm <= 0.0
                or not math.isfinite(
            corridor_distance_mm
        )
                or not math.isfinite(
            corridor_bearing_rad
        )
        ):
            return False

        (
            corridor_x_mm,
            corridor_y_mm,
        ) = _relative_xy(
            corridor_distance_mm,
            corridor_bearing_rad,
        )

        corridor_length_sq = (
                corridor_x_mm * corridor_x_mm
                + corridor_y_mm * corridor_y_mm
        )

        if corridor_length_sq <= 1e-9:
            return False

        robot_radius_mm = (
                float(
                    self.collision_geometry
                    .collision_radius_mm
                )
                + float(
            self.collision_geometry
            .safety_margin_mm
        )
        )

        for observation in obstacle_observations:
            distance_mm = float(
                observation.distance_mm
            )

            bearing_rad = float(
                observation.bearing_rad
            )

            if (
                    distance_mm <= 0.0
                    or not math.isfinite(distance_mm)
                    or not math.isfinite(bearing_rad)
            ):
                continue

            obstacle_x_mm, obstacle_y_mm = (
                _relative_xy(
                    distance_mm,
                    bearing_rad,
                )
            )

            along = (
                            obstacle_x_mm * corridor_x_mm
                            + obstacle_y_mm * corridor_y_mm
                    ) / corridor_length_sq

            if (
                    along <= 0.0
                    or along >= 1.0
            ):
                continue

            closest_x_mm = (
                    along * corridor_x_mm
            )
            closest_y_mm = (
                    along * corridor_y_mm
            )

            lateral_mm = math.hypot(
                obstacle_x_mm - closest_x_mm,
                obstacle_y_mm - closest_y_mm,
            )

            required_clearance_mm = (
                    robot_radius_mm
                    + max(
                0.0,
                float(observation.radius_mm),
            )
            )

            if lateral_mm <= required_clearance_mm:
                return True

        return False

    def _blocking_tracks(
            self,
    ) -> list[
        tuple[
            _ObstacleTrack,
            float,
        ]
    ]:
        corridor_endpoint = (
            self._corridor_endpoint_xy()
        )

        if corridor_endpoint is None:
            return []

        (
            corridor_x_mm,
            corridor_y_mm,
        ) = corridor_endpoint

        corridor_length_sq = (
                corridor_x_mm
                * corridor_x_mm
                + corridor_y_mm
                * corridor_y_mm
        )

        if corridor_length_sq <= 1e-9:
            return []

        robot_radius_mm = (
                float(
                    self.collision_geometry
                    .collision_radius_mm
                )
                + float(
            self.collision_geometry
            .safety_margin_mm
        )
        )

        blocked: list[
            tuple[
                _ObstacleTrack,
                float,
            ]
        ] = []

        for track in self._tracks.values():
            along = (
                            track.x_mm
                            * corridor_x_mm
                            + track.y_mm
                            * corridor_y_mm
                    ) / corridor_length_sq

            # Only the finite robot -> corridor-horizon
            # segment matters.
            if (
                    along <= 0.0
                    or along >= 1.0
            ):
                continue

            closest_x_mm = (
                    along
                    * corridor_x_mm
            )

            closest_y_mm = (
                    along
                    * corridor_y_mm
            )

            lateral_mm = math.hypot(
                track.x_mm
                - closest_x_mm,
                track.y_mm
                - closest_y_mm,
            )

            corridor_radius_mm = (
                    robot_radius_mm
                    + track.radius_mm
            )

            if (
                    lateral_mm
                    <= corridor_radius_mm
            ):
                blocked.append(
                    (
                        track,
                        lateral_mm,
                    )
                )

        blocked.sort(
            key=lambda item: item[1]
        )

        return blocked

    def _handoff_corridor_clear(
            self,
            *,
            blocking_tracks: list[
                tuple[
                    _ObstacleTrack,
                    float,
                ]
            ],
    ) -> bool:
        """
        Return True once all remaining blockers have enough
        lateral separation for control to return to the
        nominal controller.

        This is a handoff threshold, not a collision envelope.

        The nominal controller is expected to curve back toward
        its goal. LocalAvoidance may take control again if that
        nominal motion subsequently becomes obstructed.
        """

        for (
                _track,
                lateral_mm,
        ) in blocking_tracks:
            if (
                    lateral_mm
                    < DEFAULT_HANDOFF_LATERAL_CLEARANCE_MM
            ):
                return False

        return True

    def _prune_stale_nonblocking_tracks(
        self,
        *,
        now_s: float,
        blocking_tracks: list[
            tuple[
                _ObstacleTrack,
                float,
            ]
        ],
    ) -> None:
        """
        Remove stale tracks only after they are no longer relevant
        to the nominal corridor.

        A stale obstacle which still geometrically blocks the
        robot-to-goal corridor must not disappear and thereby
        create a false corridor-clear handoff.
        """

        blocking_ids = {
            track.obstacle_id
            for (
                track,
                _,
            ) in blocking_tracks
        }

        stale_nonblocking_ids = [
            obstacle_id
            for (
                obstacle_id,
                track,
            ) in self._tracks.items()
            if (
                now_s
                - track.last_seen_s
                > self.obstacle_track_max_age_s
                and obstacle_id
                not in blocking_ids
            )
        ]

        for obstacle_id in stale_nonblocking_ids:
            del self._tracks[
                obstacle_id
            ]

    # ==================================================
    # Planner obstacle field
    # ==================================================

    def _planner_observations(
        self,
        *,
        current_observations: tuple[
            LocalObstacleObservation,
            ...
        ],
        currently_seen_ids: set[int],
        blocking_tracks: list[
            tuple[
                _ObstacleTrack,
                float,
            ]
        ],
    ) -> tuple[
        LocalObstacleObservation,
        ...
    ]:
        """
        Planner receives:

            - all CURRENT obstacle observations; plus
            - only persisted obstacles which are still blocking
              the nominal corridor.

        We deliberately do NOT inject every stale track back into FTG.
        """

        observations = list(
            current_observations
        )

        for (
            track,
            _,
        ) in blocking_tracks:
            if (
                track.obstacle_id
                in currently_seen_ids
            ):
                continue

            distance_mm = math.hypot(
                track.x_mm,
                track.y_mm,
            )

            if distance_mm <= 0.0:
                continue

            observations.append(
                LocalObstacleObservation(
                    obstacle_id=(
                        track.obstacle_id
                    ),
                    distance_mm=(
                        distance_mm
                    ),
                    bearing_rad=math.atan2(
                        track.y_mm,
                        track.x_mm,
                    ),
                    radius_mm=(
                        track.radius_mm
                    ),
                )
            )

        return tuple(
            observations
        )

    def _build_obstacle_field(
        self,
        *,
        observations: Iterable[
            LocalObstacleObservation
        ],
        timestamp_s: float,
    ) -> LocalObstacleField:
        sector_width_rad = (
            self.field_fov_rad
            / float(
                self.scan_sectors
            )
        )

        half_fov_rad = (
            self.field_fov_rad
            / 2.0
        )

        bearings = [
            -half_fov_rad
            + sector_width_rad
            * (index + 0.5)
            for index
            in range(
                self.scan_sectors
            )
        ]

        clear_range_mm = (
            self.lookahead_distance_mm
            * 1.5
        )

        ranges = [
            clear_range_mm
            for _ in bearings
        ]

        corridor_distance_mm = (
            self._active_corridor_distance_mm()
        )

        for observation in observations:
            distance_mm = float(
                observation.distance_mm
            )

            bearing_rad = float(
                observation.bearing_rad
            )

            radius_mm = max(
                0.0,
                float(
                    observation.radius_mm
                ),
            )

            if (
                distance_mm <= 0.0
                or not math.isfinite(
                    distance_mm
                )
                or not math.isfinite(
                    bearing_rad
                )
            ):
                continue

            # Obstacles beyond the active point goal or
            # direction-only planning horizon cannot block
            # this local manoeuvre.
            if (
                    corridor_distance_mm is not None
                    and distance_mm
                    >= corridor_distance_mm
            ):
                continue

            if (
                bearing_rad < -half_fov_rad
                or bearing_rad > half_fov_rad
            ):
                continue

            obstacle_half_width_rad = (
                math.atan2(
                    radius_mm,
                    distance_mm,
                )
            )

            for (
                index,
                sample_bearing_rad,
            ) in enumerate(
                bearings
            ):
                if abs(
                    sample_bearing_rad
                    - bearing_rad
                ) <= (
                    obstacle_half_width_rad
                    + sector_width_rad / 2.0
                ):
                    ranges[index] = min(
                        ranges[index],
                        distance_mm,
                    )

        samples = tuple(
            PolarRangeSample(
                bearing_rad=bearing_rad,
                range_mm=range_mm,
                angular_width_rad=(
                    sector_width_rad
                ),
                confidence=1.0,
            )
            for (
                bearing_rad,
                range_mm,
            ) in zip(
                bearings,
                ranges,
            )
        )

        return LocalObstacleField(
            samples=samples,
            timestamp_s=timestamp_s,
        )

    def _overlay_persisted_blockers(
        self,
        *,
        obstacle_field: LocalObstacleField,
        blocking_tracks: list[
            tuple[
                _ObstacleTrack,
                float,
            ]
        ],
        currently_seen_ids: set[int],
    ) -> LocalObstacleField:
        """
        Overlay propagated blockers which are no longer currently
        observed onto the supplied perception obstacle field.

        Current perception remains authoritative for currently
        visible obstacles.

        This preserves obstacle continuity when a blocker leaves
        the camera FOV during an avoidance manoeuvre.
        """

        persisted_tracks = [
            track
            for (
                track,
                _,
            ) in blocking_tracks
            if (
                track.obstacle_id
                not in currently_seen_ids
            )
        ]

        if not persisted_tracks:
            return obstacle_field

        corridor_distance_mm = (
            self._active_corridor_distance_mm()
        )

        samples = []
        changed = False

        for sample in obstacle_field.samples:
            sample_bearing_rad = float(
                sample.bearing_rad
            )

            sample_half_width_rad = (
                max(
                    0.0,
                    float(
                        sample.angular_width_rad
                    ),
                )
                / 2.0
            )

            range_mm = float(
                sample.range_mm
            )

            for track in persisted_tracks:
                distance_mm = math.hypot(
                    track.x_mm,
                    track.y_mm,
                )

                if distance_mm <= 0.0:
                    continue

                if (
                    corridor_distance_mm is not None
                    and distance_mm
                    >= corridor_distance_mm
                ):
                    continue

                bearing_rad = math.atan2(
                    track.y_mm,
                    track.x_mm,
                )

                bearing_error_rad = abs(
                    math.atan2(
                        math.sin(
                            sample_bearing_rad
                            - bearing_rad
                        ),
                        math.cos(
                            sample_bearing_rad
                            - bearing_rad
                        ),
                    )
                )

                obstacle_half_width_rad = (
                    math.atan2(
                        max(
                            0.0,
                            track.radius_mm,
                        ),
                        distance_mm,
                    )
                )

                if (
                    bearing_error_rad
                    <= (
                        obstacle_half_width_rad
                        + sample_half_width_rad
                    )
                ):
                    new_range_mm = min(
                        range_mm,
                        distance_mm,
                    )

                    if new_range_mm < range_mm:
                        range_mm = new_range_mm
                        changed = True

            samples.append(
                replace(
                    sample,
                    range_mm=range_mm,
                )
            )

        if not changed:
            return obstacle_field

        return replace(
            obstacle_field,
            samples=tuple(samples),
        )

    # ==================================================
    # Update
    # ==================================================

    def update(
        self,
        *,
        now_s: float,
        goal_distance_mm: Optional[float] = None,
        goal_bearing_rad: Optional[float] = None,
        preferred_bearing_rad: Optional[float] = None,
        obstacle_observations: Iterable[
        LocalObstacleObservation
        ] = (),
        obstacle_field: Optional[
            LocalObstacleField
        ] = None,
        **_,
    ) -> PrimitiveStatus:
        if (
            self.status
            != PrimitiveStatus.RUNNING
        ):
            return self.status

        if self.velocity_backend is None:
            self.reason = "not_started"
            self.status = PrimitiveStatus.FAILED
            return self.status

        now_s = float(
            now_s
        )

        if self._previous_update_s is None:
            dt_s = 0.0

        else:
            dt_s = max(
                0.0,
                now_s
                - self._previous_update_s,
            )

        self._previous_update_s = (
            now_s
        )

        # ----------------------------------------------
        # Propagate existing local state through motion.
        # ----------------------------------------------

        self._propagate_state(
            dt_s=dt_s
        )

        # ----------------------------------------------
        # Correct nominal guidance.
        #
        # There are two deliberately different forms:
        #
        #   point goal:
        #       genuine measured range + bearing
        #
        #   direction only:
        #       route intent with no invented distance
        # ----------------------------------------------

        self.goal_visible = (
                goal_distance_mm is not None
                and goal_bearing_rad is not None
        )

        if self.goal_visible:
            distance_mm = float(
                goal_distance_mm
            )

            bearing_rad = float(
                goal_bearing_rad
            )

            if (
                    distance_mm > 0.0
                    and math.isfinite(
                distance_mm
            )
                    and math.isfinite(
                bearing_rad
            )
            ):
                (
                    self._goal_x_mm,
                    self._goal_y_mm,
                ) = _relative_xy(
                    distance_mm,
                    bearing_rad,
                )

                self._goal_last_seen_s = (
                    now_s
                )

                # A genuine point goal supersedes any
                # previous direction-only route intent.
                self._direction_bearing_rad = None

        elif (
                preferred_bearing_rad is not None
                and math.isfinite(
            float(preferred_bearing_rad)
        )
        ):
            self._direction_bearing_rad = float(
                preferred_bearing_rad
            )

            # Direction-only guidance is not a synthetic
            # point target.
            self._goal_x_mm = None
            self._goal_y_mm = None
            self._goal_last_seen_s = None

        # Point-goal expiry applies only to a genuine
        # propagated target position. Direction-only
        # avoidance remains valid until its propagated
        # blocker is no longer relevant.
        if (
                self._goal_x_mm is not None
                and self._goal_y_mm is not None
                and self._goal_last_seen_s is not None
        ):
            self.goal_age_s = (
                    now_s
                    - self._goal_last_seen_s
            )

            if (
                    self.goal_age_s
                    > self.goal_estimate_timeout_s
            ):
                self.stop()

                self.reason = (
                    "goal_estimate_expired"
                )

                self.status = (
                    PrimitiveStatus.FAILED
                )

                print(
                    "[LOCAL_AVOIDANCE] "
                    "goal estimate expired "
                    f"age={self.goal_age_s:.2f}s"
                )

                return self.status

        else:
            self.goal_age_s = None

        self.preferred_bearing_rad = (
            self._active_preferred_bearing_rad()
        )

        if self.preferred_bearing_rad is None:
            self.stop()

            self.reason = (
                "no_local_guidance"
            )

            return PrimitiveStatus.RUNNING

        if (
                self._goal_x_mm is not None
                and self._goal_y_mm is not None
        ):
            self.goal_distance_mm = (
                math.hypot(
                    self._goal_x_mm,
                    self._goal_y_mm,
                )
            )

            self.goal_bearing_rad = (
                self.preferred_bearing_rad
            )

        else:
            self.goal_distance_mm = None
            self.goal_bearing_rad = None

        # ----------------------------------------------
        # Current + persistent obstacle state.
        # ----------------------------------------------

        current_observations = tuple(
            obstacle_observations
        )

        currently_seen_ids = (
            self._update_tracks(
                observations=(
                    current_observations
                ),
                now_s=now_s,
            )
        )

        blocking_tracks = (
            self._blocking_tracks()
        )

        self._prune_stale_nonblocking_tracks(
            now_s=now_s,
            blocking_tracks=blocking_tracks,
        )

        handoff_corridor_clear = (
            self._handoff_corridor_clear(
                blocking_tracks=blocking_tracks,
            )
        )

        if blocking_tracks:
            (
                nearest_blocker,
                nearest_lateral_mm,
            ) = blocking_tracks[0]

            self.blocking_obstacle_id = (
                nearest_blocker.obstacle_id
            )

            self.blocking_lateral_mm = (
                nearest_lateral_mm
            )

            # Establish an avoidance side from blocker geometry
            # before FTG runs.
            #
            # This matters when a close obstacle causes FTG to
            # report NO_GAP on the very first planning cycle:
            # there may not yet have been an OK FTG result from
            # which to infer a side.
            #
            # Cross product sign tells us which side of the
            # nominal goal ray the blocker lies on:
            #
            #   +cross = blocker left  -> avoid right
            #   -cross = blocker right -> avoid left
            corridor_endpoint = (
                self._corridor_endpoint_xy()
            )

            if (
                    self.avoidance_side is None
                    and corridor_endpoint is not None
            ):
                (
                    corridor_x_mm,
                    corridor_y_mm,
                ) = corridor_endpoint

                blocker_cross = (
                        corridor_x_mm
                        * nearest_blocker.y_mm
                        - corridor_y_mm
                        * nearest_blocker.x_mm
                )

                if abs(blocker_cross) > 1e-6:
                    self.avoidance_side = (
                        -1
                        if blocker_cross > 0.0
                        else 1
                    )

                    print(
                        "[LOCAL_AVOIDANCE] "
                        "initial side="
                        f"{'LEFT' if self.avoidance_side > 0 else 'RIGHT'} "
                        "source=blocking_geometry "
                        f"blocker={nearest_blocker.obstacle_id}"
                    )

            self.avoidance_engaged = True


        else:

            self.blocking_obstacle_id = None

            self.blocking_lateral_mm = None

            # If avoidance never actually engaged, there is

            # nothing for this skill to do.

            if not self.avoidance_engaged:
                self.stop()

                self.reason = (

                    "corridor_clear"

                )

                self.status = (

                    PrimitiveStatus.SUCCEEDED

                )

                return self.status

            # If avoidance DID engage, do not release here.

            # Continue through FTG/DWA so handoff uses the same

            # nominal-direction confirmation as every other case.

        # ----------------------------------------------
        # Build planner input.
        #
        # Current obstacles are always used.
        #
        # A previously-seen obstacle is reintroduced only
        # while it still geometrically blocks the nominal
        # corridor.
        # ----------------------------------------------

        if obstacle_field is None:
            planner_observations = (
                self._planner_observations(
                    current_observations=(
                        current_observations
                    ),
                    currently_seen_ids=(
                        currently_seen_ids
                    ),
                    blocking_tracks=(
                        blocking_tracks
                    ),
                )
            )

            obstacle_field = (
                self._build_obstacle_field(
                    observations=(
                        planner_observations
                    ),
                    timestamp_s=now_s,
                )
            )

        else:
            obstacle_field = (
                self._overlay_persisted_blockers(
                    obstacle_field=obstacle_field,
                    blocking_tracks=(
                        blocking_tracks
                    ),
                    currently_seen_ids=(
                        currently_seen_ids
                    ),
                )
            )

        # ----------------------------------------------
        # FTG + DWA
        # ----------------------------------------------

        request = LocalPlanningRequest(
            obstacle_field=(
                obstacle_field
            ),
            preferred_direction=(
                PreferredDirection(
                    bearing_rad=(
                        self.preferred_bearing_rad
                    ),
                    weight=1.0,
                )
            ),
            motion=(
                RobotMotionState(
                    linear_mm_s=(
                        self.last_linear_mm_s
                    ),
                    angular_rad_s=(
                        self.last_angular_rad_s
                    ),
                )
            ),
            dynamic_limits=(
                self.dynamic_limits
            ),
            collision_geometry=(
                self.collision_geometry
            ),
            tracked_obstacles=(),
        )

        planned = plan_local_motion(
            request=request,
            config=self.planner_config,
        )

        self.last_plan = (
            planned
        )

        if (
                planned.result.status.value
                != "ok"
        ):
            self._direct_release_count = 0

            # --------------------------------------------------
            # Temporary FTG loss while already avoiding.
            #
            # With a narrow local FoV it is possible for the
            # currently observed field to contain no traversable
            # gap even though continuing the established avoidance
            # turn will reveal one.
            #
            # Do not translate blindly. Rotate in place toward the
            # already committed avoidance side until FTG can make
            # a normal decision again.
            # --------------------------------------------------

            if (
                    planned.result.status.value
                    == "no_gap"
                    and self.avoidance_engaged
                    and self.avoidance_side is not None
            ):
                recovery_angular_rad_s = (
                        min(
                            DEFAULT_NO_GAP_RECOVERY_ANGULAR_RAD_S,
                            self.dynamic_limits.angular_max_rad_s,
                        )
                        * self.avoidance_side
                )

                self.last_linear_mm_s = 0.0
                self.last_angular_rad_s = (
                    recovery_angular_rad_s
                )

                command = VelocityCommand(
                    linear_x_mps=0.0,
                    angular_z_rps=(
                        recovery_angular_rad_s
                    ),
                    lateral_y_mps=0.0,
                    timestamp=now_s,
                )

                self.velocity_backend.update(
                    command
                )

                self.reason = (
                    "no_gap_recovery"
                )

                return PrimitiveStatus.RUNNING

            if (
                planned.result.status.value == "no_gap"
                and self.avoidance_engaged
                and self.avoidance_side is None
            ):
                self.stop()

                self.reason = "no_gap_no_avoidance_side"

                self.status = PrimitiveStatus.FAILED

                print(
                    "[LOCAL_AVOIDANCE] "
                    "NO_GAP but no geometric avoidance side "
                    "can be established -> FAILED"
                )

                return self.status

            self.stop()

            self.reason = (
                planned.result.reason
            )

            return PrimitiveStatus.RUNNING

        selected_bearing_rad = float(
            planned.result.selected_bearing_rad
        )

        # --------------------------------------------------
        # Nominal-direction release
        #
        # FTG has finished its avoidance job once its selected
        # traversable direction has converged back onto the
        # nominal goal direction.
        #
        # DWA must not continue merely to restore the robot's
        # original heading or remove the lateral offset created
        # by the avoidance manoeuvre. The nominal controller
        # owns that work after handoff.
        # --------------------------------------------------

        direction_error_rad = math.atan2(
            math.sin(
                selected_bearing_rad
                - self.preferred_bearing_rad
            ),
            math.cos(
                selected_bearing_rad
                - self.preferred_bearing_rad
            ),
        )

        nominal_direction_available = (
                abs(direction_error_rad)
                <= DEFAULT_DIRECT_RELEASE_TOLERANCE_RAD
        )

        if (
                self.avoidance_engaged
                and nominal_direction_available
        ):
            self._direct_release_count += 1
        else:
            self._direct_release_count = 0

        if (
                self._direct_release_count
                >= DEFAULT_DIRECT_RELEASE_CONFIRMATIONS
                and handoff_corridor_clear
        ):
            self.stop()

            self.reason = (
                "handoff_corridor_clear"
            )

            self.status = (
                PrimitiveStatus.SUCCEEDED
            )

            blocker_lateral_text = (
                "none"
                if self.blocking_lateral_mm is None
                else (
                    f"{self.blocking_lateral_mm:.0f}mm"
                )
            )

            print(
                "[LOCAL_AVOIDANCE] "
                "handoff clear -> release "
                f"preferred="
                f"{math.degrees(self.preferred_bearing_rad):+.1f}deg "
                f"selected="
                f"{math.degrees(selected_bearing_rad):+.1f}deg "
                f"error="
                f"{math.degrees(direction_error_rad):+.1f}deg "
                f"blocker_lateral="
                f"{blocker_lateral_text}"
            )

            return self.status

        self.last_linear_mm_s = float(
            planned.result.linear_mm_s
        )

        self.last_angular_rad_s = float(
            planned.result.angular_rad_s
        )

        # Diagnostic only. Do not yet influence FTG.
        if (
            self.avoidance_side is None
            and abs(
                selected_bearing_rad
            ) > math.radians(1.0)
        ):
            self.avoidance_side = (
                1
                if selected_bearing_rad > 0.0
                else -1
            )

            print(
                "[LOCAL_AVOIDANCE] "
                "initial side="
                f"{'LEFT' if self.avoidance_side > 0 else 'RIGHT'} "
                f"selected="
                f"{math.degrees(selected_bearing_rad):+.1f}deg"
            )

        command = VelocityCommand(
            linear_x_mps=(
                self.last_linear_mm_s
                / 1000.0
            ),
            angular_z_rps=(
                self.last_angular_rad_s
            ),
            lateral_y_mps=0.0,
            timestamp=now_s,
        )

        self.velocity_backend.update(
            command
        )

        self.reason = (
            "avoiding"
        )

        return PrimitiveStatus.RUNNING