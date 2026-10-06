# tools/challenges/local_planning.py

from __future__ import annotations

import math
import time

from autonomous.SR2026.init_escape import InitEscape

from perception import (
    get_visible_targets,
    sense,
)

from perception.providers.local_obstacle_field import (
    local_obstacle_field_from_objects,
)

from perception.robot_geometry import (
    relative_target_from_base_link,
)

from primitives.base import PrimitiveStatus

from skills.navigation.local_avoidance import (
    LocalAvoidance,
    LocalObstacleObservation,
)


# ==================================================
# Challenge configuration
# ==================================================

LOOP_DELAY_S = 0.05
LOG_INTERVAL_S = 0.20
MAX_RUN_TIME_S = 20.0

TARGET_KIND_OVERRIDE: str | None = None

# Fixed target for repeatable avoidance tuning.
TARGET_OBJECT_ID: int | None = 101

# SR2026 cube physical radius.
OBJECT_RADIUS_MM = 65.0


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


def _front_camera_fov_rad(
    controller,
) -> float:
    try:
        fov_deg = float(
            controller
            .calibration
            .cameras["front"]
            .meta
            .fov_deg
        )

    except Exception as exc:
        raise RuntimeError(
            "Local-planning challenge requires "
            "front-camera FOV calibration."
        ) from exc

    if fov_deg <= 0.0:
        raise RuntimeError(
            f"Invalid front-camera FOV: {fov_deg}"
        )

    return math.radians(
        fov_deg
    )


def _vision_bearing_to_planner_rad(
    bearing_deg: float,
) -> float:
    """
    Camera/perception:
        positive = image right

    Local planning:
        positive = left / counter-clockwise
    """

    return -math.radians(
        float(bearing_deg)
    )


def _visible_objects(
    perception,
    *,
    max_age_s: float,
):
    """
    Perception currently timestamps visibility using time.time(),
    so let get_visible_targets remain in its own clock domain.
    """

    objects = []

    for kind in (
        "acidic",
        "basic",
    ):
        objects.extend(
            get_visible_targets(
                perception,
                kind,
                max_age_s=max_age_s,
            )
        )

    return objects


def _obstacle_observations(
    *,
    config,
    visible_objects,
    target_id: int,
) -> tuple[
    LocalObstacleObservation,
    ...
]:
    """
    Temporary identity-bearing obstacle observations used by
    LocalAvoidance for short-term blocker persistence and handoff.

    The planner obstacle field itself now comes from perception.
    """

    observations = []

    for obj in visible_objects:
        try:
            object_id = int(
                obj["id"]
            )

            target = (
                relative_target_from_base_link(
                    observation=obj,
                    config=config,
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

        if object_id == target_id:
            continue

        if distance_mm <= 0.0:
            continue

        observations.append(
            LocalObstacleObservation(
                obstacle_id=object_id,
                distance_mm=distance_mm,
                bearing_rad=bearing_rad,
                radius_mm=OBJECT_RADIUS_MM,
            )
        )

    return tuple(
        observations
    )


def _run_init_escape(
    controller,
) -> None:
    print(
        "[LOCAL_PLAN] InitEscape "
        f"drive="
        f"{controller.config.init_escape_drive_mm}mm "
        f"rotate="
        f"{controller.config.init_escape_rotate_deg:+.1f}deg"
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
            return

        if status.name == "FAILED":
            raise RuntimeError(
                "InitEscape failed during "
                "local-planning challenge"
            )

        controller.io.sleep(
            LOOP_DELAY_S
        )


# ==================================================
# Challenge
# ==================================================

def run(
    controller,
):
    """
    Thin LocalAvoidance tuning harness.

    The production LocalAvoidance skill owns:

        - local goal propagation
        - blocker persistence
        - FTG
        - DWA
        - velocity output
        - corridor-clear release

    The challenge owns only:

        - InitEscape test geometry
        - fixed target selection
        - perception updates
        - observation adaptation
        - diagnostics
    """

    print(
        "\n=== LOCAL AVOIDANCE CHALLENGE: FTG + DWA ==="
    )

    config = (
        controller.config
    )

    _run_init_escape(
        controller
    )

    controller.io.sleep(
        float(
            _cfg(
                config,
                "camera_settle_time",
                0.25,
            )
        )
    )

    target_kind = (
        TARGET_KIND_OVERRIDE
        if TARGET_KIND_OVERRIDE
        is not None
        else config.default_target_kind
    )

    field_fov_rad = (
        _front_camera_fov_rad(
            controller
        )
    )

    selected_id = None
    avoidance = None

    start_s = (
        controller.io.time()
    )

    next_log_s = (
        start_s
    )

    try:
        while True:
            loop_s = (
                controller.io.time()
            )

            if (
                loop_s
                - start_s
                >= MAX_RUN_TIME_S
            ):
                print(
                    "[LOCAL_PLAN] test timeout"
                )
                break

            # ------------------------------------------
            # Vision / perception
            # ------------------------------------------

            # Existing vision/perception stack currently
            # timestamps its object memory with wall time.
            perception_now_s = (
                time.time()
            )

            vision_message = (
                controller._get_vision_message(
                    camera_name="front",
                    now_s=perception_now_s,
                )
            )

            sense(
                controller.io,
                controller.perception,
                latest_vision_message=(
                    vision_message
                ),
                stop_robot=False,
            )

            visible_objects = (
                _visible_objects(
                    controller.perception,
                    max_age_s=(
                        config.visible_max_age_s
                    ),
                )
            )

            # ------------------------------------------
            # Fixed target selection
            # ------------------------------------------


            if selected_id is None:
                candidates = [
                    obj
                    for obj in get_visible_targets(
                        controller.perception,
                        target_kind,
                        max_age_s=config.visible_max_age_s,
                    )
                    if (
                            TARGET_OBJECT_ID is None
                            or int(obj["id"]) == TARGET_OBJECT_ID
                    )
                ]

                if not candidates:
                    if loop_s >= next_log_s:
                        print(
                            "[LOCAL_PLAN] "
                            "waiting for target "
                            f"kind={target_kind} "
                            f"id={TARGET_OBJECT_ID}"
                        )

                        next_log_s = (
                            loop_s
                            + LOG_INTERVAL_S
                        )

                    controller.io.sleep(
                        LOOP_DELAY_S
                    )
                    continue

                target = min(
                    candidates,
                    key=lambda item: float(
                        item["distance"]
                    ),
                )

                selected_id = int(
                    target["id"]
                )

                print(
                    "[LOCAL_PLAN] "
                    "selected target "
                    f"id={selected_id} "
                    f"kind={target_kind}"
                )

                avoidance = LocalAvoidance(
                    config=config,
                    field_fov_rad=(
                        field_fov_rad
                    ),
                )

                avoidance.start(
                    lvl2=controller.lvl2,
                    calibration=(
                        controller.calibration
                    ),
                    now_s=loop_s,
                )

            # ------------------------------------------
            # Current target observation
            # ------------------------------------------

            visible_target = None

            for obj in visible_objects:
                try:
                    object_id = int(
                        obj["id"]
                    )

                except (
                    KeyError,
                    TypeError,
                    ValueError,
                ):
                    continue

                if object_id == selected_id:
                    visible_target = obj
                    break

            if visible_target is None:
                goal_distance_mm = None
                goal_bearing_rad = None

            else:
                goal_distance_mm = float(
                    visible_target["distance"]
                )

                goal_bearing_rad = (
                    _vision_bearing_to_planner_rad(
                        float(
                            visible_target[
                                "bearing"
                            ]
                        )
                    )
                )

            # ------------------------------------------
            # Current local obstacle observations
            # ------------------------------------------

            obstacles = (
                _obstacle_observations(
                    config=config,
                    visible_objects=(
                        visible_objects
                    ),
                    target_id=(
                        selected_id
                    ),
                )
            )

            obstacle_range_limit_mm = (
                goal_distance_mm
                if goal_distance_mm is not None
                else avoidance.goal_distance_mm
            )

            obstacle_field = (
                local_obstacle_field_from_objects(
                    config=config,
                    visible_objects=(
                        visible_objects
                    ),
                    field_fov_rad=(
                        field_fov_rad
                    ),
                    scan_sectors=(
                        avoidance.scan_sectors
                    ),
                    clear_range_mm=(
                            avoidance.lookahead_distance_mm
                            * 1.5
                    ),
                    timestamp_s=loop_s,
                    exclude_ids=(
                        selected_id,
                    ),
                    default_obstacle_radius_mm=(
                        OBJECT_RADIUS_MM
                    ),
                    max_obstacle_distance_mm=(
                        obstacle_range_limit_mm
                    ),
                )
            )

            # ------------------------------------------
            # Production local avoidance
            # ------------------------------------------

            status = avoidance.update(
                now_s=loop_s,
                goal_distance_mm=(
                    goal_distance_mm
                ),
                goal_bearing_rad=(
                    goal_bearing_rad
                ),
                obstacle_observations=(
                    obstacles
                ),
                obstacle_field=(
                    obstacle_field
                ),
            )

            # ------------------------------------------
            # Diagnostics
            # ------------------------------------------

            if loop_s >= next_log_s:
                plan = avoidance.last_plan

                if (
                        plan is None
                        or plan.result.selected_gap is None
                ):
                    gap_text = "none"
                else:
                    gap = plan.result.selected_gap

                    gap_text = (
                        f"["
                        f"{math.degrees(gap.start_bearing_rad):+.1f},"
                        f"{math.degrees(gap.end_bearing_rad):+.1f}"
                        f"]deg"
                    )

                if (
                        plan is None
                        or plan.result.selected_bearing_rad is None
                ):
                    selected_text = "none"
                else:
                    selected_text = (
                        f"{math.degrees(plan.result.selected_bearing_rad):+.1f}deg"
                    )

                if avoidance.blocking_obstacle_id is None:
                    blocker_text = "none"
                else:
                    blocker_text = (
                        f"{avoidance.blocking_obstacle_id}"
                        f"@{avoidance.blocking_lateral_mm:.0f}mm"
                    )

                if avoidance.avoidance_side is None:
                    side_text = "none"
                elif avoidance.avoidance_side > 0:
                    side_text = "LEFT"
                else:
                    side_text = "RIGHT"

                if avoidance.goal_distance_mm is None:
                    goal_dist_text = "none"
                else:
                    goal_dist_text = (
                        f"{avoidance.goal_distance_mm:.0f}mm"
                    )

                if avoidance.goal_bearing_rad is None:
                    goal_bearing_text = "none"
                else:
                    goal_bearing_text = (
                        f"{math.degrees(avoidance.goal_bearing_rad):+.1f}deg"
                    )

                print(
                    "[LOCAL_PLAN] "
                    f"id={selected_id} "
                    f"visible={visible_target is not None} "
                    f"goal_dist={goal_dist_text} "
                    f"goal_bearing={goal_bearing_text} "
                    f"gap={gap_text} "
                    f"selected={selected_text} "
                    f"v={avoidance.last_linear_mm_s:.0f}mm/s "
                    f"w={avoidance.last_angular_rad_s:+.3f}rad/s "
                    f"blocker={blocker_text} "
                    f"side={side_text} "
                    f"reason={avoidance.reason}"
                )

                next_log_s = (
                        loop_s
                        + LOG_INTERVAL_S
                )

            # ------------------------------------------
            # Challenge boundary
            # ------------------------------------------

            if (
                status
                == PrimitiveStatus.SUCCEEDED
            ):
                if avoidance.avoidance_engaged:
                    print(
                        "[LOCAL_PLAN] "
                        "HANDOFF_READY "
                        f"id={selected_id} "
                        "direct nominal corridor clear"
                    )

                else:
                    print(
                        "[LOCAL_PLAN] "
                        "NO_AVOIDANCE_REQUIRED "
                        f"id={selected_id}"
                    )

                break

            if (
                status
                == PrimitiveStatus.FAILED
            ):
                print(
                    "[LOCAL_PLAN] FAILED "
                    f"reason={avoidance.reason}"
                )

                break

            controller.io.sleep(
                LOOP_DELAY_S
            )

    finally:
        if avoidance is not None:
            avoidance.stop()