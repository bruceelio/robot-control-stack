# tools/challenges/wall_following.py

from __future__ import annotations

import time
import math

from config import CONFIG
from perception.providers.acquisition import (
    acquire_wall_geometry,
)
from navigation.wall_following.models import WallSide
from skills.navigation.follow_wall import FollowWall
from autonomous.SR2026.init_escape import InitEscape


# ==================================================
# Challenge configuration
# ==================================================

WALL_SIDE = WallSide.LEFT

# None means:
# latch the first valid side range and try to maintain it.
DESIRED_DISTANCE_MM: float | None = 300.0

# Conservative first moving test.
LINEAR_X_MPS = 0.50

# Short first run.
RUN_TIME_S = 6.0

LOOP_DELAY_S = 0.05

# Challenge-level safety guard.
FRONT_STOP_MM = 350.0


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


def run(controller):
    """
    First wall-following integration challenge.

    Current test path:

        side ultrasonic
            ↓
        WallGeometry(distance only)
            ↓
        FollowWall
            ↓
        WallFollower
            ↓
        DistanceOnlyWallFollower
            ↓
        VelocityMotionBackend
            ↓
        drivetrain

    The challenge deliberately does not estimate wall heading.
    """

    print("\n=== WALL FOLLOWING CHALLENGE ===")

    # --------------------------------------------------
    # Move clear of the starting wall and rotate
    # --------------------------------------------------

    print("[WALL_FOLLOW] running InitEscape")

    escape = InitEscape()
    escape.start(
        config=CONFIG,
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
                "InitEscape failed during wall-following challenge"
            )

        controller.io.sleep(LOOP_DELAY_S)

    print("[WALL_FOLLOW] InitEscape complete")

    controller.io.sleep(0.5)

    sensor_key = WALL_SIDE.value

    print(
        "[WALL_FOLLOW] "
        f"side={WALL_SIDE.value} "
        f"sensor=ultrasonic.{sensor_key}"
    )

    follow_wall = None

    start_s = time.monotonic()
    next_log_s = start_s

    try:
        while (
            time.monotonic() - start_s
            < RUN_TIME_S
        ):
            now_s = time.monotonic()

            wall_geometry = acquire_wall_geometry(
                config=controller.config,
                io=controller.io,
                wall_side=WALL_SIDE,
            )

            front_mm = _read_range_mm(
                controller.io,
                "front",
            )

            # ------------------------------------------
            # Front obstacle safety
            # ------------------------------------------

            if (
                front_mm is not None
                and front_mm <= FRONT_STOP_MM
            ):
                print(
                    "[WALL_FOLLOW] "
                    f"front={front_mm:.0f}mm "
                    "-> stop"
                )

                if follow_wall is not None:
                    follow_wall.stop()

                break

            # ------------------------------------------
            # No usable side-wall measurement
            # ------------------------------------------

            if not wall_geometry.has_distance:
                if follow_wall is not None:
                    follow_wall.update(
                        wall=wall_geometry,
                    )

                if now_s >= next_log_s:
                    print(
                        "[WALL_FOLLOW] "
                        f"side={WALL_SIDE.value} "
                        "wall_distance=NONE "
                        f"front={front_mm}"
                    )
                    next_log_s = now_s + 0.25

                controller.io.sleep(
                    LOOP_DELAY_S
                )
                continue

            side_mm = float(
                wall_geometry.distance_mm
            )

            # ------------------------------------------
            # Latch initial desired wall distance
            # ------------------------------------------

            if follow_wall is None:
                desired_distance_mm = (
                    side_mm
                    if DESIRED_DISTANCE_MM is None
                    else DESIRED_DISTANCE_MM
                )

                follow_wall = FollowWall(
                    config=controller.config,
                    wall_side=WALL_SIDE,
                    desired_distance_mm=(
                        desired_distance_mm
                    ),
                    linear_x_mps=LINEAR_X_MPS,
                )

                follow_wall.start(
                    lvl2=controller.lvl2,
                )

                print(
                    "[WALL_FOLLOW] "
                    f"initial={side_mm:.0f}mm "
                    f"desired={desired_distance_mm:.0f}mm"
                )

            # ------------------------------------------
            # Feed semantic wall geometry to skill
            # ------------------------------------------

            follow_wall.update(
                wall=wall_geometry,
            )

            # ------------------------------------------
            # Diagnostics
            # ------------------------------------------

            if now_s >= next_log_s:
                mode = follow_wall.active_mode
                result = follow_wall.last_result
                if result is not None:
                    wall_heading_text = (
                        "NONE"
                        if not wall_geometry.has_heading
                        else (
                            f"{math.degrees(wall_geometry.heading_rad):+.1f}deg"
                        )
                    )
                    print(
                        "[WALL_FOLLOW] "
                        f"{sensor_key}={side_mm:.0f}mm "
                        f"wall_heading={wall_heading_text} "
                        f"err={result.distance_error_mm:+.0f}mm "
                        f"rate_raw={result.distance_rate_mm_s:+.0f}mm/s "
                        f"rate_filt={result.distance_rate_filtered_mm_s:+.0f}mm/s "
                        f"P={result.distance_proportional_angular_z_rps:+.3f} "
                        f"heading_ctrl={result.heading_angular_z_rps:+.3f} "
                        f"heading_est="
                        f"{'NONE' if result.inferred_heading_error_rad is None else f'{math.degrees(result.inferred_heading_error_rad):+.1f}deg'} "
                        f"raw={result.unclamped_angular_z_rps:+.3f} "
                        f"cmd={result.command.angular_z_rps:+.3f} "
                        f"front="
                        f"{'NONE' if front_mm is None else f'{front_mm:.0f}mm'} "
                        f"mode={follow_wall.active_mode.value}"
                    )
                else:
                    print(
                        "[WALL_FOLLOW] "
                        f"{sensor_key}={side_mm:.0f}mm "
                        f"front="
                        f"{'NONE' if front_mm is None else f'{front_mm:.0f}mm'} "
                        f"mode=NONE"
                    )

                next_log_s = now_s + 0.25

            controller.io.sleep(
                LOOP_DELAY_S
            )

    finally:
        if follow_wall is not None:
            follow_wall.stop()

    print("[WALL_FOLLOW] challenge complete")