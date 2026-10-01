# tools/challenges/visual_servoing.py

from __future__ import annotations

import math
import time

from config import CONFIG

from perception import sense
from perception.robot_geometry import target_from_gripper

from navigation.control.distance_angle_controller import (
    ServoingController,
    ServoingStatus,
)

from behaviors.init_escape import InitEscape
from skills.perception.select_target import SelectTarget

from navigation.command.velocity_arbiter import (
    VelocityArbiter,
    VelocitySource,
)

from motion_backends.velocity import VelocityMotionBackend


# ==================================================
# Challenge configuration
# ==================================================

# Conservative first test.
# Stop well short of the target until the complete control chain
# has been proven in simulation.
STOP_DISTANCE_MM = 0.0

# Small delay between control updates.
LOOP_DELAY_S = 0.02

# Ultrasonic diagnostics are deliberately slower than the visual_servoing loop.
# Reading all four sensors every control cycle could disturb control timing.
ULTRASONIC_LOG_INTERVAL_S = 0.25

def _read_ultrasonics(io) -> dict[str, float | None]:
    readings = {}

    for name in ("front", "left", "right", "back"):
        try:
            if not CONFIG.has_io("ultrasonic", name):
                continue
        except KeyError:
            continue

        try:
            value = io.ultrasonic[name]
            readings[name] = None if value is None else float(value)
        except Exception:
            readings[name] = None

    return readings


def _format_mm(value: float | None) -> str:
    if value is None:
        return "n/a"

    return f"{value:.0f}mm"


# ==================================================
# Challenge
# ==================================================

def run(controller):
    """
    Servoing integration challenge.

    Tests:

        vision
          ↓
        perception
          ↓
        selected target
          ↓
        ServoingController
          ↓
        VelocityArbiter
          ↓
        VelocityMotionBackend
          ↓
        drivetrain

      The nearest visible target of CONFIG.default_target_kind is

    Servoing stops when:

        - the requested stop distance is reached; or
        - the selected target observation becomes stale.
    """

    print("\n=== SERVOING CHALLENGE ===")

    print("[SERVOING] running InitEscape")

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
                "InitEscape failed during visual_servoing challenge"
            )

        controller.io.sleep(LOOP_DELAY_S)

    print("[SERVOING] InitEscape complete")

    controller.io.sleep(
        CONFIG.camera_settle_time
    )

    if not CONFIG.servoing_enabled:
        raise RuntimeError(
            "Servoing is disabled for the selected robot profile."
        )



    servoing = ServoingController(
        application="approach_target",
        config=controller.config,
    )

    velocity_arbiter = VelocityArbiter()

    velocity_motion = VelocityMotionBackend(
        lvl2=controller.lvl2,
        config=controller.config,
        calibration=controller.calibration,
    )

    select_target = SelectTarget(
        max_age_s=CONFIG.visible_max_age_s,

        required_kind=None,
        preferred_kind=CONFIG.default_target_kind,
        preferred_elevation=CONFIG.default_target_elevation,

        marker_pitch_high_deg=CONFIG.marker_pitch_high_deg,
        marker_pitch_low_deg=CONFIG.marker_pitch_low_deg,

        timeout_s=None,
        label="SERVOING_SELECT_TARGET",
    )

    select_target.start()

    selected_id: int | None = None
    selected_kind: str | None = None

    last_ultrasonic_log_s = 0.0

    try:
        while True:

            now_s = time.time()

            # --------------------------------------------------
            # Vision
            # --------------------------------------------------

            vision_message = controller._get_vision_message(
                camera_name="front",
                now_s=now_s,
            )

            # --------------------------------------------------
            # Perception
            # --------------------------------------------------

            sense(
                controller.io,
                controller.perception,
                latest_vision_message=vision_message,
                stop_robot=False,
            )

            # --------------------------------------------------
            # Initial target selection
            # --------------------------------------------------

            if selected_id is None:

                selection_status = select_target.update(
                    perception=controller.perception,
                    now=now_s,
                )

                if selection_status.name != "SUCCEEDED":
                    velocity_motion.stop()

                    controller.io.sleep(
                        LOOP_DELAY_S
                    )
                    continue

                target = select_target.selected_target
                selected_id = select_target.selected_target_id
                selected_kind = select_target.selected_kind

                if (
                        target is None
                        or selected_id is None
                        or selected_kind is None
                ):
                    raise RuntimeError(
                        "SelectTarget succeeded without a complete target"
                    )

                print(
                    f"[SERVOING] selected "
                    f"{select_target.selected_elevation} "
                    f"{selected_kind} "
                    f"id={selected_id}"
                )

            # --------------------------------------------------
            # Retain selected target
            # --------------------------------------------------
            #
            # Do not re-select another target if visibility is lost.
            #
            # The object remains in perception memory long enough for
            # ServoingController to detect that its timestamp is stale.

            target = (
                controller.perception
                .objects
                .get(selected_kind, {})
                .get(selected_id)
            )

            if target is None:
                print(
                    f"[SERVOING] selected target "
                    f"id={selected_id} no longer in perception memory"
                )
                break

            distance_mm, bearing_deg = target_from_gripper(
                observation=target,
                config=controller.config,
            )

            target_angle_rad = math.radians(
                bearing_deg
            )

            timestamp = float(
                target["last_seen"]
            )

            # --------------------------------------------------
            # Ultrasonic diagnostics
            # --------------------------------------------------

            if (
                now_s - last_ultrasonic_log_s
                >= ULTRASONIC_LOG_INTERVAL_S
            ):
                ultrasonic = _read_ultrasonics(
                    controller.io
                )

                if ultrasonic:
                    front_mm = ultrasonic.get("front")

                    # Servoing distance is radial distance from the gripper
                    # to the selected target.  Its forward component is the
                    # more useful comparison with a forward-facing sensor.
                    target_forward_mm = (
                        distance_mm
                        * math.cos(target_angle_rad)
                    )

                    if front_mm is None:
                        front_delta_text = (
                            "front_minus_target=n/a"
                        )
                    else:
                        front_delta_text = (
                            f"front_minus_target="
                            f"{front_mm - target_forward_mm:+.0f}mm"
                        )

                    print(
                        f"[SERVOING][ULTRASONIC] "
                        f"front={_format_mm(front_mm)} "
                        f"left={_format_mm(ultrasonic.get('left'))} "
                        f"right={_format_mm(ultrasonic.get('right'))} "
                        f"back={_format_mm(ultrasonic.get('back'))} "
                        f"target_forward={target_forward_mm:.0f}mm "
                        f"{front_delta_text} "
                        f"target_angle={bearing_deg:+.1f}deg"
                    )

                last_ultrasonic_log_s = now_s

            # --------------------------------------------------
            # Servoing controller
            # --------------------------------------------------

            result = servoing.update(
                distance_mm=distance_mm,
                target_angle_rad=target_angle_rad,
                timestamp=timestamp,
                stop_distance_mm=STOP_DISTANCE_MM,
                now=now_s,
            )

            # --------------------------------------------------
            # Velocity arbitration
            # --------------------------------------------------

            velocity_command = velocity_arbiter.select(
                source=VelocitySource.SERVOING,
                servoing_command=result.command,
            )

            if velocity_command is None:
                raise RuntimeError(
                    "VelocityArbiter returned no command "
                    "for visual_servoing."
                )

            # --------------------------------------------------
            # Execute velocity command
            # --------------------------------------------------

            velocity_motion.update(
                velocity_command
            )

            print(
                f"[SERVOING] "
                f"id={selected_id} "
                f"camera_dist={float(target['distance']):.0f}mm "
                f"gripper_dist={distance_mm:.0f}mm "
                f"angle={math.degrees(target_angle_rad):+.1f}deg "
                f"v={velocity_command.linear_x_mps:.3f}m/s "
                f"w={velocity_command.angular_z_rps:+.3f}rad/s "
                f"status={result.status.value}"
            )

            # --------------------------------------------------
            # Completion / failure
            # --------------------------------------------------

            if result.status == ServoingStatus.SUCCESS:
                print(
                    f"[SERVOING] SUCCESS "
                    f"id={selected_id} "
                    f"distance={distance_mm:.0f}mm"
                )
                break

            if result.status == ServoingStatus.FAILED_STALE:
                print(
                    f"[SERVOING] FAILED_STALE "
                    f"id={selected_id}"
                )
                break

            controller.io.sleep(
                LOOP_DELAY_S
            )

    finally:
        # Absolutely guarantee drivetrain shutdown.
        velocity_motion.stop()

        print("[SERVOING] drivetrain stopped")

    print("=== SERVOING CHALLENGE COMPLETE ===\n")