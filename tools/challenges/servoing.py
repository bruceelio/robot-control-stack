# tools/challenges/servoing.py

from __future__ import annotations

import math
import time

from config import CONFIG

from perception import (
    sense,
    get_visible_targets,
)
from perception.robot_geometry import target_from_gripper

from navigation.servoing.servoing_controller import (
    ServoingController,
    ServoingStatus,
)

from behaviors.init_escape import InitEscape
from skills.navigation.approach_target import _marker_elevation

from navigation.velocity_arbiter import (
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
    selected once and then retained for the duration of the test.

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
                "InitEscape failed during servoing challenge"
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

    target_kind = CONFIG.default_target_kind

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

    selected_id: int | None = None

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

                targets = get_visible_targets(
                    controller.perception,
                    target_kind,
                    now=now_s,
                    max_age_s=CONFIG.visible_max_age_s,
                )

                if not targets:
                    velocity_motion.stop()

                    print(
                        f"[SERVOING] waiting for visible "
                        f"{target_kind} target"
                    )

                    controller.io.sleep(LOOP_DELAY_S)
                    continue

                wanted_elevation = CONFIG.default_target_elevation

                matching_targets = []

                for candidate in targets:
                    pitch, src = _marker_elevation(
                        candidate.get("marker", candidate),
                    )

                    if src == "none":
                        continue

                    if (
                            wanted_elevation == "high"
                            and pitch <= float(CONFIG.marker_pitch_high_deg)
                    ):
                        matching_targets.append(candidate)

                    elif (
                            wanted_elevation == "low"
                            and pitch >= float(CONFIG.marker_pitch_low_deg)
                    ):
                        matching_targets.append(candidate)

                if not matching_targets:
                    velocity_motion.stop()

                    print(
                        f"[SERVOING] waiting for visible "
                        f"{wanted_elevation} {target_kind} target"
                    )

                    controller.io.sleep(LOOP_DELAY_S)
                    continue

                target = min(
                    matching_targets,
                    key=lambda t: float(t["distance"]),
                )

                selected_id = int(target["id"])

                print(
                    f"[SERVOING] selected "
                    f"{CONFIG.default_target_elevation} "
                    f"{target_kind} id={selected_id}"
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
                .get(target_kind, {})
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
                    "for servoing."
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