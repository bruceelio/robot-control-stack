# tools/challenges/servoing_arena_tag.py

from __future__ import annotations

import math
import time

from navigation.servoing.servoing_controller import (
    ServoingController,
    ServoingStatus,
)
from navigation.velocity_arbiter import (
    VelocityArbiter,
    VelocitySource,
)
from motion_backends.velocity import VelocityMotionBackend


TARGET_TAG_ID = 0
STOP_DISTANCE_MM = 500.0
LOOP_DELAY_S = 0.02


def run(controller):
    print("\n=== ARENA TAG SERVOING CHALLENGE ===")

    servoing = ServoingController()
    velocity_arbiter = VelocityArbiter()

    velocity_motion = VelocityMotionBackend(
        lvl2=controller.lvl2,
        config=controller.config,
        calibration=controller.calibration,
    )

    last_detection = None
    last_timestamp = None

    try:
        while True:
            now_s = time.time()

            vision_message = controller._get_vision_message(
                camera_name="front",
                now_s=now_s,
            )

            # ------------------------------------------
            # Look for our chosen arena tag
            # ------------------------------------------

            if vision_message is not None:
                for detection in vision_message.get("detections", []):
                    if int(detection["id"]) == TARGET_TAG_ID:
                        last_detection = detection
                        last_timestamp = float(
                            vision_message["timestamp"]
                        )
                        break

            # ------------------------------------------
            # Haven't seen the target yet
            # ------------------------------------------

            if last_detection is None:
                velocity_motion.stop()

                print(
                    f"[ARENA SERVO] waiting for "
                    f"arena tag id={TARGET_TAG_ID}"
                )

                controller.io.sleep(LOOP_DELAY_S)
                continue

            # ------------------------------------------
            # Existing visual servo controller
            # ------------------------------------------

            distance_mm = float(
                last_detection["distance_mm"]
            )

            target_angle_rad = math.radians(
                float(last_detection["bearing_deg"])
            )

            result = servoing.update(
                distance_mm=distance_mm,
                target_angle_rad=target_angle_rad,
                timestamp=last_timestamp,
                stop_distance_mm=STOP_DISTANCE_MM,
                now=now_s,
            )

            velocity_command = velocity_arbiter.select(
                source=VelocitySource.SERVOING,
                servoing_command=result.command,
            )

            if velocity_command is None:
                raise RuntimeError(
                    "VelocityArbiter returned no command."
                )

            velocity_motion.update(velocity_command)

            print(
                f"[ARENA SERVO] "
                f"id={TARGET_TAG_ID} "
                f"dist={distance_mm:.0f}mm "
                f"angle={math.degrees(target_angle_rad):+.1f}deg "
                f"v={velocity_command.linear_x_mps:.3f}m/s "
                f"w={velocity_command.angular_z_rps:+.3f}rad/s "
                f"status={result.status.value}"
            )

            if result.status == ServoingStatus.SUCCESS:
                print(
                    f"[ARENA SERVO] SUCCESS "
                    f"id={TARGET_TAG_ID} "
                    f"distance={distance_mm:.0f}mm"
                )
                break

            if result.status == ServoingStatus.FAILED_STALE:
                print(
                    f"[ARENA SERVO] FAILED_STALE "
                    f"id={TARGET_TAG_ID}"
                )
                break

            controller.io.sleep(LOOP_DELAY_S)

    finally:
        velocity_motion.stop()
        print("[ARENA SERVO] drivetrain stopped")

    print("=== ARENA TAG SERVOING CHALLENGE COMPLETE ===\n")