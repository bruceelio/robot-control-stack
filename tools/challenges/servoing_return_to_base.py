# tools/challenges/servoing_return_to_base.py

from __future__ import annotations

import time

from perception import sense
from primitives.base import PrimitiveStatus
from primitives.manipulation import LiftDown
from skills.navigation.return_to_base_servo import ReturnToBaseServo


GUIDE_IDS = (17, 18, 19)
LOOP_DELAY_S = 0.02


def run(controller):
    """
    Challenge harness for ReturnToBase Stage 2.

    Test setup:
        - lower lift

    Production code under test:
        - ReturnToBaseServo
        - guide sequence 17 -> 18 -> 19
    """

    print("\n=== SERVOING RETURN TO BASE CHALLENGE ===")

    # --------------------------------------------------
    # Test setup only: lower lift
    # --------------------------------------------------

    print("[RETURN_BASE_CHALLENGE] lowering lift")

    lift_down = LiftDown(
        settle_time=0.5,
    )

    lift_down.start(
        lvl2=controller.lvl2,
    )

    while (
        lift_down.update()
        == PrimitiveStatus.RUNNING
    ):
        controller.io.sleep(0.05)

    print("[RETURN_BASE_CHALLENGE] lift down")

    # --------------------------------------------------
    # Production Stage 2
    # --------------------------------------------------

    return_to_base = ReturnToBaseServo(
        config=controller.config,
        guide_ids=GUIDE_IDS,
    )

    return_to_base.start(
        lvl2=controller.lvl2,
    )

    try:
        while True:
            now_s = time.time()

            # ------------------------------------------
            # Vision
            # ------------------------------------------

            vision_message = controller._get_vision_message(
                camera_name="front",
                now_s=now_s,
            )

            # ------------------------------------------
            # Perception
            # ------------------------------------------

            arena_observations, _ = sense(
                controller.io,
                controller.perception,
                latest_vision_message=vision_message,
                stop_robot=False,
            )

            if vision_message is not None:
                observation_timestamp = float(
                    vision_message["timestamp"]
                )
            else:
                observation_timestamp = now_s

            # ------------------------------------------
            # ReturnToBase Stage 2
            # ------------------------------------------

            status = return_to_base.update(
                arena_observations=arena_observations,
                observation_timestamp=observation_timestamp,
            )

            if status == PrimitiveStatus.SUCCEEDED:
                print(
                    "[RETURN_BASE_CHALLENGE] "
                    "Stage 2 SUCCEEDED"
                )
                break

            if status == PrimitiveStatus.FAILED:
                raise RuntimeError(
                    "ReturnToBase Stage 2 FAILED"
                )

            controller.io.sleep(
                LOOP_DELAY_S
            )

    finally:
        return_to_base.stop()

    print(
        "=== SERVOING RETURN TO BASE "
        "CHALLENGE COMPLETE ===\n"
    )