# diagnostics/camera_angles.py

import time
from behaviors.init_escape import InitEscape
from perception import Perception, sense
from navigation.localisation import Localisation
from level2_canonical import Level2
from motion_backends import create_motion_backend
from config.base import CAMERA_SETTLE_TIME, MOTION_BACKEND


def run(robot):
    """
    Diagnostic: print camera marker orientation angles.

    Purpose:
    - Observe pitch/yaw/roll for markers at different heights
    - Used to infer camera height / elevation differences (simulation only)
    """

    print("\n=== CAMERA ANGLES DIAGNOSTIC ===")

    # --- Core subsystems ---
    lvl2 = Level2(robot)
    perception = Perception()
    localisation = Localisation()

    motion_backend = create_motion_backend(
        MOTION_BACKEND,
        lvl2
    )

    # --- Step 1: Init escape ---
    print("Running InitEscape...")
    behavior = InitEscape()
    behavior.start(motion_backend=motion_backend)

    while True:
        status = behavior.update(
            lvl2=lvl2,
            localisation=localisation,
            motion_backend=motion_backend,
        )
        if status.name == "SUCCEEDED":
            break
        time.sleep(0.02)

    print("InitEscape complete")

    # --- Step 2: let camera settle ---
    time.sleep(CAMERA_SETTLE_TIME)

    # --- Step 3: read camera ---
    pose, objects = sense(robot, perception)
    markers = robot.camera.see()

    if not markers:
        print("No markers visible")
        return

    print(f"\nDetected {len(markers)} markers:\n")

    for marker in markers:
        o = marker.orientation
        print(
            f"ID={marker.id:3d} | "
            f"yaw={o.yaw:7.2f}° | "
            f"pitch={o.pitch:7.2f}° | "
            f"roll={o.roll:7.2f}°"
        )

    print("\n=== END CAMERA ANGLES DIAGNOSTIC ===")
