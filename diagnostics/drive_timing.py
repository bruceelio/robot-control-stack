# diagnostics/drive_timing.py

import time

from behaviors.init_escape import InitEscape
from navigation.localisation import Localisation
from primitives.motion import Drive
from primitives.base import PrimitiveStatus
from motion_backends import create_motion_backend
from config import CONFIG


# Distances to test (mm)
DISTANCES_MM = [
    100, 200, 400, 600,
    1000, 1500,
]


def run(robot):
    print("\n=== DRIVE CALIBRATION DIAGNOSTIC ===\n")

    localisation = Localisation()

    motion_backend = create_motion_backend(
        CONFIG.motion_backend,
        robot.lvl2,
        CONFIG,
        None,   # calibration already resolved in controller normally
    )

    # -------------------------------------------------
    # Optional: InitEscape for repeatable starting pose
    # -------------------------------------------------
    print("[DIAG] Running InitEscape...")
    escape = InitEscape()
    escape.start(motion_backend=motion_backend)

    while True:
        status = escape.update(
            lvl2=robot.lvl2,
            localisation=localisation,
            motion_backend=motion_backend,
        )
        if status.name == "SUCCEEDED":
            break
        time.sleep(0.02)

    print("[DIAG] InitEscape complete\n")
    time.sleep(0.5)

    # -------------------------------------------------
    # Drive tests
    # -------------------------------------------------
    test_distances = []
    for d in DISTANCES_MM:
        test_distances.append(d)
        test_distances.append(-d)

    print("cmd_mm, actual_mm, error_mm")

    for distance in test_distances:
        # Capture pose before
        pose_before = localisation.get_pose()
        x0, y0 = pose_before.x_mm, pose_before.y_mm

        drive = Drive(distance_mm=distance)
        drive.start(motion_backend=motion_backend)

        # Wait for drive to complete
        while True:
            status = drive.update(motion_backend=motion_backend)
            if status == PrimitiveStatus.SUCCEEDED:
                break
            if status == PrimitiveStatus.FAILED:
                print(f"[ERROR] Drive FAILED at distance {distance}")
                return
            time.sleep(0.01)

        time.sleep(0.2)  # settle

        pose_after = localisation.get_pose()
        x1, y1 = pose_after.x_mm, pose_after.y_mm

        # Euclidean distance traveled
        dx = x1 - x0
        dy = y1 - y0
        actual = (dx ** 2 + dy ** 2) ** 0.5

        # Preserve sign (project onto forward axis)
        sign = 1 if distance >= 0 else -1
        actual *= sign

        error = actual - distance

        print(
            f"{distance:7.1f}, "
            f"{actual:7.1f}, "
            f"{error:7.1f}"
        )

        time.sleep(0.5)

    print("\n=== END DRIVE CALIBRATION ===")
