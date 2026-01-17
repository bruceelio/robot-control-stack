# diagnostics/rotation_timing.py

import time
import math

from behaviors.init_escape import InitEscape
from navigation.localisation import Localisation
from primitives.motion import Rotate
from primitives.base import PrimitiveStatus
from motion_backends import create_motion_backend
from config import CONFIG


ANGLES_DEG = [
    22.5, 45.0, 67.5,
    90.0, 180.0, 270.0, 360.0,
]


def normalize_deg(angle):
    """Normalize angle to [-180, 180)."""
    while angle >= 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def run(robot):
    print("\n=== ROTATION CALIBRATION DIAGNOSTIC ===\n")

    localisation = Localisation()
    motion_backend = create_motion_backend(
        CONFIG.motion_backend,
        None,  # Level2 not required for Rotate
    )

    # -------------------------------------------------
    # Optional: InitEscape for repeatable starting pose
    # -------------------------------------------------
    print("[DIAG] Running InitEscape...")
    escape = InitEscape()
    escape.start(motion_backend=motion_backend)

    while True:
        status = escape.update(
            lvl2=None,
            localisation=localisation,
            motion_backend=motion_backend,
        )
        if status.name == "SUCCEEDED":
            break
        time.sleep(0.02)

    print("[DIAG] InitEscape complete\n")
    time.sleep(0.5)

    # -------------------------------------------------
    # Rotation tests
    # -------------------------------------------------
    test_angles = []
    for a in ANGLES_DEG:
        test_angles.append(a)
        test_angles.append(-a)

    print("cmd_deg, actual_deg, error_deg")

    for angle in test_angles:
        # Capture yaw before
        pose_before = localisation.get_pose()
        yaw_before = pose_before.yaw_deg

        rotate = Rotate(angle_deg=angle)
        rotate.start(motion_backend=motion_backend)

        # Wait for rotation to complete
        while True:
            status = rotate.update(motion_backend=motion_backend)
            if status == PrimitiveStatus.SUCCEEDED:
                break
            if status == PrimitiveStatus.FAILED:
                print(f"[ERROR] Rotate FAILED at angle {angle}")
                return
            time.sleep(0.01)

        time.sleep(0.2)  # settle

        pose_after = localisation.get_pose()
        yaw_after = pose_after.yaw_deg

        delta = normalize_deg(yaw_after - yaw_before)
        error = delta - angle

        print(
            f"{angle:6.1f}, "
            f"{delta:6.1f}, "
            f"{error:6.1f}"
        )

        time.sleep(0.5)

    print("\n=== END ROTATION CALIBRATION ===")
