# diagnostics/drive_timing.py

import time
import math

from primitives.motion import Drive
from primitives.base import PrimitiveStatus
from behaviors.init_escape import InitEscape
from navigation.localisation import Localisation
from perception import Perception, sense

from motion_backends import create_motion_backend
from level2.level2_canonical import Level2
from config import CONFIG
from calibration.resolve import resolve as resolve_calibration
from hw_io.resolve import resolve_io


# Distances to test (mm)
DISTANCES_MM = [100, 200, 400, 600, 1000, 1500]


def _update_localisation(io, perception, localisation):
    pose, _objects = sense(io, perception)
    if pose is None:
        localisation.invalidate()
        return None
    x, y, heading = pose
    localisation.set_pose((x, y), heading)
    return (x, y, heading)


def run(robot):
    print("\n=== DRIVE CALIBRATION DIAGNOSTIC ===\n")

    # --- Core subsystems (match Controller) ---
    io = resolve_io(robot=robot, hardware_profile=CONFIG.hardware_profile)
    lvl2 = Level2(io, max_power=CONFIG.max_motor_power)
    perception = Perception(io)
    localisation = Localisation()

    calibration = resolve_calibration(config=CONFIG)

    motion_backend = create_motion_backend(
        CONFIG.motion_backend,
        lvl2,
        CONFIG,
        calibration,
    )

    # -------------------------------------------------
    # Optional: InitEscape for repeatable starting pose
    # -------------------------------------------------
    print("[DIAG] Running InitEscape...")
    escape = InitEscape()
    escape.start(config=CONFIG, motion_backend=motion_backend)

    while True:
        status = escape.update(
            lvl2=lvl2,
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

    print("cmd_mm\tactual_mm\terror_mm")

    for distance in test_distances:
        # Pose before
        p0 = _update_localisation(io, perception, localisation)
        if p0 is None:
            print("[ERROR] No pose before drive (vision lost?)")
            return
        x0, y0, _h0 = p0

        drive = Drive(distance_mm=distance)
        drive.start(motion_backend=motion_backend)

        while True:
            status = drive.update(motion_backend=motion_backend)
            if status == PrimitiveStatus.SUCCEEDED:
                break
            if status == PrimitiveStatus.FAILED:
                print(f"[ERROR] Drive FAILED at distance {distance}")
                return
            time.sleep(0.01)

        time.sleep(0.2)  # settle

        # Pose after
        p1 = _update_localisation(io, perception, localisation)
        if p1 is None:
            print("[ERROR] No pose after drive (vision lost?)")
            return
        x1, y1, _h1 = p1

        # Euclidean distance travelled
        dx = x1 - x0
        dy = y1 - y0
        actual = math.sqrt(dx * dx + dy * dy)

        # Preserve sign based on commanded direction
        if distance < 0:
            actual *= -1

        error = actual - distance

        print(f"{distance:.1f}\t{actual:.1f}\t{error:.1f}")
        time.sleep(0.5)

    print("\n=== END DRIVE CALIBRATION ===")
