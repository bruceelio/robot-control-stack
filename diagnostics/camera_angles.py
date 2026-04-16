# diagnostics/camera_angles.py

import time

from behaviors.init_escape import InitEscape
from perception import Perception, sense
from localisation.localisation import Localisation
from level2.level2_canonical import Level2
from motion_backends import create_motion_backend
from config import CONFIG
from calibration.resolve import resolve as resolve_calibration
from hw_io.resolve import resolve_io


def run(robot):
    """
    Diagnostic: print camera marker orientation angles.

    Robot-agnostic:
    - Uses IOMap cameras instead of robot.camera
    - Builds lvl2/motion_backend/perception like Controller
    """

    print("\n=== CAMERA ANGLES DIAGNOSTIC ===")

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

    # --- Step 1: Init escape ---
    print("Running InitEscape...")
    behavior = InitEscape()
    behavior.start(config=CONFIG, motion_backend=motion_backend)

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
    time.sleep(CONFIG.camera_settle_time)

    # --- Step 3: sense & read camera ---
    pose, _objects = sense(io, perception)
    if pose is not None:
        x, y, heading = pose
        localisation.set_pose((x, y), heading)
    else:
        localisation.invalidate()

    cams = io.cameras()
    cam = cams.get("front")
    if cam is None:
        print("No front camera available in io.cameras()")
        return

    markers = cam.see()
    if not markers:
        print("No markers visible")
        return

    print(f"\nDetected {len(markers)} markers:\n")

    for m in markers:
        o = getattr(m, "orientation", None)
        if o is None:
            print(f"ID={getattr(m, 'id', '?')} | (no orientation field)")
            continue

        print(
            f"ID={m.id:3d} | "
            f"yaw={o.yaw:7.2f}° | "
            f"pitch={o.pitch:7.2f}° | "
            f"roll={o.roll:7.2f}°"
        )

    print("\n=== END CAMERA ANGLES DIAGNOSTIC ===")
