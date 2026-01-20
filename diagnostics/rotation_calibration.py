# diagnostics/rotation_calibration.py

import time
import math

from behaviors.init_escape import InitEscape
from perception import Perception, sense
from level2.level2_canonical import Level2
from motion_backends import create_motion_backend
from calibration import CALIBRATION
from calibration.resolve import resolve as resolve_calibration
from config import CONFIG
from hw_io.resolve import resolve_io


def normalize_angle_deg(angle):
    """Normalize angle to [-180, +180]."""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def get_reference_bearing(io):
    """
    Returns horizontal bearing (deg) to the closest BASIC marker.

    Assumes the camera wrapper returns markers compatible with SR April markers:
      - m.id
      - m.position.distance (mm)
      - m.position.horizontal_angle (rad)
    """
    cams = io.cameras()
    cam = cams.get("front")
    if cam is None:
        return None

    markers = cam.see()
    basics = [m for m in markers if getattr(m, "id", -1) >= 140]
    if not basics:
        return None

    ref = min(basics, key=lambda m: m.position.distance)
    return math.degrees(ref.position.horizontal_angle)


def run(robot):
    """
    Diagnostic: rotation calibration harness.

    Measures actual robot rotation vs commanded angle
    using camera-relative marker bearings.
    """

    print("\n=== ROTATION CALIBRATION DIAGNOSTIC ===")

    # --- Core subsystems (match Controller) ---
    io = resolve_io(robot=robot, hardware_profile=CONFIG.hardware_profile)
    lvl2 = Level2(io, max_power=CONFIG.max_motor_power)
    perception = Perception(io)

    calibration = resolve_calibration(config=CONFIG)

    motion_backend = create_motion_backend(
        CONFIG.motion_backend,
        lvl2,
        CONFIG,
        calibration,
    )

    # --------------------------------------------------
    # Calibration snapshot
    # --------------------------------------------------
    print("\n[ROT-CAL][CFG]")
    print(
        f" switch={CALIBRATION.rotate_switch_deg}deg\n"
        f" SMALL: power={CALIBRATION.rotate_power_small:.2f} "
        f"m={CALIBRATION.rotate_m_small:.5f} "
        f"b={CALIBRATION.rotate_b_small:.3f}\n"
        f" LARGE: power={CALIBRATION.rotate_power_large:.2f} "
        f"m={CALIBRATION.rotate_m_large:.5f} "
        f"b={CALIBRATION.rotate_b_large:.3f}"
    )

    # --------------------------------------------------
    # Init escape
    # --------------------------------------------------
    print("\n[ROT-CAL] Running InitEscape...")
    behavior = InitEscape()
    behavior.start(config=CONFIG, motion_backend=motion_backend)

    while True:
        status = behavior.update(
            lvl2=lvl2,
            localisation=None,
            motion_backend=motion_backend,
        )
        if status.name == "SUCCEEDED":
            break
        time.sleep(0.02)

    print("[ROT-CAL] InitEscape complete")
    time.sleep(CONFIG.camera_settle_time)

    # Prime perception once (optional, but keeps logs consistent)
    sense(io, perception)

    # --------------------------------------------------
    # Test parameters
    # --------------------------------------------------
    ANGLES = [10.0, 20.0, 30.0, 45.0]
    DIRECTIONS = [("CW", +1.0), ("CCW", -1.0)]

    print("\n[ROT-CAL] Beginning measurements...\n")

    for angle in ANGLES:
        for label, direction in DIRECTIONS:
            b0 = get_reference_bearing(io)
            if b0 is None:
                print("[ROT-CAL][WARN] no reference marker before rotation")
                time.sleep(0.5)
                continue

            cmd_angle = direction * angle
            print(f"[ROT-CAL][CMD] angle={cmd_angle:+.1f}deg dir={label}")

            motion_backend.rotate(cmd_angle)
            while motion_backend.is_busy():
                time.sleep(0.01)

            time.sleep(CONFIG.camera_settle_time)

            b1 = get_reference_bearing(io)
            if b1 is None:
                print("[ROT-CAL][WARN] no reference marker after rotation")
                time.sleep(0.5)
                continue

            # Marker motion is opposite of robot motion
            marker_delta = normalize_angle_deg(b1 - b0)
            actual = normalize_angle_deg(-marker_delta)
            error = actual - cmd_angle

            print(
                "[ROT-CAL][MEAS] "
                f"cmd={cmd_angle:+.1f}deg "
                f"actual={actual:+.1f}deg "
                f"err={error:+.1f}deg "
                f"(marker_delta={marker_delta:+.1f}deg) "
                f"dir={label}"
            )

            time.sleep(1.0)

    motion_backend.stop()
    print("\n[ROT-CAL] complete")
