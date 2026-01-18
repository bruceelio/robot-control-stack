# diagnostics/rotation_calibration.py

import time
import math

from behaviors.init_escape import InitEscape
from perception import Perception
from level2.level2_canonical import Level2
from motion_backends import create_motion_backend
from calibration import CALIBRATION
from config import CONFIG


# --------------------------------------------------
# Helpers
# --------------------------------------------------

def normalize_angle_deg(angle):
    """Normalize angle to [-180, +180]."""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def get_reference_bearing(robot):
    """
    Returns horizontal bearing (deg) to the closest BASIC marker.

    Uses SR 2026 AprilCamera API:
      - position.distance (mm)
      - position.horizontal_angle (rad)
    """
    markers = robot.camera.see()
    basics = [m for m in markers if m.id >= 140]

    if not basics:
        return None

    ref = min(basics, key=lambda m: m.position.distance)
    return math.degrees(ref.position.horizontal_angle)


# --------------------------------------------------
# Diagnostic entrypoint
# --------------------------------------------------

def run(robot):
    """
    Diagnostic: rotation calibration harness.

    Measures actual robot rotation vs commanded angle
    using camera-relative marker bearings.
    """

    print("\n=== ROTATION CALIBRATION DIAGNOSTIC ===")

    # --------------------------------------------------
    # Core subsystems (MATCH camera_angles)
    # --------------------------------------------------

    lvl2 = Level2(
        robot,
        max_power=CONFIG.max_motor_power
    )

    perception = Perception()

    motion_backend = create_motion_backend(
        CONFIG.motion_backend,
        lvl2
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
    behavior.start(
        config=CONFIG,
        motion_backend=motion_backend
    )

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

    # --------------------------------------------------
    # Test parameters
    # --------------------------------------------------

    ANGLES = [10.0, 20.0, 30.0, 45.0]
    DIRECTIONS = [
        ("CW", +1.0),
        ("CCW", -1.0),
    ]

    print("\n[ROT-CAL] Beginning measurements...\n")

    # --------------------------------------------------
    # Main test loop
    # --------------------------------------------------

    for angle in ANGLES:
        for label, direction in DIRECTIONS:

            b0 = get_reference_bearing(robot)
            if b0 is None:
                print("[ROT-CAL][WARN] no reference marker before rotation")
                time.sleep(0.5)
                continue

            cmd_angle = direction * angle

            print(
                f"[ROT-CAL][CMD] "
                f"angle={cmd_angle:+.1f}deg dir={label}"
            )

            motion_backend.rotate(cmd_angle)

            while motion_backend.is_busy():
                time.sleep(0.01)

            time.sleep(CONFIG.camera_settle_time)

            b1 = get_reference_bearing(robot)
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

    # --------------------------------------------------
    # Cleanup
    # --------------------------------------------------

    motion_backend.stop()
    print("\n[ROT-CAL] complete")
