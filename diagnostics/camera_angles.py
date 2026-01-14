# diagnostics/camera_angles.py

from diagnostics.registry import register_diagnostic
import time


@register_diagnostic
def camera_marker_angles(robot):
    """
    Print orientation angles for visible markers.
    """
    time.sleep(0.5)  # allow camera to settle

    markers = robot.camera.see()

    if not markers:
        print("No markers visible.")
        return

    for m in markers:
        print(
            f"ID={m.id} "
            f"yaw={m.orientation.yaw:.2f} "
            f"pitch={m.orientation.pitch:.2f} "
            f"roll={m.orientation.roll:.2f}"
        )
