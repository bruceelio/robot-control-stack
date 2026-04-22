# diagnostics/runner.py

"""
Diagnostics execution framework.

Diagnostics are active measurement routines.
They may move the robot and read sensors.

They run ONCE and then exit.
"""

def run_diagnostics(robot, io):
    # from diagnostics.camera_angles import run
    # from diagnostics.rotation_calibration import run
    # from diagnostics.drive_timing import run
    # from diagnostics.rotation_timing import run
    # from diagnostics.marker_pitches import run
    # from diagnostics.camera_only import run
    from diagnostics.apriltag_pose_check import run

    run(robot=robot, io=io)
