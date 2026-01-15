"""
Diagnostics execution framework.

Diagnostics are active measurement routines.
They may move the robot and read sensors.

They run ONCE and then exit.
"""

def run_diagnostics(robot):
    from diagnostics.camera_angles import run
    run(robot)
