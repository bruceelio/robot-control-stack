# diagnostics/runner.py

"""
Diagnostics execution framework.

Diagnostics are active measurement routines.
They may move the robot and read sensors.

They run ONCE and then exit.
"""

from diagnostics.registry import DIAGNOSTICS


def run_diagnostics(robot):
    """
    Execute all registered diagnostics.

    Each diagnostic:
      - Receives the robot instance
      - Runs once
      - Prints or logs results
    """
    if not DIAGNOSTICS:
        print("No diagnostics registered.")
        return

    print(f"Running {len(DIAGNOSTICS)} diagnostics...\n")

    for diag in DIAGNOSTICS:
        name = diag.__name__

        print(f"--- Diagnostic: {name} ---")
        try:
            diag(robot)
        except Exception as e:
            print(f"[ERROR] Diagnostic {name} failed: {e}")

        print(f"--- End {name} ---\n")

    print("All diagnostics complete.")
