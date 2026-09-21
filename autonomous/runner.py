# autonomous/runner.py

from config.strategy import AutonomousProgram


def run_autonomous(autonomous_program, controller):
    if autonomous_program == AutonomousProgram.AUTO_SR2026_STAGE1:
        from autonomous.auto_sr2026_stage1 import run
        return run(controller)

    raise RuntimeError(
        f"Unsupported autonomous program: {autonomous_program}"
    )
