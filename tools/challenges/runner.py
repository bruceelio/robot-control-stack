# tools/challenges/runner.py

from config.strategy import Challenge


def run_challenge(challenge, controller):
    if challenge == Challenge.VISION:
        from tools.challenges import run
        return run(controller)

    if challenge == Challenge.MOVEMENT:
        from tools.challenges.sr2026.movement import run
        return run(controller)

    if challenge == Challenge.MECHANICS:
        from tools.challenges.sr2026.mechanics import run
        return run(controller)

    if challenge == Challenge.SENSING:
        from tools.challenges import run
        return run(controller)

    if challenge == Challenge.SIMULATOR:
        from tools.challenges import run
        return run(controller)

    if challenge == Challenge.TRANSPORTATION:
        from tools.challenges import run
        return run(controller)

    if challenge == Challenge.STOPPING:
        from tools.challenges import run
        return run(controller)

    if challenge == Challenge.SERVOING:
        from tools.challenges.servoing import run
        return run(controller)

    if challenge == Challenge.SERVOING_ARENA_TAG:
        from tools.challenges.servoing_arena_tag import run
        return run(controller)

    if challenge == Challenge.SERVOING_RETURN_TO_BASE:
        from tools.challenges.servoing_return_to_base import run
        return run(controller)

    if challenge == Challenge.SERVOING_POSE:
        from tools.challenges.servoing_pose import run
        return run(controller)

    if challenge == Challenge.DRIVE_CALIBRATION:
        from tools.challenges.drive_calibration import run
        return run(controller)

    if challenge == Challenge.ROTATE_CALIBRATION:
        from tools.challenges.rotate_calibration import run
        return run(controller)

    if challenge == Challenge.WALL_FOLLOWING:
        from tools.challenges.wall_following import run
        return run(controller)

    if challenge == Challenge.LIFT_ULTRASONIC_SWEEP:
        from tools.challenges.lift_ultrasonic_sweep import run
        return run(controller)

    if challenge == Challenge.LIFT_VISION_SWEEP:
        from tools.challenges.lift_vision_sweep import run
        return run(controller)

    if challenge == Challenge.LOCAL_PLANNING:
        from tools.challenges.local_planning import run
        return run(controller)

    raise RuntimeError(f"Unsupported challenge: {challenge}")