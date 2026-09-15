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

    raise RuntimeError(f"Unsupported challenge: {challenge}")