from config.strategy import Challenge


def run_challenge(challenge, controller):
    if challenge == Challenge.VISION:
        from challenges.vision import run
        return run(controller)

    if challenge == Challenge.MOVEMENT:
        from challenges.movement import run
        return run(controller)

    if challenge == Challenge.MECHANICS:
        from challenges.mechanics import run
        return run(controller)

    if challenge == Challenge.SENSING:
        from challenges.sensing import run
        return run(controller)

    if challenge == Challenge.SIMULATOR:
        from challenges.simulator import run
        return run(controller)

    if challenge == Challenge.TRANSPORTATION:
        from challenges.transportation import run
        return run(controller)

    if challenge == Challenge.STOPPING:
        from challenges.stopping import run
        return run(controller)

    raise RuntimeError(f"Unsupported challenge: {challenge}")