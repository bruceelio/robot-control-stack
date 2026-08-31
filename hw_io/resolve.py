# hw_io/resolve.py

from hw_io.base import IOMap


def required_backends(io_config: dict[str, str | None]) -> list[str]:
    return sorted({
        backend
        for backend in io_config.values()
        if backend is not None
    })


def resolve_io(*, robot, camera_manager=None) -> IOMap:

    from config import CONFIG

    backends = required_backends(CONFIG.io)
    print(f"[IO RESOLVE] required backends={backends}")

    # Temporary limitation during migration:
    # currently each robot must resolve to one backend.
    if len(backends) != 1:
        raise RuntimeError(
            f"Expected exactly one IO backend, got {backends}"
        )

    backend = backends[0]

    # Student Robotics 2026 Webots simulation.
    # Uses the SR robot3 API, but has simulator-specific hardware mappings.
    if backend == "sr2026sim":
        from hw_io.hw_sr2026sim import SR2026SimIO
        return SR2026SimIO(robot)

    # Physical Student Robotics 2026 hardware.
    if backend == "sr2026":
        from hw_io.hw_sr2026 import SR2026IO
        return SR2026IO(robot)

    if backend == "mega2560":
        from hw_io.hw_mega2560 import Mega2560IO
        return Mega2560IO(
            robot,
            camera_manager=camera_manager,
        )

    if backend == "mega2560alt":
        from hw_io.hw_mega2560alt import Mega2560AltIO
        return Mega2560AltIO(
            robot,
            camera_manager=camera_manager,
        )

    raise RuntimeError(
        f"No IOMap implementation for backend={backend!r}"
    )