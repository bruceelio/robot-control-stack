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

    backend_names = required_backends(CONFIG.io)

    print(
        f"[IO RESOLVE] required backends={backend_names}"
    )

    if not backend_names:
        raise RuntimeError(
            "Robot configuration does not define any IO backends"
        )

    # --------------------------------------------------
    # Build requested backends
    # --------------------------------------------------

    resolved = {}

    for backend_name in backend_names:

        if backend_name == "sr2026sim":
            from hw_io.hw_sr2026sim import SR2026SimIO

            resolved[backend_name] = SR2026SimIO(robot)
            continue

        if backend_name == "sr2026":
            from hw_io.hw_sr2026 import SR2026IO

            resolved[backend_name] = SR2026IO(robot)
            continue

        if backend_name == "mega2560":
            from hw_io.hw_mega2560 import Mega2560IO

            resolved[backend_name] = Mega2560IO(
                robot,
                camera_manager=camera_manager,
            )
            continue

        if backend_name == "mega2560alt":
            from hw_io.hw_mega2560alt import Mega2560AltIO

            resolved[backend_name] = Mega2560AltIO(
                robot,
                camera_manager=camera_manager,
            )
            continue

        if backend_name == "pi":
            from hw_io.hw_pi import PiBackend

            resolved[backend_name] = PiBackend(
                robot=robot,
                camera_manager=camera_manager,
            )
            continue

        raise RuntimeError(
            f"No IO backend implementation for "
            f"{backend_name!r}"
        )

    # --------------------------------------------------
    # Preserve existing single-backend behaviour
    # --------------------------------------------------

    if len(resolved) == 1:
        backend_name = backend_names[0]

        # PiBackend is deliberately only a partial backend,
        # so it must still be wrapped if ever used alone.
        if backend_name != "pi":
            return resolved[backend_name]

    # --------------------------------------------------
    # Multiple backends
    # --------------------------------------------------

    from hw_io.composite import CompositeIO

    print(
        f"[IO RESOLVE] composing backends="
        f"{backend_names}"
    )

    return CompositeIO(
        backends=resolved,
        io_config=CONFIG.io,
    )