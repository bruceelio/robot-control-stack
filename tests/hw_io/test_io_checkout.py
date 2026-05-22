"""Registered pre-competition IO checkout.

Run through the existing tests.runner entry point.
"""

from tests.registry import register_test
from hw_io.resolve import resolve_io
from config import CONFIG
from tests.hw_io.io_runner import run_io_checkout


def _io(robot):
    from hw_io.cameras.camera_process import CameraProcessManager

    camera_manager = CameraProcessManager(
        camera_names=list(CONFIG.cameras.keys()),
        robot=robot,
    )

    io = resolve_io(
        robot=robot,
        hardware_profile=CONFIG.hardware_profile,
        camera_manager=camera_manager,
    )

    camera_manager.start()
    return io, camera_manager


def _csv_path():
    # Prefer explicit config, but keep a safe default for first bring-up.
    return getattr(CONFIG, "io_map_csv", "tests/hw_io/maps/Bob_Bot.csv")


@register_test(
    name="test_precomp_io_checkout",
    category="io",
    enabled=True,
    requires_robot=True,
)
def test_precomp_io_checkout(robot):
    io, camera_manager = _io(robot)
    try:
        return run_io_checkout(io, _csv_path())
    finally:
        camera_manager.stop()

if __name__ == "__main__":
    print("=== Running IO Checkout (direct mode) ===")

    io, camera_manager = _io(robot=None)

    try:
        run_io_checkout(io, _csv_path())
    finally:
        camera_manager.stop()