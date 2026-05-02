"""Registered pre-competition IO checkout.

Run through the existing tests.runner entry point.
"""

from tests.registry import register_test
from hw_io.resolve import resolve_io
from config import CONFIG
from tests.hw_io.io_runner import run_io_checkout


def _io(robot):
    return resolve_io(robot=robot, hardware_profile=CONFIG.hardware_profile)


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
    io = _io(robot)
    return run_io_checkout(io, _csv_path())

if __name__ == "__main__":
    from hw_io.resolve import resolve_io
    from config import CONFIG

    print("=== Running IO Checkout (direct mode) ===")

    # resolve IO the same way your tests do
    io = resolve_io(robot=None, hardware_profile=CONFIG.hardware_profile)

    # run checkout
    from tests.hw_io.io_runner import run_io_checkout
    run_io_checkout(io, _csv_path())