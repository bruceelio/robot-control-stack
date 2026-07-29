# diagnostics/rotation_timing.py

from __future__ import annotations

import csv
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

from config import CONFIG
from hw_io.resolve import resolve_io


LEFT_DRIVE_MOTOR = "drive_front_left"
RIGHT_DRIVE_MOTOR = "drive_front_right"

RESULTS_PATH = Path("diagnostics/results/rotation_timing.csv")

CSV_FIELDS = [
    "timestamp",
    "robot_id",
    "hardware_profile",
    "trial",
    "direction",
    "left_motor",
    "right_motor",
    "left_power",
    "right_power",
    "power_magnitude",
    "duration_commanded_s",
    "duration_powered_s",
    "battery_before_v",
    "battery_after_v",
    "rotation_measured_deg",
    "notes",
]


def _prompt_float(
    prompt: str,
    *,
    minimum: Optional[float] = None,
    maximum: Optional[float] = None,
) -> float:
    while True:
        raw = input(prompt).strip()

        try:
            value = float(raw)
        except ValueError:
            print("Enter a numeric value.")
            continue

        if minimum is not None and value < minimum:
            print(f"Value must be at least {minimum}.")
            continue

        if maximum is not None and value > maximum:
            print(f"Value must not exceed {maximum}.")
            continue

        return value


def _prompt_direction() -> tuple[str, float, float]:
    """
    Return the requested robot rotation direction and motor direction signs.

    Positive motor power is assumed to drive each wheel forwards through the
    semantic hw_io interface.
    """
    while True:
        raw = input("Rotation direction [L/R]: ").strip().lower()

        if raw in {"l", "left", "ccw", "counterclockwise", "anticlockwise"}:
            return "left", -1.0, 1.0

        if raw in {"r", "right", "cw", "clockwise"}:
            return "right", 1.0, -1.0

        print("Enter L for left/anticlockwise or R for right/clockwise.")


def _prompt_yes_no(prompt: str, *, default: bool = False) -> bool:
    suffix = " [Y/n]: " if default else " [y/N]: "

    while True:
        raw = input(prompt + suffix).strip().lower()

        if raw == "":
            return default

        if raw in {"y", "yes"}:
            return True

        if raw in {"n", "no"}:
            return False

        print("Enter Y or N.")


def _read_battery_voltage(io) -> Optional[float]:
    try:
        value = io.voltage["battery"].volts
    except (AttributeError, KeyError, TypeError):
        return None

    if value is None:
        return None

    try:
        value = float(value)
    except (TypeError, ValueError):
        return None

    if value <= 1.0:
        return None

    return value


def _format_voltage(value: Optional[float]) -> str:
    return "unavailable" if value is None else f"{value:.2f} V"


def _append_result(row: dict) -> None:
    RESULTS_PATH.parent.mkdir(parents=True, exist_ok=True)
    write_header = not RESULTS_PATH.exists() or RESULTS_PATH.stat().st_size == 0

    with RESULTS_PATH.open("a", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)

        if write_header:
            writer.writeheader()

        writer.writerow(row)


def _stop_drive(left_motor, right_motor) -> None:
    """Stop both named drive motors through the semantic hw_io interface."""
    first_error: Optional[Exception] = None

    try:
        left_motor.power = 0.0
    except Exception as exc:
        first_error = exc

    try:
        right_motor.power = 0.0
    except Exception:
        if first_error is None:
            raise

    if first_error is not None:
        raise first_error


def run(robot=None) -> None:
    print("\n=== PHYSICAL ROTATION TIMING DIAGNOSTIC ===")
    print("Measures physical rotation produced by chosen motor power and duration.")
    print("This diagnostic bypasses the Level2 software interface.")
    print("Clear the robot's turning area and be ready to stop the robot.\n")

    io = resolve_io(
        robot=robot,
        hardware_profile=CONFIG.hardware_profile,
    )

    # Named semantic devices are the final application-facing hw_io boundary.
    # The resolver maps these names to the selected physical or simulated backend.
    left_motor = io.motor[LEFT_DRIVE_MOTOR]
    right_motor = io.motor[RIGHT_DRIVE_MOTOR]

    max_power = float(getattr(CONFIG, "max_motor_power", 1.0))

    print(
        f"Maximum motor power: {max_power:.2f}\n"
        f"Source: CONFIG.max_motor_power for "
        f"{getattr(CONFIG, 'robot_id', 'selected robot')}"
    )

    trial = 1
    reuse_previous_settings = False

    try:
        while True:
            print(f"\n--- Trial {trial} ---")

            if reuse_previous_settings:
                print("Reusing previous rotation settings:")
                print(f"  Direction: {direction}")
                print(f"  Power:     {power_magnitude:.3f}")
                print(f"  Duration:  {duration_commanded_s:.3f} s")
            else:
                direction, left_sign, right_sign = _prompt_direction()

                power_magnitude = _prompt_float(
                    f"Motor power magnitude [0.0 to {max_power:.2f}]: ",
                    minimum=0.0,
                    maximum=max_power,
                )

                duration_commanded_s = _prompt_float(
                    "Rotation duration (seconds): ",
                    minimum=0.01,
                )

            left_power = left_sign * power_magnitude
            right_power = right_sign * power_magnitude

            print("\nPosition and align the robot with the starting reference.")
            print(
                f"Next command: rotate {direction} at power "
                f"{power_magnitude:.3f} for {duration_commanded_s:.3f} s"
            )
            print(
                f"Motor commands: left={left_power:+.3f}, "
                f"right={right_power:+.3f}"
            )

            confirmation = input(
                "Press Enter to run, or type Q then Enter to quit: "
            ).strip().lower()

            if confirmation in {"q", "quit"}:
                break

            battery_before_v = _read_battery_voltage(io)
            print(f"Battery before: {_format_voltage(battery_before_v)}")
            print("ROTATING...")

            powered_started_s = time.monotonic()

            try:
                left_motor.power = left_power
                right_motor.power = right_power

                # Use hw_io sleep so the selected backend can continue any
                # required heartbeat or service behaviour while rotating.
                io.sleep(duration_commanded_s)

            except Exception:
                print("\n[ERROR] Direct hw_io rotation command failed.")
                print("Run tests/test_motion.py to troubleshoot motor operation.")
                raise

            finally:
                powered_stopped_s = time.monotonic()

                try:
                    _stop_drive(left_motor, right_motor)
                except Exception:
                    print("\n[ERROR] One or both motor stop commands failed.")
                    raise

            duration_powered_s = powered_stopped_s - powered_started_s

            print("Rotation complete.")
            io.sleep(0.25)

            battery_after_v = _read_battery_voltage(io)
            print(f"Battery after:  {_format_voltage(battery_after_v)}")

            rotation_measured_deg = _prompt_float(
                "Measured rotation magnitude (degrees): ",
                minimum=0.0,
            )

            notes = input("Notes (optional): ").strip()

            row = {
                "timestamp": datetime.now().astimezone().isoformat(
                    timespec="seconds"
                ),
                "robot_id": getattr(CONFIG, "robot_id", ""),
                "hardware_profile": getattr(CONFIG, "hardware_profile", ""),
                "trial": trial,
                "direction": direction,
                "left_motor": LEFT_DRIVE_MOTOR,
                "right_motor": RIGHT_DRIVE_MOTOR,
                "left_power": f"{left_power:.6f}",
                "right_power": f"{right_power:.6f}",
                "power_magnitude": f"{power_magnitude:.6f}",
                "duration_commanded_s": f"{duration_commanded_s:.6f}",
                "duration_powered_s": f"{duration_powered_s:.6f}",
                "battery_before_v": (
                    "" if battery_before_v is None else f"{battery_before_v:.6f}"
                ),
                "battery_after_v": (
                    "" if battery_after_v is None else f"{battery_after_v:.6f}"
                ),
                "rotation_measured_deg": f"{rotation_measured_deg:.3f}",
                "notes": notes,
            }

            _append_result(row)

            print(
                f"Saved trial {trial}: {direction}, "
                f"power={power_magnitude:.3f}, "
                f"time={duration_commanded_s:.3f} s, "
                f"rotation={rotation_measured_deg:.1f} deg"
            )
            print(f"Results file: {RESULTS_PATH}")

            trial += 1

            if not _prompt_yes_no("Run another trial?", default=True):
                break

            print("\nPrevious rotation settings:")
            print(f"  Direction: {direction}")
            print(f"  Power:     {power_magnitude:.3f}")
            print(f"  Duration:  {duration_commanded_s:.3f} s")

            reuse_previous_settings = _prompt_yes_no(
                "Reuse previous rotation settings?",
                default=True,
            )

    finally:
        # Final safety stop for normal exit, Ctrl+C, or an unexpected error.
        try:
            _stop_drive(left_motor, right_motor)
        except Exception as exc:
            print(f"[WARN] Final motor stop failed: {exc}")

    print("\n=== END PHYSICAL ROTATION TIMING DIAGNOSTIC ===")


if __name__ == "__main__":
    run(robot=None)