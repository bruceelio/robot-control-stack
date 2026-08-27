# diagnostics/drive_timing.py

from __future__ import annotations

import csv
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

from config import CONFIG
from hw_io.resolve import resolve_io


DRIVE_GROUP = "front"

LEFT_DRIVE_MOTOR = "drive_front_left"
RIGHT_DRIVE_MOTOR = "drive_front_right"

RESULTS_PATH = Path("diagnostics/results/drive_timing.csv")

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
    "duration_commanded_s",
    "duration_powered_s",
    "battery_before_v",
    "battery_after_v",
    "distance_measured_mm",
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


def _prompt_direction() -> tuple[str, float]:
    while True:
        raw = input("Direction [F/R]: ").strip().lower()

        if raw in {"f", "forward"}:
            return "forward", 1.0

        if raw in {"r", "reverse", "backward"}:
            return "reverse", -1.0

        print("Enter F for forward or R for reverse.")


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


def _stop_drive(drive) -> None:
    """Stop both front drive motors with one paired command."""
    drive.set_power(
        left=0.0,
        right=0.0,
    )


def run(robot=None) -> None:
    print("\n=== PHYSICAL DRIVE TIMING DIAGNOSTIC ===")
    print("Measures physical distance produced by chosen motor power and duration.")
    print("This diagnostic bypasses the Level2 software interface.")
    print("Keep the test lane clear and be ready to stop the robot.\n")

    io = resolve_io(
        robot=robot,
    )

    # Paired semantic drive device ensures left and right outputs are transmitted
    # together in one Pi-to-Mega command.
    drive = io.drive[DRIVE_GROUP]

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
                print("Reusing previous drive settings:")
                print(f"  Direction: {direction}")
                print(f"  Power:     {power_magnitude:.3f}")
                print(f"  Duration:  {duration_commanded_s:.3f} s")
            else:
                direction, direction_sign = _prompt_direction()

                power_magnitude = _prompt_float(
                    f"Motor power magnitude [0.0 to {max_power:.2f}]: ",
                    minimum=0.0,
                    maximum=max_power,
                )

                duration_commanded_s = _prompt_float(
                    "Drive duration (seconds): ",
                    minimum=0.01,
                )

            signed_power = direction_sign * power_magnitude
            left_power = signed_power
            right_power = signed_power

            print("\nPosition and align the robot at the starting mark.")
            print(
                f"Next command: {direction} at power {power_magnitude:.3f} "
                f"for {duration_commanded_s:.3f} s"
            )

            confirmation = input(
                "Press Enter to run, or type Q then Enter to quit: "
            ).strip().lower()

            if confirmation in {"q", "quit"}:
                break

            battery_before_v = _read_battery_voltage(io)
            print(f"Battery before: {_format_voltage(battery_before_v)}")
            print("DRIVING...")

            powered_started_s = time.monotonic()

            try:
                drive.set_power(
                    left=left_power,
                    right=right_power,
                )

                # Use checkout sleep so the selected backend can continue any
                # required heartbeat or service behaviour while driving.
                io.sleep(duration_commanded_s)

            except Exception:
                print("\n[ERROR] Direct checkout drive command failed.")
                print("Run tests/test_motion.py to troubleshoot motor operation.")
                raise


            finally:

                powered_stopped_s = time.monotonic()

                try:

                    _stop_drive(drive)

                except Exception:

                    print("\n[ERROR] Paired drive stop command failed.")

                    raise

            duration_powered_s = powered_stopped_s - powered_started_s

            print("Drive complete.")
            io.sleep(0.25)

            battery_after_v = _read_battery_voltage(io)
            print(f"Battery after:  {_format_voltage(battery_after_v)}")

            distance_measured_mm = _prompt_float(
                "Measured distance travelled (mm): ",
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
                "duration_commanded_s": f"{duration_commanded_s:.6f}",
                "duration_powered_s": f"{duration_powered_s:.6f}",
                "battery_before_v": (
                    "" if battery_before_v is None else f"{battery_before_v:.6f}"
                ),
                "battery_after_v": (
                    "" if battery_after_v is None else f"{battery_after_v:.6f}"
                ),
                "distance_measured_mm": f"{distance_measured_mm:.3f}",
                "notes": notes,
            }

            _append_result(row)

            print(
                f"Saved trial {trial}: {direction}, "
                f"power={power_magnitude:.3f}, "
                f"time={duration_commanded_s:.3f} s, "
                f"distance={distance_measured_mm:.1f} mm"
            )
            print(f"Results file: {RESULTS_PATH}")

            trial += 1

            if not _prompt_yes_no("Run another trial?", default=True):
                break

            print("\nPrevious drive settings:")
            print(f"  Direction: {direction}")
            print(f"  Power:     {power_magnitude:.3f}")
            print(f"  Duration:  {duration_commanded_s:.3f} s")

            reuse_previous_settings = _prompt_yes_no(
                "Reuse previous drive settings?",
                default=True,
            )

    finally:
        # Final safety stop for normal exit, Ctrl+C, or an unexpected error.
        try:
            _stop_drive(drive)
        except Exception as exc:
            print(f"[WARN] Final paired drive stop failed: {exc}")

    print("\n=== END PHYSICAL DRIVE TIMING DIAGNOSTIC ===")


if __name__ == "__main__":
    run(robot=None)