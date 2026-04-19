#tools/calibrate_drive_raw.py

# python3 tools/calibrate_drive_raw.py --power 0.20 --durations 0.5 1.0 1.5 2.0

from __future__ import annotations

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse
import csv
import time

from hw_io.resolve import resolve_io


def append_csv_row(csv_path: Path, row: dict[str, object]) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    exists = csv_path.exists()
    with csv_path.open("a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if not exists:
            writer.writeheader()
        writer.writerow(row)


def prompt_float(prompt: str) -> float:
    while True:
        raw = input(prompt).strip()
        try:
            return float(raw)
        except ValueError:
            print("Please enter a valid number.")


def stop_motors(io_map) -> None:
    io_map.motors[0].power = 0.0
    io_map.motors[1].power = 0.0


def rearm_auto(io_map) -> None:
    if hasattr(io_map, "ensure_auto_mode"):
        try:
            io_map.ensure_auto_mode(force=True)
            return
        except TypeError:
            pass

        # fallback for older bob_bot.py
        if hasattr(io_map, "_auto_entered"):
            io_map._auto_entered = False
        io_map.ensure_auto_mode()


def run_drive(io_map, left_power: float, right_power: float, duration_s: float) -> None:
    try:
        rearm_auto(io_map)
        io_map.motors[0].power = left_power
        io_map.motors[1].power = right_power
        io_map.sleep(duration_s)
    finally:
        stop_motors(io_map)


def main() -> int:
    parser = argparse.ArgumentParser(description="Raw timed drive calibration for BobBot.")
    parser.add_argument("--hardware-profile", default="bob_bot")
    parser.add_argument("--power", type=float, default=0.20, help="Symmetric drive power.")
    parser.add_argument("--left-power", type=float, help="Override left motor power.")
    parser.add_argument("--right-power", type=float, help="Override right motor power.")
    parser.add_argument(
        "--durations",
        type=float,
        nargs="+",
        required=True,
        help="Durations to test, e.g. --durations 0.5 1.0 1.5 2.0",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path("logs/calibration_drive_raw.csv"),
        help="CSV log file.",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=1.0,
        help="Settle time after each run.",
    )
    parser.add_argument(
        "--label",
        default="drive_raw",
        help="Free-text label recorded in CSV.",
    )
    args = parser.parse_args()

    left_power = args.left_power if args.left_power is not None else args.power
    right_power = args.right_power if args.right_power is not None else args.power

    io_map = resolve_io(robot=None, hardware_profile=args.hardware_profile)

    print(f"hardware_profile={args.hardware_profile}")
    print(f"left_power={left_power:.3f}")
    print(f"right_power={right_power:.3f}")
    print(f"durations={args.durations}")
    print(f"csv={args.csv}")
    print()
    print("Place robot in starting position with enough room to drive.")
    print("Measure actual travel after each run and type it in.")

    for idx, duration_s in enumerate(args.durations, start=1):
        print()
        input(f"Press Enter for test {idx} (drive for {duration_s:.3f}s)... ")

        start_wall = time.time()
        run_drive(io_map, left_power, right_power, duration_s)
        end_wall = time.time()

        actual_mm = prompt_float("Observed distance travelled (mm): ")
        notes = input("Notes (optional): ").strip()

        append_csv_row(
            args.csv,
            {
                "timestamp_wall": start_wall,
                "mode": "drive",
                "label": args.label,
                "hardware_profile": args.hardware_profile,
                "test_index": idx,
                "left_power": left_power,
                "right_power": right_power,
                "duration_s_cmd": duration_s,
                "duration_s_wall": end_wall - start_wall,
                "actual_mm": actual_mm,
                "notes": notes,
            },
        )

        print(f"Logged test {idx} to {args.csv}")
        if args.settle > 0:
            print(f"Settling for {args.settle:.2f}s...")
            io_map.sleep(args.settle)

    print()
    print("Done.")
    print("Fit: duration_s_cmd = m * actual_mm + b")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())