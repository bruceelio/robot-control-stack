# diagnostics/odometry_drive_motors.py

from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

from config import CONFIG
from hw_io.resolve import resolve_io


LEFT_ENCODER = "drive_front_left"
RIGHT_ENCODER = "drive_front_right"
LEFT_MOTOR = "drive_front_left"
RIGHT_MOTOR = "drive_front_right"

RESULTS_PATH = Path("diagnostics/results/drive_motor_odometry.csv")

CSV_FIELDS = [
    "timestamp",
    "robot_id",
    "hardware_profile",
    "trial",
    "test_type",
    "direction",
    "motor_power",
    "duration_commanded_s",
    "duration_powered_s",
    "battery_before_v",
    "battery_after_v",
    "left_count_start",
    "left_count_end",
    "left_delta_raw",
    "left_encoder_sign",
    "left_delta_corrected",
    "right_count_start",
    "right_count_end",
    "right_delta_raw",
    "right_encoder_sign",
    "right_delta_corrected",
    "counts_per_rev",
    "wheel_diameter_mm",
    "track_width_mm",
    "left_distance_calculated_mm",
    "right_distance_calculated_mm",
    "centre_distance_calculated_mm",
    "heading_change_calculated_deg",
    "distance_measured_mm",
    "distance_error_mm",
    "distance_error_percent",
    "left_valid_start",
    "right_valid_start",
    "left_valid_end",
    "right_valid_end",
    "left_valid_flags_end",
    "right_valid_flags_end",
    "notes",
]


@dataclass(frozen=True)
class EncoderSnapshot:
    count: int
    timestamp_ms: int
    valid: bool
    valid_flags: int


@dataclass(frozen=True)
class OdometrySettings:
    counts_per_rev: float
    wheel_diameter_mm: float
    track_width_mm: Optional[float]
    left_sign: int
    right_sign: int


@dataclass(frozen=True)
class TrialCalculation:
    left_delta_raw: int
    right_delta_raw: int
    left_delta_corrected: int
    right_delta_corrected: int
    left_distance_mm: float
    right_distance_mm: float
    centre_distance_mm: float
    heading_change_deg: Optional[float]


def _prompt_float(
    prompt: str,
    *,
    minimum: Optional[float] = None,
    maximum: Optional[float] = None,
    default: Optional[float] = None,
) -> float:
    while True:
        suffix = "" if default is None else f" [{default:g}]"
        raw = input(f"{prompt}{suffix}: ").strip()

        if raw == "" and default is not None:
            value = default
        else:
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


def _prompt_optional_float(
    prompt: str,
    *,
    minimum: Optional[float] = None,
    default: Optional[float] = None,
) -> Optional[float]:
    while True:
        default_text = "" if default is None else f" [{default:g}]"
        raw = input(f"{prompt}{default_text} (blank = unavailable): ").strip()

        if raw == "":
            return default

        try:
            value = float(raw)
        except ValueError:
            print("Enter a numeric value or leave blank.")
            continue

        if minimum is not None and value < minimum:
            print(f"Value must be at least {minimum}.")
            continue

        return value


def _prompt_sign(prompt: str, *, default: int) -> int:
    while True:
        raw = input(f"{prompt} [+1/-1] [{default:+d}]: ").strip()

        if raw == "":
            return default

        if raw in {"1", "+1"}:
            return 1

        if raw == "-1":
            return -1

        print("Enter +1 or -1.")


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


def _prompt_direction() -> tuple[str, float]:
    while True:
        raw = input("Direction [F/R]: ").strip().lower()

        if raw in {"f", "forward"}:
            return "forward", 1.0

        if raw in {"r", "reverse", "backward"}:
            return "reverse", -1.0

        print("Enter F for forward or R for reverse.")


def _first_numeric_config(*names: str) -> Optional[float]:
    for name in names:
        value = getattr(CONFIG, name, None)
        if value is None:
            continue

        try:
            return float(value)
        except (TypeError, ValueError):
            continue

    return None


def _configured_encoder_sign(name: str, default: int) -> int:
    for attribute_name in ("encoder_sign", "encoder_polarity", "ENCODER_SIGN"):
        mapping = getattr(CONFIG, attribute_name, None)
        if isinstance(mapping, dict) and name in mapping:
            try:
                value = int(mapping[name])
            except (TypeError, ValueError):
                break

            if value in {-1, 1}:
                return value

    return default


def _read_encoder(encoder: Any) -> EncoderSnapshot:
    try:
        count = int(encoder.count)
        timestamp_ms = int(encoder.timestamp_ms)
        valid = bool(encoder.valid)
        valid_flags = int(encoder.valid_flags)
    except (AttributeError, TypeError, ValueError) as exc:
        raise RuntimeError(
            "Encoder does not expose count, timestamp_ms, valid and valid_flags."
        ) from exc

    return EncoderSnapshot(
        count=count,
        timestamp_ms=timestamp_ms,
        valid=valid,
        valid_flags=valid_flags,
    )


def _read_pair(left_encoder: Any, right_encoder: Any) -> tuple[EncoderSnapshot, EncoderSnapshot]:
    return _read_encoder(left_encoder), _read_encoder(right_encoder)


def _read_battery_voltage(io: Any) -> Optional[float]:
    try:
        value = float(io.voltage["battery"].volts)
    except (AttributeError, KeyError, TypeError, ValueError):
        return None

    return value if value > 1.0 else None


def _format_voltage(value: Optional[float]) -> str:
    return "unavailable" if value is None else f"{value:.2f} V"


def _format_snapshot(name: str, snapshot: EncoderSnapshot) -> str:
    return (
        f"{name}: count={snapshot.count}, "
        f"timestamp_ms={snapshot.timestamp_ms}, "
        f"valid={int(snapshot.valid)}, "
        f"valid_flags={snapshot.valid_flags}"
    )


def _stop_drive(left_motor: Any, right_motor: Any) -> None:
    first_error: Optional[Exception] = None

    try:
        left_motor.power = 0.0
    except Exception as exc:
        first_error = exc

    try:
        right_motor.power = 0.0
    except Exception as exc:
        if first_error is None:
            first_error = exc

    if first_error is not None:
        raise first_error


def _append_result(row: dict[str, Any]) -> None:
    RESULTS_PATH.parent.mkdir(parents=True, exist_ok=True)
    write_header = not RESULTS_PATH.exists() or RESULTS_PATH.stat().st_size == 0

    with RESULTS_PATH.open("a", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)

        if write_header:
            writer.writeheader()

        writer.writerow(row)


def _calculate_trial(
    start_left: EncoderSnapshot,
    start_right: EncoderSnapshot,
    end_left: EncoderSnapshot,
    end_right: EncoderSnapshot,
    settings: OdometrySettings,
) -> TrialCalculation:
    left_delta_raw = end_left.count - start_left.count
    right_delta_raw = end_right.count - start_right.count

    left_delta_corrected = left_delta_raw * settings.left_sign
    right_delta_corrected = right_delta_raw * settings.right_sign

    mm_per_count = math.pi * settings.wheel_diameter_mm / settings.counts_per_rev
    left_distance_mm = left_delta_corrected * mm_per_count
    right_distance_mm = right_delta_corrected * mm_per_count
    centre_distance_mm = (left_distance_mm + right_distance_mm) / 2.0

    heading_change_deg: Optional[float] = None
    if settings.track_width_mm is not None and settings.track_width_mm > 0.0:
        heading_change_rad = (
            right_distance_mm - left_distance_mm
        ) / settings.track_width_mm
        heading_change_deg = math.degrees(heading_change_rad)

    return TrialCalculation(
        left_delta_raw=left_delta_raw,
        right_delta_raw=right_delta_raw,
        left_delta_corrected=left_delta_corrected,
        right_delta_corrected=right_delta_corrected,
        left_distance_mm=left_distance_mm,
        right_distance_mm=right_distance_mm,
        centre_distance_mm=centre_distance_mm,
        heading_change_deg=heading_change_deg,
    )


def _print_calculation(calculation: TrialCalculation) -> None:
    print("\nEncoder change:")
    print(
        f"  Left:  raw={calculation.left_delta_raw:+d}, "
        f"corrected={calculation.left_delta_corrected:+d}, "
        f"distance={calculation.left_distance_mm:+.1f} mm"
    )
    print(
        f"  Right: raw={calculation.right_delta_raw:+d}, "
        f"corrected={calculation.right_delta_corrected:+d}, "
        f"distance={calculation.right_distance_mm:+.1f} mm"
    )
    print(f"  Centre distance: {calculation.centre_distance_mm:+.1f} mm")

    if calculation.heading_change_deg is None:
        print("  Heading change: unavailable (track width not supplied)")
    else:
        print(f"  Heading change: {calculation.heading_change_deg:+.2f} deg")


def _check_snapshot_validity(
    left: EncoderSnapshot,
    right: EncoderSnapshot,
    *,
    phase: str,
) -> None:
    if not left.valid or not right.valid:
        print(f"[WARN] One or both encoder snapshots are invalid at {phase}.")

    if left.valid_flags != 0 or right.valid_flags != 0:
        print(
            f"[WARN] Encoder validity flags at {phase}: "
            f"left={left.valid_flags}, right={right.valid_flags}"
        )


def _stationary_test(
    io: Any,
    left_encoder: Any,
    right_encoder: Any,
) -> None:
    duration_s = _prompt_float(
        "Stationary observation duration (seconds)",
        minimum=0.2,
        default=3.0,
    )
    interval_s = _prompt_float(
        "Sample interval (seconds)",
        minimum=0.05,
        default=0.5,
    )

    print("\nKeep both wheels completely stationary.")
    input("Press Enter to begin: ")

    start_left, start_right = _read_pair(left_encoder, right_encoder)
    previous_left = start_left
    previous_right = start_right

    print(_format_snapshot("Left start ", start_left))
    print(_format_snapshot("Right start", start_right))

    deadline = time.monotonic() + duration_s
    sample_number = 1

    while time.monotonic() < deadline:
        io.sleep(min(interval_s, max(0.0, deadline - time.monotonic())))
        current_left, current_right = _read_pair(left_encoder, right_encoder)

        print(
            f"Sample {sample_number:02d}: "
            f"left={current_left.count} ({current_left.count - previous_left.count:+d}), "
            f"right={current_right.count} ({current_right.count - previous_right.count:+d})"
        )

        previous_left = current_left
        previous_right = current_right
        sample_number += 1

    end_left, end_right = _read_pair(left_encoder, right_encoder)
    left_change = end_left.count - start_left.count
    right_change = end_right.count - start_right.count

    print("\nStationary result:")
    print(f"  Left count change:  {left_change:+d}")
    print(f"  Right count change: {right_change:+d}")

    if left_change == 0 and right_change == 0:
        print("  PASS: both cumulative counts remained stable.")
    else:
        print("  REVIEW: a stationary encoder count changed.")

    _check_snapshot_validity(end_left, end_right, phase="stationary test end")


def _manual_count_test(
    left_encoder: Any,
    right_encoder: Any,
    settings: OdometrySettings,
) -> None:
    print("\nRaise the robot so the wheels can be turned safely by hand.")
    print("This test does not command either motor.")

    start_left, start_right = _read_pair(left_encoder, right_encoder)
    print(_format_snapshot("Left start ", start_left))
    print(_format_snapshot("Right start", start_right))

    input("\nTurn the required wheel(s), then press Enter: ")

    end_left, end_right = _read_pair(left_encoder, right_encoder)
    calculation = _calculate_trial(
        start_left,
        start_right,
        end_left,
        end_right,
        settings,
    )

    print(_format_snapshot("Left end   ", end_left))
    print(_format_snapshot("Right end  ", end_right))
    _print_calculation(calculation)
    _check_snapshot_validity(end_left, end_right, phase="manual test end")

    print(
        "\nFor a one-revolution test, compare the absolute corrected delta "
        "with the configured counts per revolution."
    )


def _powered_trial(
    *,
    trial: int,
    io: Any,
    left_motor: Any,
    right_motor: Any,
    left_encoder: Any,
    right_encoder: Any,
    settings: OdometrySettings,
    max_power: float,
) -> None:
    print(f"\n--- Powered Trial {trial} ---")
    direction, direction_sign = _prompt_direction()
    power_magnitude = _prompt_float(
        f"Motor power magnitude [0.0 to {max_power:.2f}]",
        minimum=0.0,
        maximum=max_power,
    )
    duration_commanded_s = _prompt_float(
        "Drive duration (seconds)",
        minimum=0.01,
    )

    signed_power = direction_sign * power_magnitude

    print("\nPosition and align the robot at the starting mark.")
    print(
        f"Next command: {direction} at power {power_magnitude:.3f} "
        f"for {duration_commanded_s:.3f} s"
    )

    confirmation = input(
        "Press Enter to run, or type Q then Enter to cancel: "
    ).strip().lower()
    if confirmation in {"q", "quit"}:
        return

    start_left, start_right = _read_pair(left_encoder, right_encoder)
    _check_snapshot_validity(start_left, start_right, phase="powered trial start")

    battery_before_v = _read_battery_voltage(io)
    print(f"Battery before: {_format_voltage(battery_before_v)}")
    print("DRIVING...")

    powered_started_s = time.monotonic()

    try:
        left_motor.power = signed_power
        right_motor.power = signed_power
        io.sleep(duration_commanded_s)
    except Exception:
        print("\n[ERROR] Direct checkout drive command failed.")
        print("Run the drive motor IO checkout before repeating this diagnostic.")
        raise
    finally:
        powered_stopped_s = time.monotonic()
        _stop_drive(left_motor, right_motor)

    duration_powered_s = powered_stopped_s - powered_started_s
    print("Drive complete.")
    io.sleep(0.25)

    end_left, end_right = _read_pair(left_encoder, right_encoder)
    battery_after_v = _read_battery_voltage(io)
    calculation = _calculate_trial(
        start_left,
        start_right,
        end_left,
        end_right,
        settings,
    )

    print(f"Battery after:  {_format_voltage(battery_after_v)}")
    _print_calculation(calculation)
    _check_snapshot_validity(end_left, end_right, phase="powered trial end")

    distance_measured_mm = _prompt_optional_float(
        "Measured centre distance travelled (mm)",
        minimum=0.0,
    )
    notes = input("Notes (optional): ").strip()

    distance_error_mm: Optional[float] = None
    distance_error_percent: Optional[float] = None

    if distance_measured_mm is not None:
        expected_signed_distance = direction_sign * distance_measured_mm
        distance_error_mm = calculation.centre_distance_mm - expected_signed_distance

        if distance_measured_mm > 0.0:
            distance_error_percent = 100.0 * distance_error_mm / distance_measured_mm

        print(f"Calculated - measured error: {distance_error_mm:+.1f} mm")
        if distance_error_percent is not None:
            print(f"Distance error: {distance_error_percent:+.2f}%")

    row = {
        "timestamp": datetime.now().astimezone().isoformat(timespec="seconds"),
        "robot_id": getattr(CONFIG, "robot_id", ""),
        "hardware_profile": getattr(CONFIG, "hardware_profile", ""),
        "trial": trial,
        "test_type": "powered_straight",
        "direction": direction,
        "motor_power": f"{signed_power:.6f}",
        "duration_commanded_s": f"{duration_commanded_s:.6f}",
        "duration_powered_s": f"{duration_powered_s:.6f}",
        "battery_before_v": "" if battery_before_v is None else f"{battery_before_v:.6f}",
        "battery_after_v": "" if battery_after_v is None else f"{battery_after_v:.6f}",
        "left_count_start": start_left.count,
        "left_count_end": end_left.count,
        "left_delta_raw": calculation.left_delta_raw,
        "left_encoder_sign": settings.left_sign,
        "left_delta_corrected": calculation.left_delta_corrected,
        "right_count_start": start_right.count,
        "right_count_end": end_right.count,
        "right_delta_raw": calculation.right_delta_raw,
        "right_encoder_sign": settings.right_sign,
        "right_delta_corrected": calculation.right_delta_corrected,
        "counts_per_rev": f"{settings.counts_per_rev:.6f}",
        "wheel_diameter_mm": f"{settings.wheel_diameter_mm:.6f}",
        "track_width_mm": "" if settings.track_width_mm is None else f"{settings.track_width_mm:.6f}",
        "left_distance_calculated_mm": f"{calculation.left_distance_mm:.6f}",
        "right_distance_calculated_mm": f"{calculation.right_distance_mm:.6f}",
        "centre_distance_calculated_mm": f"{calculation.centre_distance_mm:.6f}",
        "heading_change_calculated_deg": (
            "" if calculation.heading_change_deg is None else f"{calculation.heading_change_deg:.6f}"
        ),
        "distance_measured_mm": (
            "" if distance_measured_mm is None else f"{distance_measured_mm:.3f}"
        ),
        "distance_error_mm": "" if distance_error_mm is None else f"{distance_error_mm:.6f}",
        "distance_error_percent": (
            "" if distance_error_percent is None else f"{distance_error_percent:.6f}"
        ),
        "left_valid_start": int(start_left.valid),
        "right_valid_start": int(start_right.valid),
        "left_valid_end": int(end_left.valid),
        "right_valid_end": int(end_right.valid),
        "left_valid_flags_end": end_left.valid_flags,
        "right_valid_flags_end": end_right.valid_flags,
        "notes": notes,
    }

    _append_result(row)
    print(f"Saved powered trial {trial}.")
    print(f"Results file: {RESULTS_PATH}")


def _resolve_settings() -> OdometrySettings:
    configured_counts_per_rev = _first_numeric_config(
        "output_shaft_counts_per_rev",
        "encoder_counts_per_rev",
        "drive_encoder_counts_per_rev",
    )
    configured_wheel_diameter = _first_numeric_config(
        "effective_wheel_diameter_mm",
        "wheel_diameter_mm",
        "drive_wheel_diameter_mm",
    )
    configured_track_width = _first_numeric_config(
        "effective_track_width_mm",
        "track_width_mm",
        "drive_track_width_mm",
    )

    counts_per_rev = _prompt_float(
        "Encoder counts per wheel/output-shaft revolution",
        minimum=1.0,
        default=configured_counts_per_rev or 700.0,
    )
    wheel_diameter_mm = _prompt_float(
        "Effective drive-wheel diameter (mm)",
        minimum=1.0,
        default=configured_wheel_diameter or 96.0,
    )
    track_width_mm = _prompt_optional_float(
        "Effective drive track width (mm)",
        minimum=1.0,
        default=configured_track_width,
    )

    left_sign = _prompt_sign(
        "Left encoder installation sign",
        default=_configured_encoder_sign(LEFT_ENCODER, 1),
    )
    right_sign = _prompt_sign(
        "Right encoder installation sign",
        default=_configured_encoder_sign(RIGHT_ENCODER, 1),
    )

    return OdometrySettings(
        counts_per_rev=counts_per_rev,
        wheel_diameter_mm=wheel_diameter_mm,
        track_width_mm=track_width_mm,
        left_sign=left_sign,
        right_sign=right_sign,
    )


def run(robot=None) -> None:
    print("\n=== DRIVE-MOTOR ODOMETRY DIAGNOSTIC ===")
    print("Checks cumulative drive encoder counts and calculated wheel travel.")
    print("Arduino supplies raw cumulative counts; this program performs Pi-side odometry calculations.")
    print("Powered trials bypass the Level2 software interface.")
    print("Raise the robot for manual tests and clear the lane for powered tests.\n")

    io = resolve_io(
        robot=robot,
    )

    left_encoder = io.encoder[LEFT_ENCODER]
    right_encoder = io.encoder[RIGHT_ENCODER]
    left_motor = io.motor[LEFT_MOTOR]
    right_motor = io.motor[RIGHT_MOTOR]

    max_power = float(getattr(CONFIG, "max_motor_power", 1.0))

    initial_left, initial_right = _read_pair(left_encoder, right_encoder)
    print("Initial encoder snapshots:")
    print(_format_snapshot("Left ", initial_left))
    print(_format_snapshot("Right", initial_right))
    _check_snapshot_validity(initial_left, initial_right, phase="startup")

    print(
        f"\nMaximum motor power: {max_power:.2f}\n"
        f"Source: CONFIG.max_motor_power for "
        f"{getattr(CONFIG, 'robot_id', 'selected robot')}"
    )

    settings = _resolve_settings()

    mm_per_count = math.pi * settings.wheel_diameter_mm / settings.counts_per_rev
    print("\nResolved diagnostic conversion:")
    print(f"  Counts per revolution: {settings.counts_per_rev:g}")
    print(f"  Wheel diameter:       {settings.wheel_diameter_mm:g} mm")
    print(f"  Distance per count:   {mm_per_count:.6f} mm")
    print(f"  Left encoder sign:    {settings.left_sign:+d}")
    print(f"  Right encoder sign:   {settings.right_sign:+d}")
    print(
        "  Track width:          "
        + (
            "unavailable"
            if settings.track_width_mm is None
            else f"{settings.track_width_mm:g} mm"
        )
    )

    powered_trial_number = 1

    try:
        while True:
            print("\nSelect diagnostic:")
            print("  1 - Stationary count stability")
            print("  2 - Manual wheel/count test")
            print("  3 - Powered straight-drive odometry trial")
            print("  Q - Quit")

            selection = input("Selection: ").strip().lower()

            if selection in {"q", "quit"}:
                break

            if selection == "1":
                _stationary_test(io, left_encoder, right_encoder)
            elif selection == "2":
                _manual_count_test(left_encoder, right_encoder, settings)
            elif selection == "3":
                _powered_trial(
                    trial=powered_trial_number,
                    io=io,
                    left_motor=left_motor,
                    right_motor=right_motor,
                    left_encoder=left_encoder,
                    right_encoder=right_encoder,
                    settings=settings,
                    max_power=max_power,
                )
                powered_trial_number += 1
            else:
                print("Enter 1, 2, 3 or Q.")
                continue

            if not _prompt_yes_no("Return to the diagnostic menu?", default=True):
                break

    finally:
        try:
            _stop_drive(left_motor, right_motor)
        except Exception as exc:
            print(f"[WARN] Final motor stop failed: {exc}")

    print("\n=== END DRIVE-MOTOR ODOMETRY DIAGNOSTIC ===")


if __name__ == "__main__":
    run(robot=None)