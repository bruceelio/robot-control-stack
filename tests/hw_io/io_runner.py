"""Interactive pre-competition IO checkout runner.

The CSV is the script. Rows run top-to-bottom.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import csv
import os
import sys
import time
from typing import Callable, Iterable, Optional

from tests.hw_io.io_path import parse_io_path, read_io_path, write_io_path
from tests.hw_io.io_rules import (
    DEFAULT_MOTOR_POWER,
    DEFAULT_MOTOR_PULSE_SECONDS,
    DEFAULT_SERVO_RANGE,
    RANGES,
    TEST_RULES,
    THRESHOLDS,
)


@dataclass
class Result:
    status: str
    path: str
    note: str = ""


def as_bool(value: str) -> bool:
    return str(value).strip().lower() in {"true", "t", "yes", "y", "1"}


def load_csv(csv_path: str) -> list[dict]:
    with open(csv_path, newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"pi_io_path", "exists", "enabled", "notes"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"CSV missing columns: {sorted(missing)}")
        return [row for row in reader if row.get("pi_io_path", "").strip()]


def start_log(csv_path: str):
    base = os.path.splitext(os.path.basename(csv_path))[0]
    stamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    log_path = os.path.join(os.path.dirname(csv_path), f"{base}_checkout_{stamp}.txt")
    log = open(log_path, "w", encoding="utf-8")
    log.write("=== IO CHECKOUT ===\n")
    log.write(f"Robot map: {os.path.basename(csv_path)}\n")
    log.write(f"Timestamp: {datetime.now().isoformat(timespec='seconds')}\n\n")
    print(f"Log: {log_path}")
    return log


def log_result(log, result: Result) -> None:
    note = f"  ({result.note})" if result.note else ""
    line = f"{result.status:<6} {result.path}{note}"
    print(line)
    log.write(line + "\n")
    log.flush()


def find_rule(path: str) -> str:
    parsed = parse_io_path(path)
    try:
        return TEST_RULES[parsed.rule_prefix]
    except KeyError as exc:
        raise ValueError(f"No test rule for {parsed.rule_prefix} ({path})") from exc


def operator_choice(prompt="Choice: ", allowed=("c", "r", "f", "s", "q")) -> str:
    allowed = set(allowed)
    while True:
        value = input(prompt).strip().lower()
        if value in allowed:
            return value
        print(f"Choose one of: {', '.join(sorted(allowed))}")


def sleep_io(io, seconds: float) -> None:
    sleeper = getattr(io, "sleep", None)
    if callable(sleeper):
        sleeper(seconds)
    else:
        time.sleep(seconds)


def monitor_until(
    io,
    *,
    path: str,
    label: str,
    formatter: Callable[[object], str] = str,
    pass_check: Callable[[list[object]], bool],
    max_seconds: Optional[float] = None,
    poll_seconds: float = 0.05,
) -> Result:
    print("Options while monitoring: [f] fail and continue  [s] skip  [q] quit checkout")
    values = []
    last_text = None
    start = time.monotonic()

    while True:
        value = read_io_path(io, path)
        values.append(value)
        text = formatter(value)
        if text != last_text:
            print(f"{label}: {text}")
            last_text = text

        if pass_check(values):
            return Result("PASS", path)

        if key_pressed():
            choice = sys.stdin.read(1).lower()
            if choice == "f":
                return Result("FAIL", path, "operator failed")
            if choice == "s":
                return Result("SKIP", path, "operator skipped")
            if choice == "q":
                return Result("ABORT", path, "operator quit")

        if max_seconds and time.monotonic() - start > max_seconds:
            return Result("FAIL", path, "timeout")

        sleep_io(io, poll_seconds)


def key_pressed() -> bool:
    """Non-blocking keyboard check for Linux terminals; falls back to False."""
    try:
        import select
        import termios
        import tty

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            return bool(ready)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
    except Exception:
        return False


def run_io_checkout(io, csv_path: str) -> bool:
    rows = load_csv(csv_path)
    active = [r for r in rows if as_bool(r["exists"]) and as_bool(r["enabled"])]
    disabled = [r for r in rows if as_bool(r["exists"]) and not as_bool(r["enabled"])]
    not_present = [r for r in rows if not as_bool(r["exists"])]

    print("\n=== IO CHECKOUT START ===")
    print(f"Loaded: {csv_path}")
    print(f"Active: {len(active)}  Disabled: {len(disabled)}  Not present: {len(not_present)}")
    input("Press Enter to begin...")

    log = start_log(csv_path)
    results: list[Result] = []

    try:
        for row in rows:
            path = row["pi_io_path"].strip()
            notes = row.get("notes", "").strip()

            if not as_bool(row["exists"]):
                continue

            if not as_bool(row["enabled"]):
                result = Result("SKIP", path, notes or "disabled")
                results.append(result)
                log_result(log, result)
                continue

            try:
                rule = find_rule(path)
            except Exception as exc:
                result = Result("FAIL", path, f"rule error: {exc}")
                results.append(result)
                log_result(log, result)
                continue

            if rule == "external_protocol":
                result = Result("SKIP", path, notes or "tested separately")
            else:
                result = run_rule(rule, io, path, notes)

            results.append(result)
            log_result(log, result)

            if result.status == "ABORT":
                break
    finally:
        write_summary(log, results)
        log.close()

    return not any(r.status in {"FAIL", "ABORT"} for r in results)


def run_rule(rule: str, io, path: str, notes: str = "") -> Result:
    while True:
        print(f"\n[RUN] {path}")
        if notes:
            print(f"notes: {notes}")

        if rule == "digital_input":
            result = test_digital_input(io, path)
        elif rule == "analog_delta":
            result = test_analog_delta(io, path)
        elif rule == "range_check":
            result = test_range_check(io, path)
        elif rule == "motor_output":
            result = test_motor_output(io, path)
        elif rule == "servo_output":
            result = test_servo_output(io, path)
        elif rule == "encoder":
            result = test_encoder(io, path)
        elif rule == "imu":
            result = test_imu(io, path)
        elif rule == "otos":
            result = test_otos(io, path)
        else:
            result = Result("FAIL", path, f"unknown rule: {rule}")

        if result.status in {"ABORT", "SKIP", "FAIL"}:
            return result

        choice = operator_choice("[c] continue  [r] retest  [f] fail  [s] skip  [q] quit: ")
        if choice == "c":
            return result
        if choice == "r":
            continue
        if choice == "f":
            return Result("FAIL", path, "operator failed after test")
        if choice == "s":
            return Result("SKIP", path, "operator skipped after test")
        if choice == "q":
            return Result("ABORT", path, "operator quit")


def test_digital_input(io, path: str) -> Result:
    parsed = parse_io_path(path)
    name = parsed.name or parsed.category

    def check(values):
        return True in values and False in values

    return monitor_until(
        io,
        path=path,
        label=name,
        formatter=lambda v: f"{'TRIGGERED' if bool(v) else 'RELEASED'} / {bool(v)}",
        pass_check=check,
    )


def test_analog_delta(io, path: str) -> Result:
    parsed = parse_io_path(path)
    label = parsed.name or path
    threshold = THRESHOLDS.get("analog_delta", 0.15)
    if parsed.category == "ultrasonic":
        threshold = THRESHOLDS.get("ultrasonic_delta", 150.0)
    if parsed.category == "current":
        threshold = THRESHOLDS.get("current_delta_amps", 0.2)

    values: list[float] = []

    def check(raw_values):
        try:
            nums = [float(v) for v in raw_values]
        except Exception:
            return False
        values[:] = nums
        return len(nums) > 2 and max(nums) - min(nums) >= threshold

    result = monitor_until(
        io,
        path=path,
        label=label,
        formatter=lambda v: f"{v}  (min/max: {min(values) if values else v}/{max(values) if values else v})",
        pass_check=check,
    )
    if result.status == "PASS" and values:
        result.note = f"min={min(values):.3g}, max={max(values):.3g}, delta={max(values)-min(values):.3g}"
    return result


def test_range_check(io, path: str) -> Result:
    value = float(read_io_path(io, path))
    low, high = RANGES.get(path, (float("-inf"), float("inf")))
    print(f"value: {value}")
    print(f"expected range: {low} to {high}")
    if low <= value <= high:
        return Result("PASS", path, f"value={value:.3g}")
    return Result("FAIL", path, f"value={value:.3g} outside range")


def test_motor_output(io, path: str) -> Result:
    power = DEFAULT_MOTOR_POWER
    duration = DEFAULT_MOTOR_PULSE_SECONDS
    input("Ensure mechanism/wheels are clear or lifted. Press Enter to pulse motor...")
    try:
        print(f"Forward pulse: +{power}")
        write_io_path(io, path, power)
        sleep_io(io, duration)
        write_io_path(io, path, 0)
        sleep_io(io, 0.2)
        print(f"Reverse pulse: -{power}")
        write_io_path(io, path, -power)
        sleep_io(io, duration)
        write_io_path(io, path, 0)
    finally:
        try:
            write_io_path(io, path, 0)
        except Exception:
            pass
    choice = operator_choice("[c] correct/pass  [w] wrong direction  [n] no movement  [r] retest  [f] fail  [s] skip  [q] quit: ", allowed=("c", "w", "n", "r", "f", "s", "q"))
    if choice == "c":
        return Result("PASS", path)
    if choice == "r":
        return test_motor_output(io, path)
    if choice == "s":
        return Result("SKIP", path, "operator skipped")
    if choice == "q":
        return Result("ABORT", path, "operator quit")
    return Result("FAIL", path, {"w": "wrong direction", "n": "no movement", "f": "operator failed"}.get(choice, "operator failed"))


def test_servo_output(io, path: str) -> Result:
    default_min, default_neutral, default_max = DEFAULT_SERVO_RANGE
    raw = input(f"Servo range min,neutral,max [default {default_min},{default_neutral},{default_max}]: ").strip()
    if raw:
        parts = [float(p.strip()) for p in raw.split(",")]
        if len(parts) != 3:
            return Result("FAIL", path, "servo range must be min,neutral,max")
        pos_min, pos_neutral, pos_max = parts
    else:
        pos_min, pos_neutral, pos_max = default_min, default_neutral, default_max

    input("Ensure mechanism is clear. Press Enter to move servo...")
    try:
        for label, position in (("min", pos_min), ("max", pos_max), ("neutral", pos_neutral)):
            print(f"Move {label}: {position}")
            write_io_path(io, path, position)
            sleep_io(io, 0.5)
    finally:
        try:
            write_io_path(io, path, pos_neutral)
        except Exception:
            pass

    choice = operator_choice("[c] correct/pass  [w] wrong direction  [n] no movement  [b] binding  [r] retest  [f] fail  [s] skip  [q] quit: ", allowed=("c", "w", "n", "b", "r", "f", "s", "q"))
    if choice == "c":
        return Result("PASS", path)
    if choice == "r":
        return test_servo_output(io, path)
    if choice == "s":
        return Result("SKIP", path, "operator skipped")
    if choice == "q":
        return Result("ABORT", path, "operator quit")
    return Result("FAIL", path, {"w": "wrong direction", "n": "no movement", "b": "binding", "f": "operator failed"}.get(choice, "operator failed"))



def test_encoder(io, path: str) -> Result:
    parsed = parse_io_path(path)
    if parsed.name is None:
        return Result("FAIL", path, "encoder path must include a named encoder")

    base = f'io.encoder["{parsed.name}"]'
    a_path = base + ".A"
    b_path = base + ".B"

    print("Move the wheel/shaft manually. No motors will be commanded.")
    print("Live A/B quadrature snapshots will be shown on change.")
    print("Options while monitoring: [f] fail and continue  [s] skip  [q] quit checkout")

    seen_a = set()
    seen_b = set()
    seen_pairs = set()
    last_pair = None
    transitions = 0

    while True:
        try:
            a = bool(read_io_path(io, a_path))
            b = bool(read_io_path(io, b_path))
        except Exception as exc:
            return Result("FAIL", path, f"encoder read error: {exc}")

        pair = (a, b)
        seen_a.add(a)
        seen_b.add(b)
        seen_pairs.add(pair)

        if pair != last_pair:
            transitions += 1
            print(f"A={int(a)}  B={int(b)}  transitions={transitions}")
            last_pair = pair

        required_changes = int(THRESHOLDS.get("encoder_state_changes", 3))
        if len(seen_a) == 2 and len(seen_b) == 2 and transitions >= required_changes:
            states = sorted((int(x), int(y)) for x, y in seen_pairs)
            return Result(
                "PASS",
                path,
                f"A changed, B changed, states_seen={states}",
            )

        if key_pressed():
            choice = sys.stdin.read(1).lower()
            if choice == "f":
                return Result("FAIL", path, "operator failed")
            if choice == "s":
                return Result("SKIP", path, "operator skipped")
            if choice == "q":
                return Result("ABORT", path, "operator quit")
        sleep_io(io, 0.1)

def test_imu(io, path: str) -> Result:
    values = {"heading": [], "pitch": [], "roll": []}
    print("Rotate or tilt the robot slightly.")
    print("Options while monitoring: [f] fail and continue  [s] skip  [q] quit checkout")
    while True:
        try:
            h = float(read_io_path(io, "io.imu.heading"))
            p = float(read_io_path(io, "io.imu.pitch"))
            r = float(read_io_path(io, "io.imu.roll"))
        except Exception as exc:
            return Result("FAIL", path, f"imu read error: {exc}")
        values["heading"].append(h)
        values["pitch"].append(p)
        values["roll"].append(r)
        print(f"heading={h:.1f}  pitch={p:.1f}  roll={r:.1f}")
        deltas = {k: max(v) - min(v) for k, v in values.items()}
        if any(delta >= THRESHOLDS["imu_degrees"] for delta in deltas.values()):
            return Result("PASS", path, ", ".join(f"{k}_delta={v:.1f}" for k, v in deltas.items()))
        if key_pressed():
            choice = sys.stdin.read(1).lower()
            if choice == "f":
                return Result("FAIL", path, "operator failed")
            if choice == "s":
                return Result("SKIP", path, "operator skipped")
            if choice == "q":
                return Result("ABORT", path, "operator quit")
        sleep_io(io, 0.25)


def _pose_components(pose):
    if isinstance(pose, dict):
        return float(pose["x"]), float(pose["y"]), float(pose["heading"])
    return float(pose.x), float(pose.y), float(pose.heading)


def test_otos(io, path: str) -> Result:
    xs, ys, headings = [], [], []
    print("Move the robot slightly forward/sideways or rotate it.")
    print("Options while monitoring: [f] fail and continue  [s] skip  [q] quit checkout")
    while True:
        try:
            x, y, h = _pose_components(read_io_path(io, "io.otos.pose"))
        except Exception as exc:
            return Result("FAIL", path, f"otos read error: {exc}")
        xs.append(x)
        ys.append(y)
        headings.append(h)
        print(f"x={x:.3f}  y={y:.3f}  heading={h:.1f}")
        dx = max(xs) - min(xs)
        dy = max(ys) - min(ys)
        dh = max(headings) - min(headings)
        if dx >= THRESHOLDS["otos_position"] or dy >= THRESHOLDS["otos_position"] or dh >= THRESHOLDS["otos_heading_degrees"]:
            return Result("PASS", path, f"dx={dx:.3g}, dy={dy:.3g}, dheading={dh:.1f}")
        if key_pressed():
            choice = sys.stdin.read(1).lower()
            if choice == "f":
                return Result("FAIL", path, "operator failed")
            if choice == "s":
                return Result("SKIP", path, "operator skipped")
            if choice == "q":
                return Result("ABORT", path, "operator quit")
        sleep_io(io, 0.25)


def write_summary(log, results: Iterable[Result]) -> None:
    results = list(results)
    counts = {"PASS": 0, "FAIL": 0, "SKIP": 0, "ABORT": 0}
    for result in results:
        counts[result.status] = counts.get(result.status, 0) + 1

    text = (
        "\n=== SUMMARY ===\n"
        f"PASS: {counts.get('PASS', 0)}\n"
        f"FAIL: {counts.get('FAIL', 0)}\n"
        f"SKIP: {counts.get('SKIP', 0)}\n"
        f"ABORT: {counts.get('ABORT', 0)}\n"
    )
    print(text)
    log.write(text)
