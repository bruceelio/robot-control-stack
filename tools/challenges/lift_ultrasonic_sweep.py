# tools/challenges/lift_ultrasonic_sweep.py
"""Diagnostic lift-position / range-sensor sweep (Webots and real robots).

The challenge entry point run(controller) starts from normal Webots spawn. It
reuses the existing autonomous state machine through its first successful
PickupObject, then performs a test-only reverse and measures the held cube.
It interrupts BEFORE PostPickupRealign rotates the robot towards home.

run_sweep(io=..., config=..., cube_state=...) is the sensor-only helper and
still requires a cube to have been staged (or an empty and cleared gripper).
Run an empty-gripper baseline separately after ensuring that no cube remains
in the ultrasonic beam. Do not assume an empty reading is a failed grip.

Do not use results for autonomous grip decisions until both sweeps have been
compared. Confirm the positions are mechanically safe for the active robot.
"""

from __future__ import annotations

import math
import statistics

from config import CONFIG


# Webots: LiftUp is +1.0 and configured LiftCarry is 0.0 (approximately
# mid-travel). Sweep the entire logical lift range to test whether the
# held cube enters the ultrasonic beam below the carrying position.
# This extension is for the Webots calibration test, not an approved
# physical-robot travel range.
WEBOTS_POSITIONS = (
    1.0, 0.75, 0.50, 0.25, 0.0,
    -0.25, -0.50, -0.75, -1.0,
)

# Mechanical travel and ground clearance must be checked on the real robot.
# Fill in the tuple after checking its existing LiftUp / LiftCarry positions.
BOBBOT_POSITIONS = None

SAMPLES_PER_POSITION = 5
SETTLE_S = 0.60
INTER_SAMPLE_S = 0.10
RANGE_CHANNELS = (
    ("ultrasonic", "front"),
    ("tof", "front_left"),
    ("tof", "front_right"),
)

# Temporary until PickupObject owns its full reverse/verification sequence.
# Set True ONLY after the full retreat has actually been implemented there;
# never reverse twice. The current PickupObject performs Grab -> LiftUp.
PICKUP_ALREADY_REVERSES = True

# Bound startup attempts so a missing/undetectable cube cannot run forever.
MAX_AUTONOMOUS_TICKS = 4000
MAX_REVERSE_TICKS = 400
NEXT_TICK_SLEEP_S = 0.01


def _is_configured(config, kind, name):
    has_io = getattr(config, "has_io", None)
    if callable(has_io):
        return bool(has_io(kind, name))
    return bool(getattr(config, "io", {}).get(f"{kind}.{name}"))


def _valid_mm(raw):
    if raw is None:
        return None
    # Current SR2026 backend returns a float; tolerate a reading wrapper.
    value = getattr(raw, "distance_mm", raw)
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) and number > 0.0 else None


def _measure_once(io, kind, name):
    try:
        collection = getattr(io, kind)
        return _valid_mm(collection[name])
    except (AttributeError, KeyError, IndexError, TypeError, RuntimeError) as exc:
        print(f"[LIFT_SWEEP][SENSOR_ERROR] {kind}.{name}: {exc}")
        return None


def _stats(readings):
    valid = [value for value in readings if value is not None]
    if not valid:
        return {"valid": 0, "median_mm": None, "min_mm": None,
                "max_mm": None, "spread_mm": None}
    return {
        "valid": len(valid),
        "median_mm": statistics.median(valid),
        "min_mm": min(valid),
        "max_mm": max(valid),
        "spread_mm": max(valid) - min(valid),
    }


def _fmt(value):
    return "None" if value is None else f"{value:.1f}"


def _positions_for(config):
    robot_id = str(getattr(config, "robot_id", ""))
    if robot_id == "webots_robot":
        return WEBOTS_POSITIONS
    if robot_id == "bob_bot":
        if BOBBOT_POSITIONS is None:
            raise ValueError(
                "Set BOBBOT_POSITIONS to mechanically checked servo targets "
                "before running the real-robot sweep."
            )
        return BOBBOT_POSITIONS
    raise ValueError(
        f"No approved lift sweep for robot_id={robot_id!r}; "
        "pass explicitly checked positions to run_sweep()."
    )


def run_sweep(*, io, config, cube_state, positions=None):
    """Move a held or empty lift across targets and log raw sensor data.

    Returns a list of dictionaries. Never changes grip, vacuum or drive.
    The caller is responsible for staging the cube and reversing clear.
    """
    if cube_state not in ("held", "empty"):
        raise ValueError("cube_state must be 'held' or 'empty'")

    if not _is_configured(config, "servo", "lift"):
        raise RuntimeError("No servo.lift in resolved IO configuration")

    channels = [
        (kind, name) for kind, name in RANGE_CHANNELS
        if _is_configured(config, kind, name)
    ]
    if not channels:
        print("[LIFT_SWEEP] No configured front range sensor; no sweep run")
        return []

    targets = tuple(_positions_for(config) if positions is None else positions)
    if not targets or any(
        not isinstance(p, (int, float)) or not math.isfinite(p)
        or not -1.0 <= p <= 1.0 for p in targets
    ):
        raise ValueError("Specify finite, mechanically safe servo targets in [-1, +1]")

    lift = io.servo["lift"]
    robot_id = getattr(config, "robot_id", "unknown")
    carry = getattr(config, "lift_carry_position", None)
    print(f"\n=== LIFT ULTRASONIC SWEEP [{cube_state.upper()}] ===")
    print(f"[LIFT_SWEEP] robot={robot_id} carry={carry} targets={targets}")
    print(f"[LIFT_SWEEP] channels={[f'{a}.{b}' for a, b in channels]}")
    print("[LIFT_SWEEP] PRECONDITION: cube state is physically staged; "
          "robot has already reversed away from the pickup site")

    results = []
    try:
        for position in targets:
            print(f"[LIFT_SWEEP][MOVE] position={position:+.2f}")
            lift.position = float(position)
            io.sleep(SETTLE_S)

            readings = {f"{k}.{n}": [] for k, n in channels}
            for sample in range(SAMPLES_PER_POSITION):
                for kind, name in channels:
                    key = f"{kind}.{name}"
                    value = _measure_once(io, kind, name)
                    readings[key].append(value)
                    print(f"[LIFT_SWEEP][RAW] state={cube_state} "
                          f"position={position:+.2f} sample={sample + 1} "
                          f"channel={key} distance_mm={value}")
                if sample + 1 < SAMPLES_PER_POSITION:
                    io.sleep(INTER_SAMPLE_S)

            for channel, samples in readings.items():
                summary = _stats(samples)
                row = {"robot_id": robot_id, "state": cube_state,
                       "position": float(position), "channel": channel,
                       "samples_mm": samples, **summary}
                results.append(row)
                print(
                    f"[LIFT_SWEEP][SUMMARY] state={cube_state} "
                    f"position={position:+.2f} channel={channel} "
                    f"valid={summary['valid']}/{SAMPLES_PER_POSITION} "
                    f"median_mm={_fmt(summary['median_mm'])} "
                    f"spread_mm={_fmt(summary['spread_mm'])} "
                    f"raw={samples}"
                )
    finally:
        # Restore configured camera-compatible carrying position, even if
        # a measurement fails. Gripper and drivetrain remain untouched.
        if carry is not None:
            lift.position = float(carry)
            io.sleep(SETTLE_S)
            print(f"[LIFT_SWEEP][RESTORE] carry={float(carry):+.2f}")

    print(f"[LIFT_SWEEP][DONE] state={cube_state} rows={len(results)}")
    return results



def run(controller):
    """Run from normal spawn: escape -> find cube -> pickup -> retreat -> sweep.

    The full autonomous program owns InitEscape, selection, approach, alignment,
    final drive, grasp and lift. Stop at the first successful PickupObject,
    before PostPickupRealign can rotate away from the cube. For the current
    implementation, retreat is temporary challenge-only code; the long-term
    owner of retreat and verification is PickupObject.
    """
    from autonomous.auto_sr2026_stage1 import AutoSR2026Stage1
    from config.strategy import STARTUP_SCRIPT, StartupScript
    from primitives.motion import Drive
    from primitives.base import PrimitiveStatus
    from state_machine import RobotState

    config = getattr(controller, "config", CONFIG)

    if STARTUP_SCRIPT != StartupScript.NONE:
        raise RuntimeError(
            "Lift sweep needs STARTUP_SCRIPT=NONE to run InitEscape from spawn"
        )
    if not _is_configured(config, "servo", "lift"):
        raise RuntimeError("servo.lift is not enabled in the resolved profile")
    if not any(_is_configured(config, k, n) for k, n in RANGE_CHANNELS):
        raise RuntimeError("No front ultrasonic/ToF is enabled in this profile")
    # Check positions before moving the robot; Bobbot requires an approved set.
    _positions_for(config)

    print("\n=== AUTONOMOUS LIFT ULTRASONIC CHALLENGE ===")
    print("[LIFT_SWEEP][STAGING] InitEscape -> target selection -> "
          "ApproachObject -> PickupObject")

    autonomous = AutoSR2026Stage1()
    saw_pickup = False
    commitment_mm = None

    for tick in range(MAX_AUTONOMOUS_TICKS):
        controller.tick()

        # While PickupObject is active, preserve its real final-drive amount:
        # auto_sr2026_stage1 clears the behavior when pickup succeeds.
        if autonomous.state == RobotState.PICKUP_OBJECT:
            saw_pickup = True
            current = getattr(autonomous, "behavior", None)
            current_mm = getattr(current, "final_drive_mm", None)
            if current_mm is not None:
                commitment_mm = float(current_mm)

        autonomous.update(controller)

        if autonomous.state == RobotState.POST_PICKUP_REALIGN and saw_pickup:
            # Intercept before the next autonomous tick starts homeward work.
            break

        controller.io.sleep(NEXT_TICK_SLEEP_S)
    else:
        raise RuntimeError(
            f"No successful pickup within {MAX_AUTONOMOUS_TICKS} ticks. "
            "Inspect the preceding autonomous log; lift sweep not run."
        )

    if commitment_mm is None:
        approach_mm = getattr(autonomous, "pickup_distance_mm", None)
        if approach_mm is None:
            raise RuntimeError("Pickup completed but retreat distance is unknown")
        push_mm = float(getattr(config, "final_approach_marker_push", 0.0))
        commitment_mm = max(0.0, float(approach_mm) + push_mm)

    print(f"[LIFT_SWEEP][STAGING] pickup complete "
          f"id={autonomous.last_collected_id} "
          f"commitment={commitment_mm:.1f}mm")

    if not PICKUP_ALREADY_REVERSES:
        if commitment_mm <= 0.0:
            raise RuntimeError("Invalid commitment: refusing to reverse")
        print(f"[LIFT_SWEEP][RETREAT] "
              f"reverse {-commitment_mm:.1f}mm before measuring")
        reverse = Drive(distance_mm=-commitment_mm)
        reverse.start(motion_backend=controller.motion_backend)

        for tick in range(MAX_REVERSE_TICKS):
            controller.tick()
            status = reverse.update(motion_backend=controller.motion_backend)
            if status == PrimitiveStatus.SUCCEEDED:
                print("[LIFT_SWEEP][RETREAT] complete")
                break
            if status == PrimitiveStatus.FAILED:
                raise RuntimeError("Test reverse failed; lift sweep not run")
            controller.io.sleep(NEXT_TICK_SLEEP_S)
        else:
            raise RuntimeError("Test reverse timed out; lift sweep not run")
    else:
        print("[LIFT_SWEEP][RETREAT] already completed inside PickupObject")

    print("[LIFT_SWEEP][STAGING] held cube, clear of original pickup point")
    return run_sweep(io=controller.io, config=config, cube_state="held")
