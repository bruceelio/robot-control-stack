# tools/diagnostics/camera_rotation_visibility.py

from __future__ import annotations

import math
from dataclasses import dataclass

from calibration import CALIBRATION
from config import CONFIG


CAMERA_NAME = "front"
DRIVE_GROUP = "front"

# Move clear of the starting wall before testing.
CLEARANCE_DRIVE_MM = 200.0

# Camera sampling / settling.
SETTLE_S = 0.35
BASELINE_DURATION_S = 1.0
SAMPLE_DELAY_S = 0.01

# One nominal full rotation per test.
ROTATION_SWEEP_DEG = 360.0

# Simulation-only stress sweep.
#
# 0.10 -> slowest calibrated region
# 0.20-0.60 -> existing Webots rotation calibration range
# 0.80/1.00 -> deliberate stress tests
ROTATION_POWERS = (
    0.10,
    0.20,
    0.30,
    0.40,
    0.50,
    0.60,
    0.80,
    1.00,
)

# Rotation direction through the semantic 2WD drive.
#
# left / CCW:
#   left wheel backwards
#   right wheel forwards
#
# right / CW:
#   left wheel forwards
#   right wheel backwards
DIRECTIONS = (
    ("left", -1.0, +1.0),
    ("right", +1.0, -1.0),
)


@dataclass
class MarkerObservation:
    marker_id: int
    bearing_deg: float | None
    distance_mm: float | None


@dataclass
class RunResult:
    direction: str
    power: float
    elapsed_s: float
    sample_count: int
    marker_observation_count: int
    unique_ids: set[int]
    mean_interval_s: float | None
    lost_events: int
    reacquired_events: int


def _now(io) -> float:
    """
    Use the resolved IO clock.

    In Webots this gives simulation-time semantics rather than
    accidentally mixing wall-clock time with simulator time.
    """
    return float(io.time())


def _stop_drive(drive) -> None:
    drive.set_power(
        left=0.0,
        right=0.0,
    )


def _read_marker(marker) -> MarkerObservation:
    marker_id = int(marker.id)

    position = getattr(marker, "position", None)

    bearing_deg = None
    distance_mm = None

    if position is not None:
        horizontal_angle = getattr(
            position,
            "horizontal_angle",
            None,
        )

        if horizontal_angle is not None:
            bearing_deg = math.degrees(
                float(horizontal_angle)
            )

        distance = getattr(
            position,
            "distance",
            None,
        )

        if distance is not None:
            distance_mm = float(distance)

    return MarkerObservation(
        marker_id=marker_id,
        bearing_deg=bearing_deg,
        distance_mm=distance_mm,
    )


def _read_camera(camera) -> list[MarkerObservation]:
    observations = [
        _read_marker(marker)
        for marker in camera.see()
    ]

    observations.sort(
        key=lambda obs: obs.marker_id
    )

    return observations


def _format_observation(
    observation: MarkerObservation,
) -> str:

    bearing = (
        "b=?"
        if observation.bearing_deg is None
        else f"b={observation.bearing_deg:+.1f}deg"
    )

    distance = (
        "d=?"
        if observation.distance_mm is None
        else f"d={observation.distance_mm:.0f}mm"
    )

    return (
        f"id={observation.marker_id} "
        f"{bearing} "
        f"{distance}"
    )


def _print_sample(
    *,
    prefix: str,
    elapsed_s: float,
    observations: list[MarkerObservation],
) -> None:

    if observations:
        marker_text = " | ".join(
            _format_observation(obs)
            for obs in observations
        )
    else:
        marker_text = "none"

    print(
        f"{prefix} "
        f"t={elapsed_s:6.3f}s "
        f"markers={len(observations):2d} "
        f"{marker_text}"
    )


def _drive_clearance(
    *,
    io,
    drive,
) -> None:
    """
    Drive forward using the currently resolved Webots timed-drive
    calibration.

    This is only to get the robot away from the starting wall.
    """

    distance_mm = CLEARANCE_DRIVE_MM
    abs_distance_mm = abs(distance_mm)

    if abs_distance_mm < CALIBRATION.drive_switch_mm:
        power = float(
            CALIBRATION.drive_power_short
        )
        m = float(
            CALIBRATION.drive_m_short
        )
        b = float(
            CALIBRATION.drive_b_short
        )
    else:
        power = float(
            CALIBRATION.drive_power_long
        )
        m = float(
            CALIBRATION.drive_m_long
        )
        b = float(
            CALIBRATION.drive_b_long
        )

    duration_s = (
        m * abs_distance_mm + b
    ) * float(
        getattr(CONFIG, "drive_factor", 1.0)
    )

    direction = (
        1.0
        if distance_mm >= 0.0
        else -1.0
    )

    signed_power = direction * power

    print(
        "\n[CAM_ROT][CLEARANCE] "
        f"drive={distance_mm:.0f}mm "
        f"power={signed_power:+.2f} "
        f"duration={duration_s:.3f}s"
    )

    try:
        drive.set_power(
            left=signed_power,
            right=signed_power,
        )

        io.sleep(duration_s)

    finally:
        _stop_drive(drive)

    print(
        "[CAM_ROT][CLEARANCE] complete"
    )

    io.sleep(SETTLE_S)


def _rotation_duration_s(
    power: float,
) -> tuple[float, float]:
    """
    Estimate the duration needed for approximately one 360-degree
    rotation at the requested motor power.

    The current active Webots large-angle rotation calibration is
    used as the reference point.

    For powers other than the calibrated reference power, duration
    is scaled inversely with motor power.

    This is intentionally only a nominal sweep estimate. The test
    measures vision continuity, not rotation-angle accuracy.
    """

    reference_power = float(
        CALIBRATION.rotate_power_large
    )

    reference_duration_s = (
        float(CALIBRATION.rotate_m_large)
        * ROTATION_SWEEP_DEG
        + float(CALIBRATION.rotate_b_large)
    )

    reference_duration_s *= float(
        getattr(CONFIG, "rotate_factor", 1.0)
    )

    duration_s = (
        reference_duration_s
        * reference_power
        / power
    )

    nominal_rate_deg_s = (
        ROTATION_SWEEP_DEG / duration_s
    )

    return duration_s, nominal_rate_deg_s


def _stationary_baseline(
    *,
    io,
    camera,
) -> set[int]:

    print(
        "\n"
        "========================================\n"
        "CAMERA STATIONARY BASELINE\n"
        "========================================"
    )

    start_s = _now(io)

    previous_sample_s = None
    intervals: list[float] = []
    unique_ids: set[int] = set()

    sample_count = 0

    while True:
        now_s = _now(io)

        if now_s - start_s >= BASELINE_DURATION_S:
            break

        observations = _read_camera(camera)

        sample_time_s = _now(io)
        elapsed_s = sample_time_s - start_s

        sample_count += 1

        if previous_sample_s is not None:
            intervals.append(
                sample_time_s - previous_sample_s
            )

        previous_sample_s = sample_time_s

        unique_ids.update(
            obs.marker_id
            for obs in observations
        )

        _print_sample(
            prefix="[CAM_ROT][BASELINE]",
            elapsed_s=elapsed_s,
            observations=observations,
        )

        io.sleep(SAMPLE_DELAY_S)

    mean_interval_s = (
        sum(intervals) / len(intervals)
        if intervals
        else None
    )

    if (
        mean_interval_s is not None
        and mean_interval_s > 0.0
    ):
        rate_hz = 1.0 / mean_interval_s
        timing_text = (
            f"mean_interval={mean_interval_s:.3f}s "
            f"rate={rate_hz:.1f}Hz"
        )
    else:
        timing_text = "mean_interval=? rate=?"

    print(
        "[CAM_ROT][BASELINE][SUMMARY] "
        f"samples={sample_count} "
        f"unique_ids={sorted(unique_ids)} "
        f"{timing_text}"
    )

    return unique_ids


def _run_rotation(
    *,
    io,
    drive,
    camera,
    direction: str,
    left_sign: float,
    right_sign: float,
    power: float,
) -> RunResult:

    duration_s, nominal_rate_deg_s = (
        _rotation_duration_s(power)
    )

    nominal_rate_rad_s = math.radians(
        nominal_rate_deg_s
    )

    left_power = left_sign * power
    right_power = right_sign * power

    print(
        "\n"
        "----------------------------------------"
    )

    print(
        "[CAM_ROT][START] "
        f"direction={direction} "
        f"power={power:.2f} "
        f"left={left_power:+.2f} "
        f"right={right_power:+.2f} "
        f"nominal_duration={duration_s:.3f}s "
        f"nominal_rate={nominal_rate_deg_s:.1f}deg/s "
        f"({nominal_rate_rad_s:.2f}rad/s)"
    )

    start_s = _now(io)

    previous_sample_s = None
    previous_ids: set[int] = set()

    # Most recent observation for every ID.
    last_seen: dict[
        int,
        tuple[float, float | None],
    ] = {}

    intervals: list[float] = []
    unique_ids: set[int] = set()

    sample_count = 0
    marker_observation_count = 0
    lost_events = 0
    reacquired_events = 0

    try:
        drive.set_power(
            left=left_power,
            right=right_power,
        )

        while True:
            before_s = _now(io)

            if before_s - start_s >= duration_s:
                break

            observations = _read_camera(camera)

            sample_s = _now(io)
            elapsed_s = sample_s - start_s

            sample_count += 1
            marker_observation_count += len(
                observations
            )

            if previous_sample_s is not None:
                intervals.append(
                    sample_s - previous_sample_s
                )

            previous_sample_s = sample_s

            current_ids = {
                obs.marker_id
                for obs in observations
            }

            unique_ids.update(current_ids)

            observation_by_id = {
                obs.marker_id: obs
                for obs in observations
            }

            # ------------------------------------------
            # IDs which disappeared since last sample.
            # ------------------------------------------

            for marker_id in sorted(
                previous_ids - current_ids
            ):
                previous = last_seen.get(
                    marker_id
                )

                previous_bearing = (
                    previous[1]
                    if previous is not None
                    else None
                )

                bearing_text = (
                    "?"
                    if previous_bearing is None
                    else f"{previous_bearing:+.1f}deg"
                )

                print(
                    "[CAM_ROT][LOST] "
                    f"direction={direction} "
                    f"power={power:.2f} "
                    f"id={marker_id} "
                    f"last_bearing={bearing_text}"
                )

                lost_events += 1

            # ------------------------------------------
            # IDs which appeared since last sample.
            # ------------------------------------------

            for marker_id in sorted(
                current_ids - previous_ids
            ):
                observation = observation_by_id[
                    marker_id
                ]

                previous = last_seen.get(
                    marker_id
                )

                # First appearance in this rotation.
                if previous is None:
                    print(
                        "[CAM_ROT][FOUND] "
                        f"direction={direction} "
                        f"power={power:.2f} "
                        f"{_format_observation(observation)}"
                    )

                # Seen previously, disappeared, and
                # has now appeared again.
                else:
                    previous_time_s, previous_bearing = (
                        previous
                    )

                    gap_s = (
                        sample_s - previous_time_s
                    )

                    old_bearing_text = (
                        "?"
                        if previous_bearing is None
                        else f"{previous_bearing:+.1f}deg"
                    )

                    new_bearing_text = (
                        "?"
                        if observation.bearing_deg
                        is None
                        else (
                            f"{observation.bearing_deg:+.1f}deg"
                        )
                    )

                    print(
                        "[CAM_ROT][REACQUIRED] "
                        f"direction={direction} "
                        f"power={power:.2f} "
                        f"id={marker_id} "
                        f"gap={gap_s:.3f}s "
                        f"previous_bearing="
                        f"{old_bearing_text} "
                        f"new_bearing="
                        f"{new_bearing_text}"
                    )

                    reacquired_events += 1

            # Update last-seen state only for markers
            # actually visible in this camera sample.
            for observation in observations:
                last_seen[
                    observation.marker_id
                ] = (
                    sample_s,
                    observation.bearing_deg,
                )

            _print_sample(
                prefix=(
                    f"[CAM_ROT][SAMPLE] "
                    f"dir={direction} "
                    f"power={power:.2f}"
                ),
                elapsed_s=elapsed_s,
                observations=observations,
            )

            previous_ids = current_ids

            # Small simulator-time advance.
            #
            # camera.see() itself has a cost, so this is
            # deliberately much shorter than the expected
            # camera update interval.
            io.sleep(SAMPLE_DELAY_S)

    finally:
        _stop_drive(drive)

    end_s = _now(io)
    elapsed_s = end_s - start_s

    mean_interval_s = (
        sum(intervals) / len(intervals)
        if intervals
        else None
    )

    result = RunResult(
        direction=direction,
        power=power,
        elapsed_s=elapsed_s,
        sample_count=sample_count,
        marker_observation_count=(
            marker_observation_count
        ),
        unique_ids=unique_ids,
        mean_interval_s=mean_interval_s,
        lost_events=lost_events,
        reacquired_events=reacquired_events,
    )

    _print_summary(result)

    io.sleep(SETTLE_S)

    return result


def _print_summary(
    result: RunResult,
) -> None:

    if (
        result.mean_interval_s is not None
        and result.mean_interval_s > 0.0
    ):
        rate_hz = (
            1.0 / result.mean_interval_s
        )

        timing_text = (
            f"mean_interval="
            f"{result.mean_interval_s:.3f}s "
            f"camera_rate={rate_hz:.1f}Hz"
        )
    else:
        timing_text = (
            "mean_interval=? camera_rate=?"
        )

    print(
        "[CAM_ROT][SUMMARY] "
        f"direction={result.direction} "
        f"power={result.power:.2f} "
        f"elapsed={result.elapsed_s:.3f}s "
        f"samples={result.sample_count} "
        f"marker_observations="
        f"{result.marker_observation_count} "
        f"unique_ids="
        f"{sorted(result.unique_ids)} "
        f"lost_events="
        f"{result.lost_events} "
        f"reacquired_events="
        f"{result.reacquired_events} "
        f"{timing_text}"
    )


def run(
    robot=None,
    io=None,
) -> None:

    print(
        "\n"
        "========================================\n"
        "WEBOTS CAMERA ROTATION VISIBILITY\n"
        "========================================"
    )

    environment = str(
        getattr(CONFIG, "environment", "")
    ).lower()

    if environment != "simulation":
        raise RuntimeError(
            "camera_rotation_visibility is "
            "simulation-only"
        )

    if io is None:
        raise RuntimeError(
            "camera_rotation_visibility requires "
            "the already-resolved io instance"
        )

    drive = io.drive[DRIVE_GROUP]

    camera = io.cameras().get(
        CAMERA_NAME
    )

    if camera is None:
        raise RuntimeError(
            f"Camera {CAMERA_NAME!r} "
            "is not available"
        )

    print(
        "[CAM_ROT] simulation-only diagnostic"
    )
    print(
        f"[CAM_ROT] clearance="
        f"{CLEARANCE_DRIVE_MM:.0f}mm"
    )
    print(
        f"[CAM_ROT] powers="
        f"{list(ROTATION_POWERS)}"
    )
    print(
        f"[CAM_ROT] sweep="
        f"{ROTATION_SWEEP_DEG:.0f}deg "
        "per test"
    )

    reference_ids: dict[
        str,
        set[int],
    ] = {}

    try:
        # ------------------------------------------
        # 1. Move away from the wall.
        # ------------------------------------------

        _drive_clearance(
            io=io,
            drive=drive,
        )

        # ------------------------------------------
        # 2. Establish stationary vision behaviour.
        # ------------------------------------------

        baseline_ids = _stationary_baseline(
            io=io,
            camera=camera,
        )

        print(
            "[CAM_ROT] stationary baseline IDs: "
            f"{sorted(baseline_ids)}"
        )

        # ------------------------------------------
        # 3. Rotation visibility sweep.
        # ------------------------------------------

        for power in ROTATION_POWERS:

            for (
                direction,
                left_sign,
                right_sign,
            ) in DIRECTIONS:

                result = _run_rotation(
                    io=io,
                    drive=drive,
                    camera=camera,
                    direction=direction,
                    left_sign=left_sign,
                    right_sign=right_sign,
                    power=power,
                )

                # Slowest run becomes the reference
                # set for this direction.
                if direction not in reference_ids:
                    reference_ids[
                        direction
                    ] = set(
                        result.unique_ids
                    )

                    print(
                        "[CAM_ROT][REFERENCE] "
                        f"direction={direction} "
                        f"power={power:.2f} "
                        f"ids="
                        f"{sorted(result.unique_ids)}"
                    )

                    continue

                reference = reference_ids[
                    direction
                ]

                missing = sorted(
                    reference
                    - result.unique_ids
                )

                extra = sorted(
                    result.unique_ids
                    - reference
                )

                print(
                    "[CAM_ROT][COMPARE] "
                    f"direction={direction} "
                    f"power={power:.2f} "
                    f"missing_vs_0.10="
                    f"{missing} "
                    f"extra_vs_0.10="
                    f"{extra}"
                )

    finally:
        _stop_drive(drive)

    print(
        "\n"
        "========================================\n"
        "CAMERA ROTATION VISIBILITY COMPLETE\n"
        "========================================"
    )