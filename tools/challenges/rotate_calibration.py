# tools/challenges/rotate_calibration.py

import math
import time


# ==================================================
# Challenge configuration
# ==================================================

# Enable ONE of these at a time.
# TEST_POWER = 0.10
# TEST_POWER = 0.30
TEST_POWER = 0.50

# Positive = counter-clockwise / left.
# Negative = clockwise / right.
TURN_SIGN = 1.0

# Move away from the starting wall before testing rotation.
START_CLEARANCE_MM = 1000.0
START_DRIVE_POWER = 0.65

# Measured Webots drive calibration at power 0.65:
# time_s = m * distance_mm + b
START_DRIVE_M = 0.00124038
START_DRIVE_B = 0.065394

# Continuous rotation duration for each test power.
#
# These are chosen to produce substantially more than 180 degrees,
# but less than a full 360-degree revolution, so headings can be
# unwrapped unambiguously from the starting heading.
RUN_DURATION_BY_POWER = {
    0.10: 5.00,
    0.30: 1.70,
    0.50: 1.00,
}

# Small pause between vision attempts while the motors remain powered.
SAMPLE_DELAY_S = 0.02

# Before rotation starts we want a definite absolute heading so that
# elapsed time and cumulative angle are anchored at (0 s, 0 deg).
START_HEADING_RETRIES = 20
START_HEADING_RETRY_DELAY_S = 0.05

# Ignore extremely tiny angle changes in the fit.
MIN_FIT_ANGLE_DEG = 5.0


# ==================================================
# Vision / localisation helpers
# ==================================================

def _vision_heading(controller):
    """
    Take one fresh camera observation and return:

        (heading_rad, source)

    Localisation heading is in radians.

    Returns None if this frame does not produce an absolute heading.
    """

    now_s = time.time()

    vision_message = controller._get_vision_message(
        camera_name="front",
        now_s=now_s,
    )

    if vision_message is None:
        return None

    obs = controller.localisation.estimate(
        io=controller.io,
        vision_message=vision_message,
        now_s=now_s,
    )

    if obs is None:
        return None

    if not obs.heading_valid:
        return None

    if not obs.is_absolute:
        return None

    if obs.heading is None:
        return None

    return (
        float(obs.heading),
        str(obs.source),
    )


def _initial_heading(controller):
    """
    Obtain the absolute heading which anchors the calibration.

    We wait before movement because a missing starting heading would
    prevent us from measuring cumulative rotation accurately.
    """

    for _ in range(START_HEADING_RETRIES):

        heading = _vision_heading(controller)

        if heading is not None:
            return heading

        controller.io.sleep(
            START_HEADING_RETRY_DELAY_S
        )

    raise RuntimeError(
        "ROTATE_CALIBRATION could not obtain "
        "an absolute starting heading"
    )


def _directed_angle_from_start_deg(
    start_heading_rad,
    current_heading_rad,
    turn_sign,
):
    """
    Return cumulative rotation from the starting heading in the known
    commanded direction.

    The challenge is deliberately kept below one full revolution, so
    modulo 2*pi is sufficient and unambiguous.
    """

    if turn_sign > 0.0:
        delta_rad = (
            current_heading_rad
            - start_heading_rad
        ) % (2.0 * math.pi)

    else:
        delta_rad = (
            start_heading_rad
            - current_heading_rad
        ) % (2.0 * math.pi)

    return math.degrees(delta_rad)


# ==================================================
# Linear fit
# ==================================================

def _fit_time_from_angle(samples):
    """
    Least-squares fit:

        time_s = m * angle_deg + b

    The resulting m and b are directly useful for timed rotation.
    """

    usable = [
        sample
        for sample in samples
        if sample["angle_deg"] >= MIN_FIT_ANGLE_DEG
    ]

    # Add the known starting condition.
    angles = [0.0]
    times = [0.0]

    for sample in usable:
        angles.append(
            float(sample["angle_deg"])
        )
        times.append(
            float(sample["time_s"])
        )

    if len(angles) < 3:
        raise RuntimeError(
            "Need at least two valid non-zero heading samples"
        )

    mean_angle = sum(angles) / len(angles)
    mean_time = sum(times) / len(times)

    denominator = sum(
        (angle - mean_angle) ** 2
        for angle in angles
    )

    if denominator <= 0.0:
        raise RuntimeError(
            "Angle samples have no usable spread"
        )

    m = sum(
        (angle - mean_angle) * (sample_time - mean_time)
        for angle, sample_time in zip(angles, times)
    ) / denominator

    b = mean_time - m * mean_angle

    predicted = [
        m * angle + b
        for angle in angles
    ]

    ss_res = sum(
        (sample_time - predicted_time) ** 2
        for sample_time, predicted_time in zip(times, predicted)
    )

    ss_tot = sum(
        (sample_time - mean_time) ** 2
        for sample_time in times
    )

    r2 = (
        1.0
        if ss_tot <= 0.0
        else 1.0 - ss_res / ss_tot
    )

    return m, b, r2, usable


# ==================================================
# Challenge
# ==================================================

def run(controller):

    io = controller.io

    power = float(TEST_POWER)
    turn_sign = float(TURN_SIGN)

    if power not in RUN_DURATION_BY_POWER:
        raise ValueError(
            f"TEST_POWER={power} has no run duration. "
            f"Available powers: {tuple(RUN_DURATION_BY_POWER)}"
        )

    if turn_sign not in (-1.0, 1.0):
        raise ValueError(
            f"TURN_SIGN must be +1.0 or -1.0, got {turn_sign}"
        )

    run_duration_s = float(
        RUN_DURATION_BY_POWER[power]
    )

    direction_name = (
        "CCW/LEFT"
        if turn_sign > 0.0
        else "CW/RIGHT"
    )

    print()
    print("========================================")
    print(" CONTINUOUS ROTATE CALIBRATION")
    print("========================================")
    print(f"power={power:.2f}")
    print(f"direction={direction_name}")
    print(f"run_duration={run_duration_s:.3f}s")
    print()

    # --------------------------------------------------
    # Move clear of the starting wall
    # --------------------------------------------------

    clearance_time_s = (
        START_DRIVE_M * START_CLEARANCE_MM
        + START_DRIVE_B
    )

    forward_left = (
        START_DRIVE_POWER
        * controller.config.motor_polarity[0]
    )

    forward_right = (
        START_DRIVE_POWER
        * controller.config.motor_polarity[1]
    )

    print(
        "[ROTATE_CAL] CLEARANCE "
        f"distance={START_CLEARANCE_MM:.0f}mm "
        f"power={START_DRIVE_POWER:.2f} "
        f"time={clearance_time_s:.3f}s"
    )

    io.drive["front"].set_power(
        left=forward_left,
        right=forward_right,
    )

    try:
        io.sleep(clearance_time_s)

    finally:
        io.drive["front"].set_power(
            left=0.0,
            right=0.0,
        )

    io.sleep(0.20)

    # --------------------------------------------------
    # Establish absolute starting heading
    # --------------------------------------------------

    start_heading_rad, start_source = (
        _initial_heading(controller)
    )

    print(
        "[ROTATE_CAL] START "
        f"heading={math.degrees(start_heading_rad):+.2f}deg "
        f"source={start_source}"
    )

    # --------------------------------------------------
    # Continuous rotation
    # --------------------------------------------------

    left_power = (
        -turn_sign
        * power
        * controller.config.motor_polarity[0]
    )

    right_power = (
        turn_sign
        * power
        * controller.config.motor_polarity[1]
    )

    samples = []

    rotate_start_s = controller.robot.time()

    io.drive["front"].set_power(
        left=left_power,
        right=right_power,
    )

    try:
        while True:

            elapsed_before_s = (
                controller.robot.time()
                - rotate_start_s
            )

            if elapsed_before_s >= run_duration_s:
                break

            heading = _vision_heading(controller)

            # Timestamp after the fresh vision observation because the
            # simulator may advance while the camera is being sampled.
            elapsed_s = (
                controller.robot.time()
                - rotate_start_s
            )

            if heading is not None:

                heading_rad, source = heading

                angle_deg = (
                    _directed_angle_from_start_deg(
                        start_heading_rad,
                        heading_rad,
                        turn_sign,
                    )
                )

                # Once we approach a complete revolution, modulo heading
                # becomes ambiguous. Stop accepting samples rather than
                # silently wrapping them back toward zero.
                if angle_deg < 350.0:

                    sample = {
                        "time_s": elapsed_s,
                        "angle_deg": angle_deg,
                        "heading_deg": math.degrees(
                            heading_rad
                        ),
                        "source": source,
                    }

                    samples.append(sample)

                    print(
                        "[ROTATE_CAL] SAMPLE "
                        f"t={elapsed_s:.3f}s "
                        f"angle={angle_deg:.2f}deg "
                        f"heading={sample['heading_deg']:+.2f}deg "
                        f"source={source}"
                    )

            io.sleep(SAMPLE_DELAY_S)

    finally:
        io.drive["front"].set_power(
            left=0.0,
            right=0.0,
        )

    actual_run_s = (
        controller.robot.time()
        - rotate_start_s
    )

    # --------------------------------------------------
    # Fit time = m*angle + b
    # --------------------------------------------------

    print()
    print("========================================")
    print(" RESULTS")
    print("========================================")
    print(
        f"actual_rotation_run={actual_run_s:.3f}s"
    )
    print(
        f"valid_heading_samples={len(samples)}"
    )

    try:
        m, b, r2, usable = _fit_time_from_angle(
            samples
        )

    except RuntimeError as exc:
        print(
            f"[ROTATE_CAL] {exc}"
        )
        print("========================================")
        return

    time_180_s = (
        m * 180.0
        + b
    )

    equivalent_deg_s = (
        1.0 / m
        if m > 0.0
        else float("nan")
    )

    max_angle_deg = max(
        sample["angle_deg"]
        for sample in usable
    )

    print(
        f"fit_samples={len(usable) + 1}"
    )

    print(
        f"max_measured_angle={max_angle_deg:.2f}deg"
    )

    print()

    print(
        "time_s = "
        f"{m:.8f} * angle_deg "
        f"+ {b:.6f}"
    )

    print(
        f"m={m:.8f} s/deg"
    )

    print(
        f"b={b:.6f} s"
    )

    print(
        f"equivalent_rate={equivalent_deg_s:.1f} deg/s"
    )

    print(
        f"R^2={r2:.6f}"
    )

    print()
    print(
        f"predicted_time_180={time_180_s:.4f}s"
    )

    print()
    print("Calibration row:")

    print(
        f"# power, m, b   "
        f"({power:.2f}, {m:.8f}, {b:.6f})"
    )

    print()
    print("180-degree row:")

    print(
        f"# power, time_180_s   "
        f"({power:.2f}, {time_180_s:.4f})"
    )

    print("========================================")
