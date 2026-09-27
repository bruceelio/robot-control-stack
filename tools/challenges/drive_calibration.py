# tools/challenges/drive_calibration.py

import math
import time


# Change this between simulator runs.
TEST_POWER = 0.85

CHECKPOINTS_S = (
    0.5,
    1.0,
    1.5,
    2.0,
)


def _vision_pose(controller):
    """
    Take one fresh camera observation and return an absolute
    marker-derived pose as (x_mm, y_mm, source).

    Returns None if this frame does not produce an absolute pose.
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

    if not obs.position_valid:
        return None

    if not obs.is_absolute:
        return None

    if obs.x is None or obs.y is None:
        return None

    return (
        float(obs.x),
        float(obs.y),
        str(obs.source),
    )


def _initial_vision_pose(controller):
    """
    Before movement we can afford to wait for a valid arena-marker pose.
    """

    for attempt in range(20):
        pose = _vision_pose(controller)

        if pose is not None:
            return pose

        controller.io.sleep(0.05)

    raise RuntimeError(
        "DRIVE_CALIBRATION could not obtain "
        "an absolute arena-marker start pose"
    )


def _fit_time_from_distance(samples):
    """
    Least-squares fit:

        time_s = m * distance_mm + b
    """

    if len(samples) < 2:
        raise RuntimeError(
            "Need at least two valid distance/time samples"
        )

    distances = [
        float(sample["distance_mm"])
        for sample in samples
    ]

    times = [
        float(sample["time_s"])
        for sample in samples
    ]

    mean_d = sum(distances) / len(distances)
    mean_t = sum(times) / len(times)

    denominator = sum(
        (d - mean_d) ** 2
        for d in distances
    )

    if denominator <= 0.0:
        raise RuntimeError(
            "Distance samples have no usable spread"
        )

    m = sum(
        (d - mean_d) * (t - mean_t)
        for d, t in zip(distances, times)
    ) / denominator

    b = mean_t - m * mean_d

    # Goodness of fit.
    predicted = [
        m * d + b
        for d in distances
    ]

    ss_res = sum(
        (t - p) ** 2
        for t, p in zip(times, predicted)
    )

    ss_tot = sum(
        (t - mean_t) ** 2
        for t in times
    )

    r2 = (
        1.0
        if ss_tot <= 0.0
        else 1.0 - ss_res / ss_tot
    )

    return m, b, r2


def run(controller):

    io = controller.io

    power = float(TEST_POWER)

    if not (0.0 < power <= 1.0):
        raise ValueError(
            f"Invalid TEST_POWER={power}"
        )

    print()
    print("========================================")
    print(" DRIVE CALIBRATION")
    print("========================================")
    print(f"power={power:.3f}")
    print(f"checkpoints={CHECKPOINTS_S}")
    print()

    # ------------------------------------------
    # Establish marker-derived start position
    # ------------------------------------------

    start_pose = _initial_vision_pose(controller)

    start_x, start_y, start_source = start_pose

    print(
        "[DRIVE_CAL] START "
        f"x={start_x:.1f} "
        f"y={start_y:.1f} "
        f"source={start_source}"
    )

    # ------------------------------------------
    # Start continuous forward drive
    # ------------------------------------------

    left_power = (
        power * controller.config.motor_polarity[0]
    )

    right_power = (
        power * controller.config.motor_polarity[1]
    )

    io.drive["front"].set_power(
        left=left_power,
        right=right_power,
    )

    samples = []

    drive_start_s = controller.robot.time()

    try:
        for checkpoint_s in CHECKPOINTS_S:

            elapsed_s = (
                    controller.robot.time()
                    - drive_start_s
            )

            remaining_s = checkpoint_s - elapsed_s

            if remaining_s > 0.0:
                io.sleep(remaining_s)

            pose = _vision_pose(controller)

            # Measure AFTER the camera observation because
            # the motors remained powered during that operation.
            actual_time_s = (
                    controller.robot.time()
                    - drive_start_s
            )

            if pose is None:
                print(
                    "[DRIVE_CAL] "
                    f"target={checkpoint_s:.1f}s "
                    f"actual={actual_time_s:.3f}s "
                    "NO ABSOLUTE VISION POSE"
                )
                continue

            x, y, source = pose

            dx = x - start_x
            dy = y - start_y

            distance_mm = math.hypot(
                dx,
                dy,
            )

            samples.append(
                {
                    "time_s": actual_time_s,
                    "distance_mm": distance_mm,
                    "x_mm": x,
                    "y_mm": y,
                }
            )

            print(
                "[DRIVE_CAL] "
                f"target={checkpoint_s:.1f}s "
                f"actual={actual_time_s:.3f}s "
                f"distance={distance_mm:.1f}mm "
                f"x={x:.1f} "
                f"y={y:.1f} "
                f"source={source}"
            )

    finally:
        io.drive["front"].set_power(
            left=0.0,
            right=0.0,
        )



    # ------------------------------------------
    # Fit t = m*d + b
    # ------------------------------------------

    m, b, r2 = _fit_time_from_distance(
        samples
    )

    velocity_mm_s = (
        1.0 / m
        if m > 0.0
        else float("nan")
    )

    print()
    print("========================================")
    print(" RESULTS")
    print("========================================")

    for sample in samples:
        print(
            f"{sample['time_s']:.3f}s\t"
            f"{sample['distance_mm']:.1f}mm"
        )

    print()
    print(
        "time_s = "
        f"{m:.8f} * distance_mm "
        f"+ {b:.6f}"
    )

    print(
        f"m={m:.8f} s/mm"
    )

    print(
        f"b={b:.6f} s"
    )

    print(
        f"steady_velocity={velocity_mm_s:.1f} mm/s"
    )

    print(
        f"R^2={r2:.6f}"
    )

    print()
    print("Calibration row:")
    print(
        f"# power, m, b   "
        f"({power:.2f}, {m:.8f}, {b:.6f})"
    )

    print("Velocity-curve row:")
    print(
        f"({power:.2f}, {velocity_mm_s:.1f}),"
    )

    print("========================================")