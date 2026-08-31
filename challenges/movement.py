# challenges/movement.py

from config import CONFIG
from calibration import CALIBRATION




def run(controller):
    io = controller.io

    battery_voltage = io.voltage["battery"].volts
    print(f"Battery voltage for movement: {battery_voltage:.2f} V")

    # -------------------------
    # Drive forward x mm
    # -------------------------

    distance_mm = 4000

    if distance_mm < CALIBRATION.drive_switch_mm:
        power = CALIBRATION.drive_power_short
        duration_s = (
            CALIBRATION.drive_m_short * distance_mm
            + CALIBRATION.drive_b_short
        )
    else:
        power = CALIBRATION.drive_power_long
        duration_s = (
            CALIBRATION.drive_m_long * distance_mm
            + CALIBRATION.drive_b_long
        )

    duration_s = duration_s * CONFIG.drive_factor

    dv = battery_voltage - CALIBRATION.voltage_reference

    if power <= CALIBRATION.drive_power_short:
        model = CALIBRATION.voltage_low_model
        a = CALIBRATION.voltage_low_a
        b = CALIBRATION.voltage_low_b
    else:
        model = CALIBRATION.voltage_high_model
        a = CALIBRATION.voltage_high_a
        b = CALIBRATION.voltage_high_b

    if model == "linear":
        voltage_multiplier = 1.0 + a * dv

    elif model == "quadratic":
        voltage_multiplier = 1.0 + a * dv + b * dv ** 2

    elif model == "exponential":
        voltage_multiplier = (
                                     CALIBRATION.voltage_reference / battery_voltage
                             ) ** a

    else:
        raise RuntimeError(
            f"Unknown voltage compensation model: {model}"
        )

    print(
        f"Battery voltage: {battery_voltage:.2f} V, "
        f"model: {model}, "
        f"voltage multiplier: {voltage_multiplier:.3f}, "
        f"nominal power: {power:.3f}"
    )

    power = power * voltage_multiplier
    power = min(power, CONFIG.max_motor_power)

    left_power = power * CONFIG.motor_polarity[0]
    right_power = power * CONFIG.motor_polarity[1]

    io.drive["front"].set_power(
        left=left_power,
        right=right_power,
    )

    io.sleep(duration_s)

    io.drive["front"].set_power(
        left=0.0,
        right=0.0,
    )


    # -------------------------
    # Rotate x degrees
    # -------------------------

    angle_deg = 180

    if angle_deg < CALIBRATION.rotate_switch_deg:
        power = CALIBRATION.rotate_power_small
        duration_s = (
            CALIBRATION.rotate_m_small * angle_deg
            + CALIBRATION.rotate_b_small
        )
    else:
        power = CALIBRATION.rotate_power_large
        duration_s = (
            CALIBRATION.rotate_m_large * angle_deg
            + CALIBRATION.rotate_b_large
        )

    duration_s = duration_s * CONFIG.rotate_factor

    dv = battery_voltage - CALIBRATION.voltage_reference

    if power <= CALIBRATION.drive_power_short:
        model = CALIBRATION.voltage_low_model
        a = CALIBRATION.voltage_low_a
        b = CALIBRATION.voltage_low_b
    else:
        model = CALIBRATION.voltage_high_model
        a = CALIBRATION.voltage_high_a
        b = CALIBRATION.voltage_high_b

    if model == "linear":
        voltage_multiplier = 1.0 + a * dv

    elif model == "quadratic":
        voltage_multiplier = 1.0 + a * dv + b * dv ** 2

    elif model == "exponential":
        voltage_multiplier = (
                                     CALIBRATION.voltage_reference / battery_voltage
                             ) ** a

    else:
        raise RuntimeError(
            f"Unknown voltage compensation model: {model}"
        )

    print(
        f"Battery voltage: {battery_voltage:.2f} V, "
        f"model: {model}, "
        f"voltage multiplier: {voltage_multiplier:.3f}, "
        f"nominal power: {power:.3f}"
    )

    power = power * voltage_multiplier
    power = min(power, CONFIG.max_motor_power)

    direction = CONFIG.rotation_sign

    left_power = direction * power * CONFIG.motor_polarity[0]
    right_power = -direction * power * CONFIG.motor_polarity[1]

    io.drive["front"].set_power(
        left=left_power,
        right=right_power,
    )

    io.sleep(duration_s)

    io.drive["front"].set_power(
        left=0.0,
        right=0.0,
    )