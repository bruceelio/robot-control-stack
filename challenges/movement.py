# challenges/movement.py

from config import CONFIG
from calibration import CALIBRATION


def run(controller):
    io = controller.io

    # -------------------------
    # Drive forward 1550 mm
    # -------------------------

    distance_mm = 1550

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

    battery_voltage = io.voltage["battery"].volts

    voltage_ratio = CONFIG.battery_voltage_nominal / battery_voltage

    if power <= 0.20:
        exponent = 1.0
    elif power >= 0.35:
        exponent = 2.0
    else:
        exponent = 1.0 + ((power - 0.20) / (0.35 - 0.20))

    power = power * (voltage_ratio ** exponent)
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
    # Rotate 45 degrees
    # -------------------------

    angle_deg = 45

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