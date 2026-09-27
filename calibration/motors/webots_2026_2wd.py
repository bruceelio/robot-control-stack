# calibration/motors/webots_2026_2wd.py

"""
Timed dead_reckoning calibration for the Webots 2026 2WD robot.

Motion values migrated from the former SR1 calibration profile.
"""

# --------------------------------------------------
# Timed Drive Calibration
# --------------------------------------------------

DRIVE_SWITCH_MM = 1000

# Power levels (open-loop)
DRIVE_POWER_SHORT = 0.30
DRIVE_POWER_LONG = 0.85

# Distance -> time calibration
# t = m * distance_mm + b

# power, m, b   (0.10, 0.00817990, -0.006319)
# power, m, b   (0.20, 0.00402356, 0.012701)
# power, m, b   (0.30, 0.00270106, 0.023399)
# power, m, b   (0.40, 0.00202058, 0.032628)
# power, m, b   (0.50, 0.00162047, 0.041880)
# power, m, b   (0.60, 0.00134306, 0.058942)
# power, m, b   (0.65, 0.00124038, 0.065394)
# power, m, b   (0.75, 0.00107900, 0.069456)
# power, m, b   (0.85, 0.00094816, 0.083846)


DRIVE_M_SHORT = 0.00270106
DRIVE_B_SHORT = 0.023399

DRIVE_M_LONG = 0.00094816
DRIVE_B_LONG = 0.083846


# --------------------------------------------------
# Timed Rotation Calibration
# --------------------------------------------------

ROTATE_SWITCH_DEG = 7.6

# power, m, b   (0.10, 0.02113341, 0.013277)
# power, m, b   (0.20, 0.01061071, 0.016671)
# power, m, b   (0.30, 0.00711593, 0.020910)
# power, m, b   (0.40, 0.00503003, 0.029327)
# power, m, b   (0.50, 0.00406417, 0.024977)
# power, m, b   (0.60, 0.00343107, 0.027321)


ROTATE_POWER_SMALL = 0.2
ROTATE_M_SMALL = 0.01061071
ROTATE_B_SMALL = 0.016671

ROTATE_POWER_LARGE = 0.4
ROTATE_M_LARGE = 0.00503003
ROTATE_B_LARGE = 0.029327

# --------------------------------------------------
# Drive velocity calibration
# --------------------------------------------------

# Steady-state drive velocity calibration:
# (motor_power, velocity_mm_s)
#
# Independent of timed-drive SHORT/LONG settings.
# These initial points preserve the previous Webots velocity calibration.
# Additional measured points should be added across the usable power range.

DRIVE_VELOCITY_CURVE = (
    (0.00, 0.0),
    (0.10, 122.3),
    (0.20, 248.5),
    (0.30, 370.2),
    (0.40, 494.9),
    (0.50, 617.1),
    (0.60, 744.6),
    (0.65, 806.2),
    (0.75, 926.8),
    (0.85, 1054.7),
)

# --------------------------------------------------
# Voltage Compensation
# --------------------------------------------------

# Webots dead_reckoning does not require battery-voltage compensation.
# These neutral values preserve the current calibration interface
# without changing commanded motor power.

VOLTAGE_REFERENCE = 14.17

VOLTAGE_LOW_MODEL = "linear"
VOLTAGE_LOW_A = 0.0

VOLTAGE_HIGH_MODEL = "linear"
VOLTAGE_HIGH_A = 0.0
