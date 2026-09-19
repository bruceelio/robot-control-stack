# calibration/motors/webots_2026_2wd.py

"""
Timed dead_reckoning calibration for the Webots 2026 2WD robot.

Motion values migrated from the former SR1 calibration profile.
"""

# --------------------------------------------------
# Timed Drive Calibration
# --------------------------------------------------

DRIVE_SWITCH_MM = 800

# Power levels (open-loop)
DRIVE_POWER_SHORT = 0.65
DRIVE_POWER_LONG = 0.85

# Distance -> time calibration
# t = m * distance_mm + b
DRIVE_M_SHORT = 0.00133
DRIVE_B_SHORT = 0.06

DRIVE_M_LONG = 0.00092
DRIVE_B_LONG = 0.06


# --------------------------------------------------
# Timed Rotation Calibration
# --------------------------------------------------

ROTATE_SWITCH_DEG = 7.6

# The former SR1 calibration used one rotation model.
# Use the same model on both sides of the divider for now.

ROTATE_POWER_SMALL = 0.55
ROTATE_M_SMALL = 0.00375
ROTATE_B_SMALL = 0.0

ROTATE_POWER_LARGE = 0.55
ROTATE_M_LARGE = 0.00375
ROTATE_B_LARGE = 0.0

# --------------------------------------------------
# Drive velocity calibration
# --------------------------------------------------

# Steady-state drive velocity calibration:
# (motor_power, velocity_mm_s)
#
# Currently derived from the existing timed-dead_reckoning calibration.
# Additional measured points can be added later without changing
# the velocity backend interface.
DRIVE_VELOCITY_CURVE = (
    (0.0, 0.0),
    (DRIVE_POWER_SHORT, 1.0 / DRIVE_M_SHORT),
    (DRIVE_POWER_LONG,  1.0 / DRIVE_M_LONG),
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
