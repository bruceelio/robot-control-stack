# calibration/motors/webots_2026_2wd.py

"""
Timed motion calibration for the Webots 2026 2WD robot.

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

DRIVE_M_LONG = 0.00112
DRIVE_B_LONG = 0.09


# --------------------------------------------------
# Timed Rotation Calibration
# --------------------------------------------------

ROTATE_SWITCH_DEG = 30

# The former SR1 calibration used one rotation model.
# Use the same model on both sides of the divider for now.

ROTATE_POWER_SMALL = 0.55
ROTATE_M_SMALL = 0.0056
ROTATE_B_SMALL = 0.18

ROTATE_POWER_LARGE = 0.55
ROTATE_M_LARGE = 0.0056
ROTATE_B_LARGE = 0.18


# --------------------------------------------------
# Voltage Compensation
# --------------------------------------------------

# Webots motion does not require battery-voltage compensation.
# These neutral values preserve the current calibration interface
# without changing commanded motor power.

VOLTAGE_REFERENCE = 14.17

VOLTAGE_LOW_MODEL = "linear"
VOLTAGE_LOW_A = 0.0

VOLTAGE_HIGH_MODEL = "linear"
VOLTAGE_HIGH_A = 0.0
