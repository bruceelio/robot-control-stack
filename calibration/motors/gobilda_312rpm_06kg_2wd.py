# calibration/motors/gobilda_312rpm_06kg_2wd.py

"""
Calibration profile for motors

The following was tested on bob_bot with 2wd and omni 3rd wheel
"""

# --------------------------------------------------
# Timed Drive calibration
# --------------------------------------------------

DRIVE_SWITCH_MM = 800

# Power levels (open-loop)
DRIVE_POWER_SHORT = 0.20
DRIVE_POWER_LONG  = 0.35

# Distance → time calibration
DRIVE_M_SHORT = 0.00421
DRIVE_B_SHORT = -0.0063

DRIVE_M_LONG  = 0.00208
DRIVE_B_LONG  = -0.030

# =========================
# Timed Rotation Calibration
# =========================

ROTATE_SWITCH_DEG = 10  # 30 deg is ideal?

# Small-angle rotations (precision)
ROTATE_POWER_SMALL = 0.10
ROTATE_M_SMALL = 0.025
ROTATE_B_SMALL = 0.0

# Large-angle rotations (momentum) (if under roting need to increase)
ROTATE_POWER_LARGE = 0.20
ROTATE_M_LARGE = 0.015   # sec/deg (greater number is more time turning per deg)
ROTATE_B_LARGE = -0.0413

# Rotation calibration
# ROTATE_POWER = 0.50
# ROTATE_M = 0.0051
# ROTATE_B = 0.15

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


# =========================
# Voltage Compensation
# =========================

# Voltage at which this drivetrain calibration was established.
VOLTAGE_REFERENCE = 14.17

# Low-power compensation
# scale = (reference_voltage / actual_voltage) ** A
VOLTAGE_LOW_MODEL = "exponential"
VOLTAGE_LOW_A = 1.0

# High-power compensation
# scale = (reference_voltage / actual_voltage) ** A
VOLTAGE_HIGH_MODEL = "exponential"
VOLTAGE_HIGH_A = 2.0
