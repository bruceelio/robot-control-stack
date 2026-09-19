# calibration/motors/dfrobot_254rpm_06kg_2wd_hogback94.py

# --------------------------------------------------
# Timed Drive calibration
# --------------------------------------------------

DRIVE_SWITCH_MM = 800

# Power levels (open-loop)
DRIVE_POWER_SHORT = 0.30
DRIVE_POWER_LONG  = 0.60

# Distance → time calibration
DRIVE_M_SHORT = 0.0032
DRIVE_B_SHORT = -0.0016

DRIVE_M_LONG  = 0.00153
DRIVE_B_LONG  = -0.0066

# =========================
# Timed Rotation Calibration
# =========================

ROTATE_SWITCH_DEG = 10  # 30 deg is ideal?

# Small-angle rotations (precision)
ROTATE_POWER_SMALL = 0.20
ROTATE_M_SMALL = 0.096
ROTATE_B_SMALL = 0.0

# Large-angle rotations (momentum) (if under roting need to increase)
ROTATE_POWER_LARGE = 0.30
ROTATE_M_LARGE = 0.0108   # sec/deg (greater number is more time turning per deg)
ROTATE_B_LARGE = 0.0


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


# Voltage at which the drive/rotation calibration above was established.
VOLTAGE_REFERENCE = 13.15

# Linear compensation:
# multiplier = 1 + A*dv
# Quadratic compensation:
# multiplier = 1 + A*dv + B*dv^2

# 0.30 nominal power
VOLTAGE_LOW_MODEL = "quadratic"
VOLTAGE_LOW_A = -0.1208
VOLTAGE_LOW_B = -0.04912

# 0.60 nominal power
VOLTAGE_HIGH_MODEL = "linear"
VOLTAGE_HIGH_A = -0.0623