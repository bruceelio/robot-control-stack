# config/encoders/gobilda_4bar_odometry_pod_32mm.py

"""
goBILDA 4-Bar Odometry Pod, 32 mm wheel.
"""

ENCODER_TYPE = "quadrature"
COUNTS_PER_REV = 2000
UNITS = "mm"
UNITS_PER_REV = 32.0 * 3.141592653589793
DEFAULT_INVERT = False
ZERO_ON_START = True