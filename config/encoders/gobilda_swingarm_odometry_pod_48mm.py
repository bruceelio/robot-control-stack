# config/encoders/gobilda_swingarm_odometry_pod_48mm.py

"""
goBILDA Swingarm Odometry Pod, 48 mm wheel.
"""

ENCODER_TYPE = "quadrature"
COUNTS_PER_REV = 2000
UNITS = "mm"
UNITS_PER_REV = 48.0 * 3.141592653589793
DEFAULT_INVERT = False
ZERO_ON_START = True