# config/encoders/gobilda_yellowjacket_312rpm.py

"""
goBILDA Yellow Jacket motor with encoder, 312 RPM variant.

COUNTS_PER_REV is expressed at the gearbox output shaft.
Verified on the robot using the Mega encoder decoder.
"""

ENCODER_TYPE = "quadrature"
UNITS = "rev"

COUNTS_PER_REV = 537.6   # = 28 counts/rev x 19.2 (gear ratio)

UNITS_PER_REV = 1.0
DEFAULT_INVERT = False
ZERO_ON_START = True