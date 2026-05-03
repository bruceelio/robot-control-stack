# config/encoders/gobilda_yellowjacket_6000rpm.py

"""
goBILDA Yellow Jacket motor with encoder, 6000 RPM variant.

UNITS_PER_REV is expressed at the output shaft.
Adjust if your project standardises this differently.
"""

ENCODER_TYPE = "quadrature"
UNITS = "rev"
COUNTS_PER_REV = 112   # = 112 (quadrature) x 1 (gear ratio)
UNITS_PER_REV = 1.0
DEFAULT_INVERT = False
ZERO_ON_START = True