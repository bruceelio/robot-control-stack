# config/encoders/gobilda_yellowjacket_312rpm.py

"""
goBILDA Yellow Jacket motor with encoder, 312 RPM variant.

UNITS_PER_REV is expressed at the output shaft.
Adjust if your project standardises this differently.
"""

ENCODER_TYPE = "quadrature"
UNITS = "rev"
COUNTS_PER_REV = 1.0   # replace with actual output-shaft counts/rev for this model
UNITS_PER_REV = 1.0
DEFAULT_INVERT = False
ZERO_ON_START = True