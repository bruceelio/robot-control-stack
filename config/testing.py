# config/testing.py

from .simulation import *

# Make testing safer
SURFACE = "simulation"
ENVIRONMENT = "simulation"

# Slow everything down
BASE_DRIVE_FACTOR = 0.6
BASE_ROTATE_FACTOR = 0.6

# Verbose / debug flags (future)
DEBUG = True
