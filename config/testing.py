# config/testing.py

# currently legacy and unused

from .simulation import *

RUN_TESTS = False        # Master switch
TEST_CATEGORY = None    # e.g. "safety", "motion", None = all

SURFACE = "simulation"
ENVIRONMENT = "simulation"

BASE_DRIVE_FACTOR = 0.6
BASE_ROTATE_FACTOR = 0.6
DEBUG = True
