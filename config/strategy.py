# config/strategy.py

from enum import Enum, auto

class RunMode(Enum):
    NORMAL = auto()
    TESTS = auto()
    DIAGNOSTICS = auto()

class StartupScript(Enum):
    NONE = auto()
    BASIC_GRAB = auto()
    ACIDIC_GRAB = auto()  # NEW

RUN_MODE = RunMode.NORMAL
# RUN_MODE = RunMode.TESTS
# RUN_MODE = RunMode.DIAGNOSTICS

# STARTUP_SCRIPT = StartupScript.NONE
STARTUP_SCRIPT = StartupScript.BASIC_GRAB
# STARTUP_SCRIPT = StartupScript.ACIDIC_GRAB

# DEFAULT_TARGET_KIND = "acidic"  # "acidic" or "basic"
DEFAULT_TARGET_KIND = "basic"  # "acidic" or "basic"
