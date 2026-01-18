# config/strategy.py

from enum import Enum, auto

class RunMode(Enum):
    NORMAL = auto()
    TESTS = auto()
    DIAGNOSTICS = auto()

RUN_MODE = RunMode.NORMAL
# RUN_MODE = RunMode.TESTS
# RUN_MODE = RunMode.DIAGNOSTICS

DEFAULT_TARGET_KIND = "basic"  # "acidic" or "basic"

