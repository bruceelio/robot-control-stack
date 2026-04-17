# config/strategy.py

from enum import Enum, auto


class RobotProfile(Enum):
    SIMULATION = "simulation"
    SR1 = "sr1"
    BOB_BOT = "bob_bot"


class RunMode(Enum):
    NORMAL = auto()
    TESTS = auto()
    DIAGNOSTICS = auto()


class StartupScript(Enum):
    NONE = auto()
    BASIC_GRAB = auto()
    ACIDIC_GRAB = auto()


ROBOT_PROFILE = RobotProfile.BOB_BOT
# ROBOT_PROFILE = RobotProfile.SIMULATION
# ROBOT_PROFILE = RobotProfile.SR1


RUN_MODE = RunMode.NORMAL
# RUN_MODE = RunMode.TESTS
# RUN_MODE = RunMode.DIAGNOSTICS


STARTUP_SCRIPT = StartupScript.NONE
# STARTUP_SCRIPT = StartupScript.BASIC_GRAB
# STARTUP_SCRIPT = StartupScript.ACIDIC_GRAB


# DEFAULT_TARGET_KIND = "acidic"
DEFAULT_TARGET_KIND = "basic"

START_BASE = 0
START_SLOT = 1