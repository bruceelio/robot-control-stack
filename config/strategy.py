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

class MatchZoneSource(Enum):
    AUTO = "auto"
    SR = "sr"
    USB = "usb"
    FIXED = "fixed"


ROBOT_PROFILE = RobotProfile.BOB_BOT
# ROBOT_PROFILE = RobotProfile.SIMULATION         # Run this for WeBots
# ROBOT_PROFILE = RobotProfile.SR1


RUN_MODE = RunMode.NORMAL
# RUN_MODE = RunMode.TESTS
# RUN_MODE = RunMode.DIAGNOSTICS


STARTUP_SCRIPT = StartupScript.NONE
# STARTUP_SCRIPT = StartupScript.BASIC_GRAB
# STARTUP_SCRIPT = StartupScript.ACIDIC_GRAB


DEFAULT_TARGET_KIND = "acidic"
# DEFAULT_TARGET_KIND = "basic"

# =========================
# Match / Starting Zone
# =========================

MATCH_ZONE_SOURCE = MatchZoneSource.AUTO
# MATCH_ZONE_SOURCE = MatchZoneSource.SR       # Competition: use SR API robot.zone
# MATCH_ZONE_SOURCE = MatchZoneSource.USB      # Home testing: read USB stick
# MATCH_ZONE_SOURCE = MatchZoneSource.FIXED    # Debugging: use MATCH_ZONE_FIXED

MATCH_ZONE_FIXED = 0
USB_MATCH_ZONE_FILE = "zone.txt"


START_SLOT = 3