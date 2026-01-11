# state_machine.py

from enum import Enum, auto


class RobotState(Enum):
    INIT = auto()
    SEARCH = auto()
    COMPLETE = auto()
