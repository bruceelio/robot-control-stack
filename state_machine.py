from enum import Enum, auto

class RobotState(Enum):
    INIT = auto()
    SEARCH = auto()
    ROTATE_AND_DRIVE = auto()
    COMPLETE = auto()
