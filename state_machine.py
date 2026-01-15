from enum import Enum, auto

class RobotState(Enum):
    INIT = auto()
    SEARCH = auto()
    POST_PICKUP_REALIGN = auto()
    RETURN_TO_BASE = auto()
    ROTATE_AND_DRIVE = auto()
    COMPLETE = auto()
