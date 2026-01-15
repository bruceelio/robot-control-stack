from enum import Enum, auto

class RobotState(Enum):
    INIT_ESCAPE = auto()
    SEEK_AND_COLLECT = auto()
    POST_PICKUP_REALIGN = auto()
    RECOVER_LOCALISATION = auto()
    RETURN_TO_BASE = auto()
    ROTATE_AND_DRIVE = auto()
    COMPLETE = auto()
