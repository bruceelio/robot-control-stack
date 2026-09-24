from enum import Enum, auto

class RobotState(Enum):
    SCRIPTED_START = auto()
    INIT_ESCAPE = auto()
    SEEK_AND_COLLECT = auto()
    PICKUP_OBJECT = auto()
    POST_PICKUP_REALIGN = auto()
    RECOVER_LOCALISATION = auto()
    RETURN_TO_BASE = auto()
    DROPOFF_OBJECT = auto()
    POST_DROPOFF_REALIGN = auto()
    ROTATE_AND_DRIVE = auto()
    COMPLETE = auto()
