# behaviors/base.py

from enum import Enum, auto


class BehaviorStatus(Enum):
    RUNNING = auto()
    SUCCEEDED = auto()
    FAILED = auto()


class Behavior:
    """
    Base class for all behaviors.
    """

    def __init__(self):
        self.status = BehaviorStatus.RUNNING
        self.active_primitive = None

    def start(self, **kwargs):
        """
        Called once when the behavior begins.
        """
        pass

    def update(self, **kwargs) -> BehaviorStatus:
        """
        Called repeatedly until completion.
        """
        raise NotImplementedError

    def stop(self, **kwargs) -> BehaviorStatus:
        """
        Called if the behavior is interrupted.
        """
        if self.active_primitive:
            self.active_primitive.stop()
