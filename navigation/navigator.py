# navigation/navigator.py

from enum import Enum, auto


class NavStatus(Enum):
    IDLE = auto()
    RUNNING = auto()
    SUCCEEDED = auto()
    FAILED = auto()
    CANCELLED = auto()


class Navigator:
    """
    High-level navigation interface.

    Behaviors issue goals.
    Navigator decides how to make progress.
    """

    def __init__(self, *, localisation):
        self.localisation = localisation
        self.goal = None
        self.status = NavStatus.IDLE

    def goto(self, pose):
        """
        Set a navigation goal.

        pose = (x, y[, heading])
        """
        self.goal = pose
        self.status = NavStatus.RUNNING

    def cancel(self):
        self.goal = None
        self.status = NavStatus.CANCELLED

    def update(self, *, motion_backend):
        """
        Advance navigation by one step.
        """
        if self.goal is None:
            return self.status

        # Placeholder logic for now
        return self.status
