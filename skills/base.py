# skills/base.py

from enum import Enum, auto


class SkillStatus(Enum):
    RUNNING = auto()
    SUCCEEDED = auto()
    FAILED = auto()


class Skill:
    """
    Base class for all skills.

    A Skill:
    - May sequence primitives
    - May hold short-lived internal state
    - Does NOT decide *when* it runs (that is Behavior's job)
    """

    def start(self, **kwargs) -> SkillStatus:
        """
        Called once when the skill begins.
        """
        return SkillStatus.RUNNING

    def update(self, **kwargs) -> SkillStatus:
        """
        Called repeatedly until completion.
        """
        raise NotImplementedError

    def stop(self):
        """
        Called if the skill is interrupted.
        """
        pass
