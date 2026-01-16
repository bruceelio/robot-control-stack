# primitives/__init__.py

"""
Primitive base definitions.

A primitive is an atomic robot capability.
It does ONE thing.
It does NOT decide when to run.
"""

from enum import Enum, auto


class PrimitiveStatus(Enum):
    RUNNING = auto()
    SUCCEEDED = auto()
    FAILED = auto()


class Primitive:
    """
    Base class for all primitives.
    """

    def start(self, **kwargs):
        """
        Called once when the primitive begins.
        """
        pass

    def update(self, **kwargs) -> PrimitiveStatus:
        """
        Called repeatedly until completion.
        """
        raise NotImplementedError

    def stop(self):
        """
        Called if the primitive is interrupted.
        """
        pass
