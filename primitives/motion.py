# primitives/motion.py

"""
Motion primitives.

Atomic movement commands.
No planning.
No navigation.
"""

from primitives.base import Primitive


class Drive(Primitive):
    """
    Drive a specified distance.
    """
    pass


class Rotate(Primitive):
    """
    Rotate in place by a specified angle.
    """
    pass


class Stop(Primitive):
    """
    Immediately stop all motion.
    """
    pass
