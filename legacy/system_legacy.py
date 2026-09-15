# primitives/system.py

"""
System-level primitives.

Always allowed to interrupt other actions.
"""

from primitives.base import Primitive


class EmergencyStop(Primitive):
    """
    Immediately halt all robot activity.
    """
    pass
