# primitives/sensing.py

"""
Sensing primitives.

Atomic sensing actions.
No interpretation.
"""

from primitives.base import Primitive


class DetectMarkers(Primitive):
    """
    Detect visible markers.
    """
    pass


class EstimateMarkerPose(Primitive):
    """
    Estimate bearing and distance to a marker.
    """
    pass


class IdentifyMarker(Primitive):
    """
    Identify marker type (acidic/basic).
    """
    pass
