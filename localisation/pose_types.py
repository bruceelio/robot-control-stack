# localisation/pose_types.py

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any

@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    heading: Optional[float] = None
    position_valid: bool = True
    heading_valid: bool = False
    source: str = "unknown"
    timestamp: float = 0.0

@dataclass(frozen=True)
class PoseObservation:
    x: float
    y: float
    heading: Optional[float] = None
    confidence: float = 0.5
    source: str = "unknown"
    timestamp: float = 0.0
    meta: Dict[str, Any] = None  # e.g. markers_used, camera_id
