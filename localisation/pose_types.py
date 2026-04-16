# localisation/pose_types.py

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    heading: Optional[float] = None

    position_valid: bool = True
    heading_valid: bool = False

    source: str = "unknown"
    timestamp: float = 0.0