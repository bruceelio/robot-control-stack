# localisation/providers/base.py

from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Sequence
from localisation.pose_types import PoseObservation


class PoseProvider(ABC):
    name: str = "provider"

    @abstractmethod
    def estimate(self, *, arena_detections: Sequence[dict], now_s: float) -> PoseObservation | None:
        """
        Return a PoseObservation if possible, else None.

        arena_detections: list of dicts like:
          {"id": int, "distance_mm": float, "bearing_deg": float, "camera": str, ...}
        """
        raise NotImplementedError
