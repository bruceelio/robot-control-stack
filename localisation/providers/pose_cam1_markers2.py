# localisation/providers/pose_cam1_markers2.py

from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence, Tuple

from config import CONFIG
from config.arena import marker_locations
from navigation.pose_trilaterate import trilaterate_point

from localisation.pose_types import PoseObservation
from localisation.providers.base import PoseProvider


class Cam1Markers2Provider(PoseProvider):
    """
    Single-camera pose estimator using >=2 arena markers.

    Faithful port of the original estimate_pose logic:
      - Uses ALL pairs of visible arena markers
      - For each pair, gets both circle intersection candidates (C1, C2)
      - Keeps candidates inside arena bounds
      - Averages all kept candidates -> (x, y)
      - heading is unknown; we return heading=None (truthful)
        (If you need legacy behaviour temporarily, set legacy_heading_zero=True)

    Input:
      arena_detections: list[dict] with keys:
        {"id": int, "distance_mm": float, "bearing_deg": float, "camera": str}

    Note:
      bearing_deg is intentionally unused here (matches old estimator behaviour).
    """

    name = "cam1_markers2"

    def __init__(
        self,
        *,
        arena_size_mm: Optional[float] = None,
        border_margin_mm: float = 0.0,
        legacy_heading_zero: bool = False,
    ):
        self.arena_size_mm = float(arena_size_mm) if arena_size_mm is not None else float(CONFIG.arena_size)
        self.border_margin_mm = float(border_margin_mm)
        self.legacy_heading_zero = bool(legacy_heading_zero)

        # World coordinates of arena markers: id -> (x, y)
        self._arena_markers = marker_locations(self.arena_size_mm)

    def estimate(self, *, arena_detections: Sequence[dict], now_s: float) -> PoseObservation | None:
        obs = self._normalise(arena_detections)
        if len(obs) < 2:
            return None

        positions: List[Tuple[float, float]] = []
        pairs_used: List[Tuple[int, int]] = []

        # Try all pairs (i < j)
        for i in range(len(obs)):
            for j in range(i + 1, len(obs)):
                m1 = obs[i]
                m2 = obs[j]

                mid1 = m1["id"]
                mid2 = m2["id"]

                if mid1 not in self._arena_markers or mid2 not in self._arena_markers:
                    continue

                A = self._arena_markers[mid1]
                B = self._arena_markers[mid2]
                AC = m1["distance_mm"]
                BC = m2["distance_mm"]

                try:
                    C1, C2 = trilaterate_point(A, B, AC, BC)
                except ValueError:
                    continue

                kept_any = False
                for C in (C1, C2):
                    if self._inside_arena(C[0], C[1]):
                        positions.append((float(C[0]), float(C[1])))
                        kept_any = True

                if kept_any:
                    pairs_used.append((mid1, mid2))

        if not positions:
            return None

        x = sum(p[0] for p in positions) / len(positions)
        y = sum(p[1] for p in positions) / len(positions)

        heading = 0.0 if self.legacy_heading_zero else None

        # Confidence heuristic: more valid candidates => higher confidence (capped)
        confidence = min(0.95, 0.50 + 0.05 * len(positions))

        meta: Dict[str, Any] = {
            "camera": obs[0].get("camera", "unknown"),
            "markers_seen": [m["id"] for m in obs],
            "pairs_used": pairs_used,
            "candidate_count": len(positions),
            "arena_size_mm": self.arena_size_mm,
        }

        return PoseObservation(
            x=float(x),
            y=float(y),
            heading=heading,
            confidence=float(confidence),
            source=self.name,
            timestamp=float(now_s),
            meta=meta,
        )

    # -----------------------------
    # Helpers
    # -----------------------------

    def _normalise(self, arena_detections: Sequence[dict]) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        for d in arena_detections:
            try:
                mid = int(d["id"])
                dist = float(d["distance_mm"])
            except Exception:
                continue

            if dist <= 0.0:
                continue

            out.append(
                {
                    "id": mid,
                    "distance_mm": dist,
                    "bearing_deg": float(d.get("bearing_deg", 0.0)),  # unused for now
                    "camera": d.get("camera", "unknown"),
                }
            )
        return out

    def _inside_arena(self, x: float, y: float) -> bool:
        half = (self.arena_size_mm / 2.0) - self.border_margin_mm
        return (-half <= x <= half) and (-half <= y <= half)
