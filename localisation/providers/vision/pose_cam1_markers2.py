# localisation/providers/vision/pose_cam1_markers2.py

from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence, Tuple

from config import CONFIG
from config.arena import marker_locations
from localisation.pose_types import PoseObservation
from localisation.providers.base import PoseProvider
from navigation.pose_trilaterate import trilaterate_point


class Cam1Markers2Provider(PoseProvider):
    """
    Single-camera pose estimator using >=2 arena markers.

    Faithful port of the original estimate_pose logic:
    - Uses ALL pairs of visible arena markers
    - For each pair, gets both circle intersection candidates (C1, C2)
    - Keeps candidates inside arena bounds
    - Averages all kept candidates -> (x, y)
    - heading is unknown; we return heading=None
      (If you need legacy behaviour temporarily, set legacy_heading_zero=True)

    Input:
        arena_detections: list[dict] with keys like:
            {
                "id": int,
                "distance_mm": float,
                "bearing_deg": float,
                "camera": str,
            }

    Notes:
    - bearing_deg is intentionally unused here. This matches the old estimator.
    - io and current_pose are accepted to satisfy the common PoseProvider interface,
      but are not used by this provider.
    """

    name = "cam1_markers2"

    def __init__(
        self,
        *,
        arena_size_mm: Optional[float] = None,
        border_margin_mm: float = 0.0,
        legacy_heading_zero: bool = False,
    ):
        self.arena_size_mm = (
            float(arena_size_mm)
            if arena_size_mm is not None
            else float(CONFIG.arena_size)
        )
        self.border_margin_mm = float(border_margin_mm)
        self.legacy_heading_zero = bool(legacy_heading_zero)

        # World coordinates of arena markers: id -> (x, y)
        self._arena_markers = marker_locations(self.arena_size_mm)

    def estimate(
        self,
        *,
        io,
        now_s: float,
        current_pose,
        arena_detections: Sequence[dict] | None = None,
    ) -> PoseObservation | None:
        """
        Estimate pose from arena marker detections.

        Returns:
            PoseObservation if a usable position estimate can be formed,
            otherwise None.
        """
        del io
        del current_pose

        if not arena_detections:
            return None

        detections = self._normalise(arena_detections)
        if len(detections) < 2:
            return None

        positions: List[Tuple[float, float]] = []
        pairs_used: List[Tuple[int, int]] = []

        # Try all pairs (i < j)
        for i in range(len(detections)):
            for j in range(i + 1, len(detections)):
                d1 = detections[i]
                d2 = detections[j]

                marker_id_1 = d1["id"]
                marker_id_2 = d2["id"]

                if (
                    marker_id_1 not in self._arena_markers
                    or marker_id_2 not in self._arena_markers
                ):
                    continue

                marker_pos_1 = self._arena_markers[marker_id_1]
                marker_pos_2 = self._arena_markers[marker_id_2]

                dist_1 = d1["distance_mm"]
                dist_2 = d2["distance_mm"]

                try:
                    candidate_1, candidate_2 = trilaterate_point(
                        marker_pos_1,
                        marker_pos_2,
                        dist_1,
                        dist_2,
                    )
                except ValueError:
                    continue

                kept_any = False
                for candidate in (candidate_1, candidate_2):
                    if self._inside_arena(candidate[0], candidate[1]):
                        positions.append((float(candidate[0]), float(candidate[1])))
                        kept_any = True

                if kept_any:
                    pairs_used.append((marker_id_1, marker_id_2))

        if not positions:
            return None

        x = sum(p[0] for p in positions) / len(positions)
        y = sum(p[1] for p in positions) / len(positions)

        heading = 0.0 if self.legacy_heading_zero else None

        # Confidence heuristic: more valid candidates => higher confidence (capped)
        confidence = min(0.95, 0.50 + 0.05 * len(positions))

        meta: Dict[str, Any] = {
            "camera": detections[0].get("camera", "unknown"),
            "markers_seen": [d["id"] for d in detections],
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

    def _normalise(self, arena_detections: Sequence[dict]) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []

        for detection in arena_detections:
            try:
                marker_id = int(detection["id"])
                distance_mm = float(detection["distance_mm"])
            except Exception:
                continue

            if distance_mm <= 0.0:
                continue

            out.append(
                {
                    "id": marker_id,
                    "distance_mm": distance_mm,
                    "bearing_deg": float(detection.get("bearing_deg", 0.0)),
                    "camera": detection.get("camera", "unknown"),
                }
            )

        return out

    def _inside_arena(self, x: float, y: float) -> bool:
        half = (self.arena_size_mm / 2.0) - self.border_margin_mm
        return (-half <= x <= half) and (-half <= y <= half)