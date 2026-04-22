# localisation/providers/vision/pose_cam1_markers2.py

from __future__ import annotations

import math

from typing import Any, Dict, List, Optional, Sequence, Tuple

from config import CONFIG
from config.arena import marker_locations
from localisation.providers.base import PoseObservation, PoseProvider
from navigation.pose_trilaterate import trilaterate_point


class Cam1Markers2Provider(PoseProvider):
    """
    Single-camera pose estimator using >=2 arena markers.

    Faithful port of the original estimate_pose logic:
    - Uses ALL pairs of visible arena markers
    - For each pair, gets both circle intersection candidates
    - Keeps candidates inside arena bounds
    - Averages all kept candidates -> (x, y)
    - heading is unknown unless legacy_heading_zero=True

    This provider currently expects detections to be supplied via
    `set_detections(...)` before `get_observation(...)` is called.
    """

    def __init__(
        self,
        *,
        arena_size_mm: Optional[float] = None,
        border_margin_mm: float = 0.0,
        legacy_heading_zero: bool = False,
    ):
        super().__init__("cam1_markers2", base_weight=0.8)

        self.arena_size_mm = (
            float(arena_size_mm)
            if arena_size_mm is not None
            else float(CONFIG.arena_size)
        )
        self.border_margin_mm = float(border_margin_mm)
        self.legacy_heading_zero = bool(legacy_heading_zero)

        # World coordinates of arena markers: id -> (x, y)
        self._arena_markers = marker_locations(self.arena_size_mm)

        # Latest externally supplied detections
        self._arena_detections: List[Dict[str, Any]] = []

    def set_detections(self, arena_detections: Sequence[dict] | None) -> None:
        """
        Supply the latest arena detections for later use by get_observation().
        """
        self._arena_detections = list(arena_detections or [])

    def get_observation(self, now_s: float) -> PoseObservation | None:
        """
        Estimate pose from the latest supplied arena marker detections.

        Returns:
            PoseObservation if a usable position estimate can be formed,
            otherwise None.
        """
        if not self._arena_detections:
            return None

        detections = self._normalise(self._arena_detections)
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

        cam_mount = CONFIG.camera_mounts["front"]

        # Approximate correction (assumes small heading error)
        base_x = float(x) - float(cam_mount["x_mm"])
        base_y = float(y) - float(cam_mount["y_mm"])

        heading = None

        heading_estimates = []

        for detection in detections:
            marker_id = detection["id"]
            if marker_id not in self._arena_markers:
                continue

            marker_x, marker_y = self._arena_markers[marker_id]

            # world angle from robot camera to marker
            cam_x = float(x)
            cam_y = float(y)

            world_angle = math.atan2(marker_y - cam_y, marker_x - cam_x)

            # observed bearing is degrees in robot/camera frame
            bearing_rad = math.radians(float(detection.get("bearing_deg", 0.0)))

            # robot heading estimate
            est = world_angle + bearing_rad

            # wrap to [-pi, pi]
            est = math.atan2(math.sin(est), math.cos(est))
            heading_estimates.append(est)

        if heading_estimates:
            sin_sum = sum(math.sin(a) for a in heading_estimates)
            cos_sum = sum(math.cos(a) for a in heading_estimates)
            heading = math.atan2(sin_sum, cos_sum)
        elif self.legacy_heading_zero:
            heading = 0.0

        candidate_count = len(positions)

        if candidate_count >= 6:
            quality = "good"
            confidence = min(1.0, 0.9 + min(0.08, 0.01 * candidate_count))
        elif candidate_count >= 2:
            quality = "poor"
            confidence = 0.7
        elif candidate_count == 1:
            # Common case for exactly 2 visible arena tags:
            # one pair -> two mathematical intersections -> one inside arena.
            # Treat as usable and stronger than timed motion.
            quality = "poor"
            confidence = 0.65
        else:
            quality = "bad"
            confidence = 0.0

        diagnostics: Dict[str, Any] = {
            "camera": detections[0].get("camera", "unknown"),
            "markers_seen": [d["id"] for d in detections],
            "pairs_used": pairs_used,
            "candidate_count": candidate_count,
            "arena_size_mm": self.arena_size_mm,
            "quality_reason": "multi_marker_triangulation",
        }

        diagnostics["base_link_correction"] = {
            "camera_key": "front",
            "x_mm": float(cam_mount["x_mm"]),
            "y_mm": float(cam_mount["y_mm"]),
        }

        diagnostics["heading_estimate_count"] = len(heading_estimates)
        diagnostics["heading_estimates_deg"] = [
            math.degrees(a) for a in heading_estimates
        ]

        return PoseObservation(
            x=base_x,
            y=base_y,
            heading=heading,
            position_valid=True,
            heading_valid=heading is not None,
            confidence=float(confidence),
            source=self.name,
            timestamp=float(now_s),
            is_absolute=True,
            quality=quality,
            diagnostics=diagnostics,
        )

    def reseed(self, pose) -> None:
        """
        Vision is absolute, so it usually does not need reseeding.
        """
        return None

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