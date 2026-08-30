# localisation/providers/vision/vision_arbiter.py

from __future__ import annotations

from typing import Iterable, Optional, Sequence

from vision.apriltag.observations import AprilTagObservation
from vision.apriltag.reconcile import reconcile_apriltag_markers
from localisation.providers.base import PoseObservation, PoseProvider


class VisionArbiter(PoseProvider):
    """
    Vision-level arbiter.

    Owns multiple vision localisation providers and exposes them as one
    higher-level 'vision' provider to the final localisation arbiter.

    Example child providers:
    - Cam1Markers2Provider
    - AprilTagPnPProvider
    """

    def __init__(self, providers: Iterable[PoseProvider]):
        super().__init__("vision", base_weight=0.9)
        self.providers = list(providers)
        self._selected_source: str | None = None

    def set_vision_message(
            self,
            vision_message: dict | None,
    ) -> None:

        # No current vision result: clear vision-provider inputs.
        if vision_message is None:
            self.set_detections([])

            self.set_apriltag_observations(
                source_id=None,
                observations=[],
            )
            return

        camera_name = str(
            vision_message.get("camera", "unknown")
        )

        measurement_timestamp = float(
            vision_message.get("timestamp", 0.0)
        )

        # ----------------------------------
        # Range/bearing arena observations
        # -> cam1_markers2
        # ----------------------------------

        arena_detections = [
            {
                **detection,
                "timestamp": measurement_timestamp,
            }
            for detection
            in vision_message.get("detections", [])
        ]

        self.set_detections(arena_detections)

        # ----------------------------------
        # Neutral AprilTag observations
        # -> PnP
        # ----------------------------------

        source_id, apriltag_observations = (
            reconcile_apriltag_markers(
                camera_name=camera_name,
                timestamp=measurement_timestamp,
                markers=list(
                    vision_message.get("markers", [])
                ),
            )
        )

        self.set_apriltag_observations(
            source_id=source_id,
            observations=apriltag_observations,
        )

    def set_detections(self, arena_detections) -> None:
        for provider in self.providers:
            if hasattr(provider, "set_detections"):
                provider.set_detections(arena_detections)

    def set_apriltag_observations(
            self,
            *,
            source_id: str | None,
            observations: Sequence[AprilTagObservation] | None,
    ) -> None:
        for provider in self.providers:
            if hasattr(
                    provider,
                    "set_apriltag_observations",
            ):
                provider.set_apriltag_observations(
                    source_id=source_id,
                    observations=observations,
                )

    def get_observation(self, now_s: float) -> Optional[PoseObservation]:
        candidates: list[PoseObservation] = []

        for provider in self.providers:
            obs = provider.get_observation(now_s=now_s)
            if obs is None:
                continue
            if not obs.position_valid:
                continue
            candidates.append(obs)

        if not candidates:
            if self._selected_source is not None:
                print(
                    f"[VISION_ARBITER][SOURCE] "
                    f"{self._selected_source} -> None"
                )
                self._selected_source = None

            return None

        print("[VISION_ARBITER] candidates:")
        for obs in candidates:
            print(
                f"  source={obs.source} "
                f"conf={obs.confidence:.3f} "
                f"age={obs.age(now_s):.3f}s "
                f"heading_valid={obs.heading_valid}"
            )

        best = max(
            candidates,
            key=lambda obs: (
                float(obs.confidence),
                1 if obs.heading_valid else 0,
            ),
        )

        if best.source != self._selected_source:
            print(
                f"[VISION_ARBITER][SOURCE] "
                f"{self._selected_source} -> {best.source} "
                f"confidence={best.confidence:.3f}"
            )

            self._selected_source = best.source

        return PoseObservation(
            x=best.x,
            y=best.y,
            heading=best.heading,
            position_valid=best.position_valid,
            heading_valid=best.heading_valid,
            confidence=best.confidence,
            source=best.source,
            timestamp=best.timestamp,
            is_absolute=best.is_absolute,
            diagnostics={
                "vision_provider": best.source,
                "vision_candidates": [
                    {
                        "source": obs.source,
                        "confidence": obs.confidence,
                         "heading_valid": obs.heading_valid,
                    }
                    for obs in candidates
                ],
                "selected": best.source,
                "selected_diagnostics": best.diagnostics,
            },
        )

    def reseed(self, pose) -> None:
        for provider in self.providers:
            if hasattr(provider, "reseed"):
                provider.reseed(pose)

    def invalidate(self) -> None:
        for provider in self.providers:
            if hasattr(provider, "invalidate"):
                provider.invalidate()