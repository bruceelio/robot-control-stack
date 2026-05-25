# localisation/providers/vision/vision_arbiter.py

from __future__ import annotations

from typing import Iterable, Optional

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

    def set_detections(self, arena_detections) -> None:
        for provider in self.providers:
            if hasattr(provider, "set_detections"):
                provider.set_detections(arena_detections)

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
            return None

        print("[VISION_ARBITER] candidates:")
        for obs in candidates:
            print(
                f"  source={obs.source} "
                f"conf={obs.confidence:.3f} "
                f"quality={obs.quality} "
                f"heading_valid={obs.heading_valid}"
            )

        best = max(
            candidates,
            key=lambda obs: (
                float(obs.confidence),
                1 if obs.heading_valid else 0,
            ),
        )

        return PoseObservation(
            x=best.x,
            y=best.y,
            heading=best.heading,
            position_valid=best.position_valid,
            heading_valid=best.heading_valid,
            confidence=best.confidence,
            source=f"vision:{best.source}",
            timestamp=best.timestamp,
            is_absolute=best.is_absolute,
            quality=best.quality,
            diagnostics={
                "vision_provider": best.source,
                "vision_candidates": [
                    {
                        "source": obs.source,
                        "confidence": obs.confidence,
                        "quality": obs.quality,
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