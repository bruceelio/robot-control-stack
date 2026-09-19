# localisation/providers/odometry/odometry_arbiter.py

from __future__ import annotations

from dataclasses import replace
from typing import Iterable

from localisation.providers.base import PoseObservation, PoseProvider


class OdometryArbiter(PoseProvider):
    """
    Family arbiter for robot odometry sources.

    Typical sources include:
    - drivetrain encoders
    - three-wheel deadwheel odometry
    - two-wheel deadwheel + IMU odometry

    Most robots will normally configure only one odometry source.

    If multiple valid sources are present, the currently selected source
    remains selected while it continues to provide a usable observation.
    """

    def __init__(
        self,
        providers: Iterable[PoseProvider],
    ):
        super().__init__("odometry")

        self.providers = list(providers)
        self._selected_source: str | None = None

    # --------------------------------------------------
    # External inputs
    # --------------------------------------------------

    def set_io(self, io) -> None:
        """
        Forward semantic IO to odometry providers which require it.
        """
        for provider in self.providers:
            if hasattr(provider, "set_io"):
                provider.set_io(io)

    # --------------------------------------------------
    # Observation
    # --------------------------------------------------

    def get_observation(
        self,
        now_s: float,
    ) -> PoseObservation | None:

        candidates: list[PoseObservation] = []

        for provider in self.providers:
            obs = provider.get_observation(now_s)

            if obs is None:
                continue

            if not obs.is_usable():
                continue

            # Odometry is a propagated / relative family.
            if obs.is_absolute:
                print(
                    "[ODOMETRY_ARBITER][WARN] "
                    f"ignoring absolute observation from {obs.source}"
                )
                continue

            if not (0.0 <= obs.confidence <= 1.0):
                continue

            if obs.timestamp > now_s:
                continue

            candidates.append(obs)

        # --------------------------------------------------
        # No usable odometry
        # --------------------------------------------------

        if not candidates:
            if self._selected_source is not None:
                print(
                    "[ODOMETRY_ARBITER][SOURCE] "
                    f"{self._selected_source} -> None"
                )

            self._selected_source = None
            return None

        # --------------------------------------------------
        # Sticky source selection
        # --------------------------------------------------

        selected = None

        if self._selected_source is not None:
            for candidate in candidates:
                if candidate.source == self._selected_source:
                    selected = candidate
                    break

        # No currently selected source is available.
        if selected is None:
            selected = max(
                candidates,
                key=lambda obs: (
                    float(obs.confidence),
                    -obs.age(now_s),
                ),
            )

        provider_source = selected.source

        # --------------------------------------------------
        # Source-change diagnostics
        # --------------------------------------------------

        if provider_source != self._selected_source:
            print(
                "[ODOMETRY_ARBITER][SOURCE] "
                f"{self._selected_source} -> {provider_source} "
                f"confidence={selected.confidence:.3f}"
            )

            self._selected_source = provider_source

        # --------------------------------------------------
        # Family-level observation
        # --------------------------------------------------

        diagnostics = dict(selected.diagnostics)

        diagnostics["family"] = "odometry"
        diagnostics["provider_source"] = provider_source

        return replace(
            selected,
            source=self.name,
            diagnostics=diagnostics,
        )

    # --------------------------------------------------
    # Lifecycle
    # --------------------------------------------------

    def reseed(self, pose) -> None:
        for provider in self.providers:
            provider.reseed(pose)

    def invalidate(self) -> None:
        self._selected_source = None

        for provider in self.providers:
            provider.invalidate()