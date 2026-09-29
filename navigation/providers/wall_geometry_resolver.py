# skills/navigation/wall_geometry_resolver.py

"""
Wall-geometry acquisition boundary.

This module converts already-normalized wall information into the
shared navigation.wall_geometry.WallGeometry representation.

It deliberately knows nothing about:

    - IO backends
    - sensor names
    - robot profiles
    - wall-following control laws
    - PBVS / pickup behaviour

The caller decides which measurements belong to the wall of interest.

Supported inputs:

    1. Direct semantic wall information:
           heading_rad
           distance_mm

       Either value may be supplied independently.

    2. Two RangeObservation2D measurements known by the caller to
       intersect the same physical wall plane.

       These are passed to the pure navigation provider
       estimate_wall_geometry_two_ranges(), which returns both wall
       heading and perpendicular wall distance.

Two-range geometry takes precedence when a complete pair is supplied,
because heading and distance should then come from the same geometric
construction.

A single raw range observation is intentionally not interpreted here.
One ray alone does not define a wall plane. If a caller already knows
that a measurement represents semantic perpendicular wall distance, it
should pass that value as distance_mm instead.
"""

from __future__ import annotations

import math

from navigation.wall_geometry import (
    RangeObservation2D,
    WallGeometry,
)
from navigation.providers.wall_geometry_two_ranges import (
    estimate_wall_geometry_two_ranges,
)


class WallGeometryResolver:
    """
    Build WallGeometry from normalized wall information.

    The source is intentionally stateless for now. A class is retained
    as the public boundary so that confidence, continuity, freshness,
    or source diagnostics can be added later without changing consumers.
    """

    def reset(self) -> None:
        """
        Stateless at present; retained for a stable source-style API.
        """
        return None

    def update(
        self,
        *,
        heading_rad: float | None = None,
        distance_mm: float | None = None,
        range_a: RangeObservation2D | None = None,
        range_b: RangeObservation2D | None = None,
    ) -> WallGeometry:
        """
        Return the strongest wall geometry available.

        Precedence:

            range_a + range_b
                -> estimate heading + distance from the two rays

            otherwise
                -> return any directly supplied semantic heading/distance

            nothing usable
                -> empty WallGeometry()

        range_a and range_b must either both be supplied or both omitted.
        The caller is responsible for ensuring that the pair observes the
        same physical wall plane.
        """

        if (range_a is None) != (range_b is None):
            raise ValueError(
                "range_a and range_b must be supplied together"
            )

        if range_a is not None and range_b is not None:
            return estimate_wall_geometry_two_ranges(
                range_a,
                range_b,
            )

        return WallGeometry(
            heading_rad=self._normalise_heading(
                heading_rad
            ),
            distance_mm=self._normalise_distance(
                distance_mm
            ),
        )

    @staticmethod
    def _normalise_heading(
        value: float | None,
    ) -> float | None:
        if value is None:
            return None

        value = float(value)

        if not math.isfinite(value):
            return None

        return math.atan2(
            math.sin(value),
            math.cos(value),
        )

    @staticmethod
    def _normalise_distance(
        value: float | None,
    ) -> float | None:
        if value is None:
            return None

        value = float(value)

        if (
            not math.isfinite(value)
            or value < 0.0
        ):
            return None

        return value
