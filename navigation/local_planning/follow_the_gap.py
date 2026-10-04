# navigation/follow_the_gap.py

"""
Target-biased Follow-the-Gap local planner.

This module is deliberately localisation-independent and sensor-independent.
It consumes a perception-derived robot-frame obstacle field and returns a safe
local gap / preferred bearing.  It does not produce wheel or velocity commands;
DWA or another downstream controller may do that.

Conventions
-----------
- Distances are millimetres.
- Angles are radians.
- Bearing 0 = robot forward.
- Positive bearing = left / counter-clockwise.
- Unknown perception is not free space.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import asin, isfinite, pi
from typing import Iterable, Optional, Sequence, Tuple

from .models import (
    FreeSpaceGap,
    LocalObstacleField,
    PolarRangeSample,
    PreferredDirection,
    RobotCollisionGeometry,
)


@dataclass(frozen=True)
class FollowTheGapConfig:
    """
    Configuration for Follow-the-Gap.

    `lookahead_distance_mm`
        Obstacles inside this robot-relative distance participate in gap
        blocking.  Obstacles farther away remain useful to perception but do
        not close a local gap in this FTG pass.

    `min_gap_width_rad`
        Minimum centre-line angular width accepted as a usable gap after
        footprint inflation.

    `edge_margin_rad`
        Additional angular margin kept between the selected bearing and a gap
        edge.  This is a navigation margin, separate from footprint inflation.

    `min_confidence`
        Valid range samples below this perception confidence are treated as
        unknown.

    `unknown_is_blocked`
        Recommended True.  An unavailable / invalid sector must not silently
        become free space.
    """

    lookahead_distance_mm: float
    min_gap_width_rad: float = 0.0
    edge_margin_rad: float = 0.0
    min_confidence: float = 0.0
    unknown_is_blocked: bool = True


@dataclass(frozen=True)
class _AngularInterval:
    start_rad: float
    end_rad: float

    @property
    def width_rad(self) -> float:
        return self.end_rad - self.start_rad


@dataclass(frozen=True)
class _SampleSupport:
    sample: PolarRangeSample
    start_rad: float
    end_rad: float


_EPS = 1e-9


def find_best_gap(
    obstacle_field: LocalObstacleField,
    preferred_direction: PreferredDirection,
    collision_geometry: RobotCollisionGeometry,
    config: FollowTheGapConfig,
) -> Optional[FreeSpaceGap]:
    """
    Return the best traversable local gap, or None if no safe gap exists.

    Selection is target / intent biased:

    1. Build blocked angular intervals from local obstacle observations.
    2. Inflate blocked intervals by the robot collision radius.
    3. Find the remaining free angular intervals.
    4. Prefer a gap containing the requested bearing.
    5. Otherwise choose the gap requiring the smallest angular deviation from
       the requested bearing; wider gaps break ties.
    6. Select the requested bearing inside that gap where possible, otherwise
       the nearest admissible bearing inside the gap.

    This keeps FTG responsible for *where to go*.  It intentionally does not
    decide *how fast to go*; DWA or another downstream controller owns that.
    """

    _validate_config(config)
    _validate_collision_geometry(collision_geometry)

    supports = _build_sample_supports(obstacle_field.samples)
    if not supports:
        return None

    fov_start = min(item.start_rad for item in supports)
    fov_end = max(item.end_rad for item in supports)

    if not (isfinite(fov_start) and isfinite(fov_end)) or fov_end <= fov_start:
        return None

    blocked = _build_blocked_intervals(
        supports=supports,
        fov_start=fov_start,
        fov_end=fov_end,
        inflated_radius_mm=collision_geometry.inflated_radius_mm,
        config=config,
    )
    blocked = _merge_intervals(blocked)

    free = _complement_intervals(
        blocked=blocked,
        domain_start=fov_start,
        domain_end=fov_end,
    )

    usable = [
        interval
        for interval in free
        if _usable_interval_width(interval, config.edge_margin_rad)
        + _EPS
        >= config.min_gap_width_rad
    ]
    if not usable:
        return None

    preferred = preferred_direction.bearing_rad
    if not isfinite(preferred):
        return None

    chosen = min(
        usable,
        key=lambda interval: (
            _distance_to_interval_interior(
                preferred,
                interval,
                config.edge_margin_rad,
            ),
            -interval.width_rad,
        ),
    )

    selected_bearing = _clamp_to_interval_interior(
        preferred,
        chosen,
        config.edge_margin_rad,
    )

    min_clearance_mm = _minimum_clearance_in_gap(
        supports,
        chosen.start_rad,
        chosen.end_rad,
        min_confidence=config.min_confidence,
    )

    return FreeSpaceGap(
        start_bearing_rad=chosen.start_rad,
        end_bearing_rad=chosen.end_rad,
        selected_bearing_rad=selected_bearing,
        min_clearance_mm=min_clearance_mm,
    )


def _validate_config(config: FollowTheGapConfig) -> None:
    if not isfinite(config.lookahead_distance_mm) or config.lookahead_distance_mm <= 0.0:
        raise ValueError("lookahead_distance_mm must be finite and > 0")
    if not isfinite(config.min_gap_width_rad) or config.min_gap_width_rad < 0.0:
        raise ValueError("min_gap_width_rad must be finite and >= 0")
    if not isfinite(config.edge_margin_rad) or config.edge_margin_rad < 0.0:
        raise ValueError("edge_margin_rad must be finite and >= 0")
    if not isfinite(config.min_confidence) or not 0.0 <= config.min_confidence <= 1.0:
        raise ValueError("min_confidence must be within [0, 1]")


def _validate_collision_geometry(geometry: RobotCollisionGeometry) -> None:
    if not isfinite(geometry.collision_radius_mm) or geometry.collision_radius_mm < 0.0:
        raise ValueError("collision_radius_mm must be finite and >= 0")
    if not isfinite(geometry.safety_margin_mm) or geometry.safety_margin_mm < 0.0:
        raise ValueError("safety_margin_mm must be finite and >= 0")


def _build_sample_supports(
    samples: Sequence[PolarRangeSample],
) -> Tuple[_SampleSupport, ...]:
    """
    Associate every sample with an angular support interval.

    Perception should ideally provide `angular_width_rad`.  When it is zero,
    support is inferred from neighbouring sample bearings.  If sensing has a
    real angular hole, perception should represent that hole explicitly with
    unknown samples rather than omit it; unknown sectors are then blocked by
    default.
    """

    ordered = sorted(
        (sample for sample in samples if isfinite(sample.bearing_rad)),
        key=lambda sample: sample.bearing_rad,
    )
    if not ordered:
        return ()

    if len(ordered) == 1:
        sample = ordered[0]
        if not isfinite(sample.angular_width_rad) or sample.angular_width_rad <= 0.0:
            return ()
        half = sample.angular_width_rad / 2.0
        return (
            _SampleSupport(
                sample=sample,
                start_rad=sample.bearing_rad - half,
                end_rad=sample.bearing_rad + half,
            ),
        )

    supports = []
    for index, sample in enumerate(ordered):
        explicit_width = (
            sample.angular_width_rad
            if isfinite(sample.angular_width_rad) and sample.angular_width_rad > 0.0
            else None
        )

        if explicit_width is not None:
            half = explicit_width / 2.0
            start = sample.bearing_rad - half
            end = sample.bearing_rad + half
        else:
            if index == 0:
                right_mid = (sample.bearing_rad + ordered[index + 1].bearing_rad) / 2.0
                half = right_mid - sample.bearing_rad
                start = sample.bearing_rad - half
                end = right_mid
            elif index == len(ordered) - 1:
                left_mid = (ordered[index - 1].bearing_rad + sample.bearing_rad) / 2.0
                half = sample.bearing_rad - left_mid
                start = left_mid
                end = sample.bearing_rad + half
            else:
                start = (ordered[index - 1].bearing_rad + sample.bearing_rad) / 2.0
                end = (sample.bearing_rad + ordered[index + 1].bearing_rad) / 2.0

        if end > start:
            supports.append(
                _SampleSupport(
                    sample=sample,
                    start_rad=start,
                    end_rad=end,
                )
            )

    return tuple(supports)


def _build_blocked_intervals(
    supports: Sequence[_SampleSupport],
    fov_start: float,
    fov_end: float,
    inflated_radius_mm: float,
    config: FollowTheGapConfig,
) -> Tuple[_AngularInterval, ...]:
    blocked = []

    for item in supports:
        sample = item.sample

        valid = (
            sample.is_valid
            and sample.confidence >= config.min_confidence
        )

        if not valid:
            if config.unknown_is_blocked:
                blocked.append(
                    _AngularInterval(
                        start_rad=max(fov_start, item.start_rad),
                        end_rad=min(fov_end, item.end_rad),
                    )
                )
            continue

        assert sample.range_mm is not None

        if sample.range_mm > config.lookahead_distance_mm:
            continue

        inflation_rad = _footprint_inflation_angle(
            obstacle_range_mm=sample.range_mm,
            inflated_radius_mm=inflated_radius_mm,
        )

        blocked.append(
            _AngularInterval(
                start_rad=max(fov_start, item.start_rad - inflation_rad),
                end_rad=min(fov_end, item.end_rad + inflation_rad),
            )
        )

    return tuple(
        interval
        for interval in blocked
        if interval.end_rad > interval.start_rad
    )


def _footprint_inflation_angle(
    obstacle_range_mm: float,
    inflated_radius_mm: float,
) -> float:
    if inflated_radius_mm <= 0.0:
        return 0.0
    if obstacle_range_mm <= 0.0:
        return pi / 2.0

    ratio = min(1.0, inflated_radius_mm / obstacle_range_mm)
    return asin(ratio)


def _merge_intervals(
    intervals: Iterable[_AngularInterval],
) -> Tuple[_AngularInterval, ...]:
    ordered = sorted(intervals, key=lambda interval: interval.start_rad)
    if not ordered:
        return ()

    merged = [ordered[0]]

    for current in ordered[1:]:
        previous = merged[-1]
        if current.start_rad <= previous.end_rad + _EPS:
            merged[-1] = _AngularInterval(
                start_rad=previous.start_rad,
                end_rad=max(previous.end_rad, current.end_rad),
            )
        else:
            merged.append(current)

    return tuple(merged)


def _complement_intervals(
    blocked: Sequence[_AngularInterval],
    domain_start: float,
    domain_end: float,
) -> Tuple[_AngularInterval, ...]:
    free = []
    cursor = domain_start

    for interval in blocked:
        if interval.start_rad > cursor + _EPS:
            free.append(
                _AngularInterval(
                    start_rad=cursor,
                    end_rad=interval.start_rad,
                )
            )
        cursor = max(cursor, interval.end_rad)

    if cursor < domain_end - _EPS:
        free.append(
            _AngularInterval(
                start_rad=cursor,
                end_rad=domain_end,
            )
        )

    return tuple(free)


def _usable_interval_width(
    interval: _AngularInterval,
    edge_margin_rad: float,
) -> float:
    return max(0.0, interval.width_rad - 2.0 * edge_margin_rad)


def _interior_bounds(
    interval: _AngularInterval,
    edge_margin_rad: float,
) -> Tuple[float, float]:
    low = interval.start_rad + edge_margin_rad
    high = interval.end_rad - edge_margin_rad

    if high < low:
        midpoint = (interval.start_rad + interval.end_rad) / 2.0
        return midpoint, midpoint

    return low, high


def _distance_to_interval_interior(
    bearing_rad: float,
    interval: _AngularInterval,
    edge_margin_rad: float,
) -> float:
    low, high = _interior_bounds(interval, edge_margin_rad)

    if bearing_rad < low:
        return low - bearing_rad
    if bearing_rad > high:
        return bearing_rad - high
    return 0.0


def _clamp_to_interval_interior(
    bearing_rad: float,
    interval: _AngularInterval,
    edge_margin_rad: float,
) -> float:
    low, high = _interior_bounds(interval, edge_margin_rad)
    return min(max(bearing_rad, low), high)


def _minimum_clearance_in_gap(
    supports: Sequence[_SampleSupport],
    start_rad: float,
    end_rad: float,
    min_confidence: float,
) -> Optional[float]:
    ranges = [
        item.sample.range_mm
        for item in supports
        if start_rad - _EPS <= item.sample.bearing_rad <= end_rad + _EPS
        and item.sample.is_valid
        and item.sample.confidence >= min_confidence
        and item.sample.range_mm is not None
    ]

    if not ranges:
        return None

    return min(ranges)
