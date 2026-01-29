# skills/perception/track_object.py

"""perception.track_object

Minimal stateful tracker for a *locked* target id.

Purpose:
  - Provide a single, consistent "source of truth" for whether the locked target
    is currently visible, and how long since it was last seen.
  - Preserve the last observation for downstream recovery behaviors.

The tracker is intentionally dumb today:
  - Matching is by exact id lookup in perception.objects[kind]
  - If id is missing, it cannot track (locked_id=None => visible_now=False)

This is designed to be extended later with match_quality / heuristics (e.g. by kind + bearing).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


def _as_int(v) -> int | None:
    if v is None:
        return None
    try:
        return int(v)
    except (TypeError, ValueError):
        return None


def _get_last_seen_s(obs: dict) -> float | None:
    # Keep compatible with existing code during refactors.
    for k in ("last_seen_s", "last_seen", "timestamp_s", "timestamp"):
        v = obs.get(k)
        if v is None:
            continue
        try:
            return float(v)
        except (TypeError, ValueError):
            return None
    return None


@dataclass(slots=True)
class TrackSnapshot:
    locked_id: int | None
    visible_now: bool
    age_s: float
    last_seen_time_s: float | None
    last_seen_bearing_deg: float | None
    last_seen_distance_mm: float | None
    current_obs: dict | None
    last_obs: dict | None

    # Optional debug
    seen_count: int = 0
    lost_count: int = 0
    match_quality: float | None = None


class TrackObject:
    """Tracks a single locked target id within perception.objects."""

    def __init__(self, *, kind: str):
        self.kind = kind
        self.locked_id: int | None = None

        self._last_obs: dict | None = None
        self._last_seen_time_s: float | None = None
        self._seen_count = 0
        self._lost_count = 0

    def reset(self, *, locked_target_id: int | None, kind: str | None = None) -> None:
        if kind is not None:
            self.kind = kind
        self.locked_id = _as_int(locked_target_id)

        self._last_obs = None
        self._last_seen_time_s = None
        self._seen_count = 0
        self._lost_count = 0

    def update(self, *, perception_objects: Any, now_s: float, locked_target_id: int | None = None, kind: str | None = None) -> TrackSnapshot:
        """
        Args:
          perception_objects: either the full perception object (with `.objects`) or the objects mapping itself.
          now_s: timestamp from the same clock domain as perception last_seen_s (usually time.time()).
          locked_target_id: optional override; if provided will update internal locked_id.
          kind: optional override kind.

        Returns:
          TrackSnapshot
        """

        if kind is not None:
            self.kind = kind
        if locked_target_id is not None or self.locked_id is None:
            self.locked_id = _as_int(locked_target_id)

        # Resolve objects mapping
        objects = getattr(perception_objects, "objects", perception_objects)
        memory = {}
        try:
            memory = (objects or {}).get(self.kind, {}) or {}
        except Exception:
            memory = {}

        current = None
        if self.locked_id is not None:
            # dict is keyed by id; be robust to str keys
            current = memory.get(self.locked_id)
            if current is None:
                current = memory.get(str(self.locked_id))

        visible_now = current is not None

        if visible_now:
            self._seen_count += 1
            self._lost_count = 0
            self._last_obs = current

            # Prefer observation's own last_seen_s, but fall back to now_s for safety.
            ts = _get_last_seen_s(current)
            self._last_seen_time_s = ts if ts is not None else float(now_s)
        else:
            self._lost_count += 1

        # Snapshot fields
        last_seen = self._last_seen_time_s
        if last_seen is None:
            age_s = math.inf
        else:
            age_s = max(0.0, float(now_s) - float(last_seen))

        last_bearing = None
        last_dist = None
        if self._last_obs is not None:
            try:
                last_bearing = float(self._last_obs.get("bearing"))
            except Exception:
                last_bearing = None
            try:
                # distance is mm in the existing codebase
                last_dist = float(self._last_obs.get("distance"))
            except Exception:
                last_dist = None

        return TrackSnapshot(
            locked_id=self.locked_id,
            visible_now=visible_now,
            age_s=age_s,
            last_seen_time_s=last_seen,
            last_seen_bearing_deg=last_bearing,
            last_seen_distance_mm=last_dist,
            current_obs=current if visible_now else None,
            last_obs=self._last_obs,
            seen_count=self._seen_count,
            lost_count=self._lost_count,
            match_quality=1.0 if visible_now else None,
        )
