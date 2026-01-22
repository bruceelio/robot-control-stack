# skills/perception/select_target_utils.py

"""
Target selection utilities (pure logic).

- No motion
- No hardware access
- No control-loop state
- Safe for use in behaviors, planning, and tests
"""

from __future__ import annotations

import time


def _get_last_seen_s(t: dict) -> float | None:
    """
    Try common timestamp keys. You should standardise on ONE of these in perception,
    but this makes selection robust while refactoring.
    """
    for k in ("last_seen_s", "last_seen", "timestamp_s", "timestamp"):
        v = t.get(k)
        if v is None:
            continue
        try:
            return float(v)
        except (TypeError, ValueError):
            return None
    return None


def get_closest_target(perception, kind, *, now=None, max_age_s: float = 0.35):
    """
    Return the closest currently-fresh target of the given kind, or None.
    Expects perception.objects[kind] to be a dict of targets keyed by id.
    """
    if now is None:
        now = time.time()

    if perception is None:
        return None

    memory = getattr(perception, "objects", {}).get(kind, {})
    if not memory:
        return None

    fresh = []
    for t in memory.values():
        ts = _get_last_seen_s(t)
        if ts is None:
            continue
        if (now - ts) <= float(max_age_s):
            fresh.append(t)

    if not fresh:
        return None

    return min(fresh, key=lambda t: t["distance"])
