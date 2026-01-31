# navigation/arc_approach_control.py

from __future__ import annotations
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ArcApproachParams:
    trigger_deg: float = 10.0          # if |bearing| >= this -> arc mode
    gain: float = 0.5                  # proportional turn: turn = gain * bearing
    max_turn_deg: float = 25.0         # clamp for turn step
    step_mm: float = 250.0             # forward step each cycle in arc mode
    min_step_mm: float = 5.0           # clamp min drive
    max_step_mm: float = 2500.0        # clamp max drive


@dataclass(frozen=True)
class ArcCommand:
    use_arc: bool
    turn_deg: float
    drive_mm: float
    reason: str = ""


def arc_command_for_band_b(
    *,
    distance_mm: float,
    bearing_deg: float,
    commit_distance_mm: float,
    params: ArcApproachParams,
) -> ArcCommand:
    """
    Produce a rotate+drive command which yields an arc-like approach using
    your existing primitives (AlignToTarget -> Drive).

    Intended for Band B: distance > commit_distance_mm.
    """
    dist = float(distance_mm)
    bearing = float(bearing_deg)
    commit = float(commit_distance_mm)

    remaining = dist - commit
    if remaining <= 0:
        return ArcCommand(False, 0.0, 0.0, "at_or_inside_commit")

    if abs(bearing) < params.trigger_deg:
        # Bearing already small; let normal "direct to commit" happen
        return ArcCommand(False, bearing, remaining, "bearing_small_direct")

    # Arc mode: small proportional turn + fixed forward step (clamped to remaining)
    turn = params.gain * bearing
    turn = max(-params.max_turn_deg, min(params.max_turn_deg, turn))

    step = min(params.step_mm, remaining)
    step = max(params.min_step_mm, min(params.max_step_mm, step))

    return ArcCommand(True, turn, step, "arc")
