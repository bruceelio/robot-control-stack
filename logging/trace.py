"""
trace.py — Structured TRACE logging helper

Outputs machine-parseable log lines:

TRACE t=<sec> tick=<int> src=<component> evt=<event> run=<run_id> phase=<phase> k=v ...

Design goals:
- zero dependencies
- cheap to call
- safe in tight loops
- consistent formatting
"""

from __future__ import annotations

import time
import threading
from typing import Any, Dict

# --------------------------------------------------
# Global switches
# --------------------------------------------------

TRACE_ENABLED = True          # master switch
TRACE_FLUSH = False           # set True if you want print(..., flush=True)

# --------------------------------------------------
# Internal state (thread-safe)
# --------------------------------------------------

_lock = threading.Lock()
_tick: int = 0
_run: int = 0

# last-emitted time for throttled keys
_last_emit: Dict[str, float] = {}


# --------------------------------------------------
# Public API
# --------------------------------------------------

def set_enabled(enabled: bool) -> None:
    global TRACE_ENABLED
    TRACE_ENABLED = bool(enabled)


def next_tick() -> int:
    """Increment and return global tick counter."""
    global _tick
    with _lock:
        _tick += 1
        return _tick


def get_tick() -> int:
    return _tick


def next_run() -> int:
    """Increment and return AcquireObject run id."""
    global _run
    with _lock:
        _run += 1
        return _run


def get_run() -> int:
    return _run


def trace(
    *,
    src: str,
    evt: str,
    phase: str,
    run: int | None = None,
    tick: int | None = None,
    **fields: Any,
) -> None:
    """
    Emit one TRACE line.

    Required:
        src   = short component tag (ACQ, APPROACH, SELECT, TRACK, RECOVER, ...)
        evt   = event code (ACQ_START, SELECT_PICK, MOTION_CMD, ...)
        phase = high-level phase (SELECT, ALIGN, APPROACHING, ...)

    Optional:
        run   = run id (defaults to global)
        tick  = tick id (defaults to global)
        **fields = extra k=v pairs
    """
    if not TRACE_ENABLED:
        return

    if run is None:
        run = _run
    if tick is None:
        tick = _tick

    t = time.time()

    parts = [
        "TRACE",
        f"t={t:.3f}",
        f"tick={tick}",
        f"src={_safe(src)}",
        f"evt={_safe(evt)}",
        f"run={run}",
        f"phase={_safe(phase)}",
    ]

    for k, v in fields.items():
        if v is None:
            continue
        parts.append(f"{_safe(k)}={_safe(v)}")

    line = " ".join(parts)
    print(line, flush=TRACE_FLUSH)


def trace_throttled(
    *,
    key: str,
    min_interval_s: float,
    src: str,
    evt: str,
    phase: str,
    run: int | None = None,
    tick: int | None = None,
    **fields: Any,
) -> None:
    """
    Emit a TRACE line at most once per min_interval_s for this key.
    Useful for SELECT_NONE, waiting loops, etc.
    """
    if not TRACE_ENABLED:
        return

    now = time.time()
    last = _last_emit.get(key)

    if last is not None and (now - last) < float(min_interval_s):
        return

    _last_emit[key] = now
    trace(src=src, evt=evt, phase=phase, run=run, tick=tick, **fields)


# --------------------------------------------------
# Helpers
# --------------------------------------------------

def _safe(v: Any) -> str:
    """
    Make value safe for key=value logs:
    - no spaces
    - no newlines
    - compact floats
    """
    if isinstance(v, float):
        return f"{v:.3f}"
    s = str(v)
    s = s.replace(" ", "_").replace("\n", "|")
    return s
