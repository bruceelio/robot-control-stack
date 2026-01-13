# hal/hardware.py
"""
Hardware environment detection.

This module answers:
- Are we running with the SR Robot3 API available?
- Are we likely in the SR simulator?
- Or are we running on non-SR hardware?

This file must have:
- No side effects
- No hardware access
- No imports from higher layers
"""

from enum import Enum, auto


class HardwareMode(Enum):
    SR = auto()        # SR Robot3 API available (real or sim)
    NON_SR = auto()    # No SR API, direct hardware control


# Detect SR Robot3 availability
try:
    import sr.robot3  # noqa: F401
    HAS_SR_API = True
except ImportError:
    HAS_SR_API = False


# Determine hardware mode
if HAS_SR_API:
    HARDWARE_MODE = HardwareMode.SR
else:
    HARDWARE_MODE = HardwareMode.NON_SR


def is_sr():
    """Return True if running with SR Robot3 API available."""
    return HARDWARE_MODE is HardwareMode.SR


def is_non_sr():
    """Return True if running without SR Robot3 API."""
    return HARDWARE_MODE is HardwareMode.NON_SR
