# hw_io/base.py

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, Protocol, Iterable


class DigitalOutputs(Protocol):
    def names(self) -> Iterable[str]: ...
    def set(self, name: str, on: bool) -> None: ...
    def get(self, name: str) -> bool: ...

class IOMap(ABC):
    """
    Canonical robot IO interface.

    Exposes robot sensors and actuators in semantic terms.
    No hardware assumptions, no pin numbers, no SR imports.
    """

    # ---------- Sensors ----------

    @abstractmethod
    def sense(self) -> Dict[str, Any]:
        """
        Unified snapshot of all sensors.
        Keys must be stable across robots.
        """
        raise NotImplementedError

    # ---------- Individual accessors (optional but recommended) ----------

    @abstractmethod
    def bumpers(self) -> Dict[str, bool]:
        raise NotImplementedError

    @abstractmethod
    def reflectance(self) -> Dict[str, float]:
        raise NotImplementedError

    @abstractmethod
    def ultrasonics(self) -> Dict[str, Optional[float]]:
        raise NotImplementedError

    # ---------- Camera ----------

    @abstractmethod
    def cameras(self) -> Dict[str, Any]:
        """
        Return available cameras keyed by semantic name.
        Example: {"front": camera_obj, "rear": camera_obj}
        """
        raise NotImplementedError


    # ---------- Actuators (exposed, not controlled) ----------

    @property
    @abstractmethod
    def motors(self):
        raise NotImplementedError

    @property
    @abstractmethod
    def servos(self):
        raise NotImplementedError

    # ---------- Digital Outputs ----------

    @property
    @abstractmethod
    def outputs(self) -> DigitalOutputs | None:
        """Optional named digital outputs (solenoids, pumps, relays, etc.)"""
        raise NotImplementedError

    # ---------- Power ----------

    @abstractmethod
    def battery(self) -> Dict[str, Optional[float]]:
        raise NotImplementedError

    # ---------- Sleep ----------

    @abstractmethod
    def sleep(self, secs: float) -> None:
        """Sleep using the platform’s preferred timing (robot time if available)."""
        raise NotImplementedError