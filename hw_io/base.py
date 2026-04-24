# hw_io/base.py

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, Protocol, Iterable


class DigitalOutputs(Protocol):
    def names(self) -> Iterable[str]: ...
    def set(self, name: str, on: bool) -> None: ...
    def get(self, name: str) -> bool: ...


class Buzzer(Protocol):
    def buzz(self, tone: Any, duration: float, *, blocking: bool = False) -> None: ...
    def off(self) -> None: ...


class IOMap(ABC):
    """
    Canonical robot IO interface.

    Exposes robot sensors and actuators in semantic terms.
    No hardware assumptions, no pin numbers, no SR imports.

    Preferred direct-access convention:
      io.bumper["front_left"]
      io.reflectance["centre"]
      io.ultrasonic["front"]
      io.current["gripper_right"].amps
      io.voltage["battery"].volts
      io.encoder["shooter"].A
      io.encoder["shooter"].B
      io.motor["shooter"].power
      io.servo["lift"].position
      io.camera["front"].see()

    Compatibility methods such as bumpers(), reflectance(), ultrasonics(),
    cameras(), motors, servos, and battery() remain for older code.
    """

    # ---------- Direct IO collections ----------

    @property
    def bumper(self):
        raise NotImplementedError

    @property
    def reflectance(self):
        raise NotImplementedError

    @property
    def ultrasonic(self):
        raise NotImplementedError

    @property
    def current(self):
        raise NotImplementedError

    @property
    def voltage(self):
        raise NotImplementedError

    @property
    def encoder(self):
        raise NotImplementedError

    @property
    def camera(self):
        raise NotImplementedError

    @property
    def motor(self):
        raise NotImplementedError

    @property
    def servo(self):
        raise NotImplementedError

    # ---------- Sensors ----------

    @abstractmethod
    def sense(self) -> Dict[str, Any]:
        """Unified snapshot of all sensors. Keys must be stable across robots."""
        raise NotImplementedError

    @abstractmethod
    def bumpers(self) -> Dict[str, bool]:
        raise NotImplementedError

    @abstractmethod
    def reflectance_values(self) -> Dict[str, float]:
        raise NotImplementedError

    @abstractmethod
    def ultrasonics(self) -> Dict[str, Optional[float]]:
        raise NotImplementedError

    # ---------- Camera ----------

    @abstractmethod
    def cameras(self) -> Dict[str, Any]:
        """Return cameras keyed by semantic name, e.g. {'front': cam}."""
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
        """Optional named digital outputs (solenoids, pumps, relays, etc.)."""
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

    # ---------- Optional capabilities ----------

    def kch(self):
        """Optional KCH/BrainBoard access (LEDs etc)."""
        return None

    def buzzer(self) -> Buzzer | None:
        """Optional piezo/buzzer access."""
        return None

    def wait_start(self):
        """Optional start-gate convenience."""
        return None
