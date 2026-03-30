# hw_io/bob_bot.py

from __future__ import annotations

import time
from typing import Dict, Any, Optional

from config import CONFIG
from hw_io.base import IOMap
from hw_io.cameras.base import Camera
from hw_io.cameras.resolve import resolve_camera


class NullOutputs:
    def __init__(self):
        self._state = {"VACUUM": False}

    def names(self):
        return self._state.keys()

    def set(self, name: str, on: bool) -> None:
        self._state[name] = bool(on)
        print(f"[NullOutputs] {name} -> {on} (not implemented yet)")

    def get(self, name: str) -> bool:
        return self._state.get(name, False)


class BobBotIO(IOMap):
    """
    Minimal BobBot IO mapping.

    Current purpose:
    - provide camera bring-up through CONFIG.cameras
    - allow hw_io.resolve() to construct a valid IOMap for bob_bot

    Future purpose:
    - Arduino Mega digital/analog IO
    - motor control
    - vacuum / outputs
    - bumpers / reflectance / ultrasonics
    """

    def __init__(self, robot):
        # For bob_bot, 'robot' is expected to be None for now.
        # Keep it anyway for interface compatibility.
        self.robot = robot

        self._cameras: Dict[str, Camera] = {}
        self._outputs = NullOutputs()

        self._detect_cameras()

    def _detect_cameras(self):
        """
        Build configured cameras from CONFIG.cameras.
        """
        for key, camera_name in CONFIG.cameras.items():
            self._cameras[key] = resolve_camera(
                camera_name=camera_name,
                robot=self.robot,
            )

    @property
    def outputs(self):
        return self._outputs

    def cameras(self) -> Dict[str, Camera]:
        return dict(self._cameras)

    def bumpers(self) -> Dict[str, bool]:
        return dict(fl=False, fr=False, rl=False, rr=False)

    def reflectance(self) -> Dict[str, float]:
        return dict(left=0.0, center=0.0, right=0.0)

    def ultrasonics(self) -> Dict[str, Optional[float]]:
        return dict(front=None, left=None, right=None, back=None)

    def sense(self) -> Dict[str, Any]:
        return {
            "bumpers": self.bumpers(),
            "reflectance": self.reflectance(),
            "ultrasonics": self.ultrasonics(),
            "battery": self.battery(),
        }

    @property
    def motors(self):
        return None

    @property
    def servos(self):
        return None

    @property
    def buzzer(self):
        return None

    @property
    def kch(self):
        return None

    def battery(self) -> Optional[float]:
        return None

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)