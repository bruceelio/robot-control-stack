# hwio/bob_bot.py

from __future__ import annotations

import time
from typing import Any, Dict, Iterable, Optional

from config import CONFIG
from hw_io.base import IOMap
from hw_io.cameras.base import Camera
from hw_io.cameras.resolve import resolve_camera
from hw_io.clients import (
    MegaSerialClient,
    MegaSerialConfig,
    StubUnoSerialClient,
    UnoSerialClient,
    UnoSerialConfig,
)


class NullOutputs:
    def __init__(self):
        self._state = {"VACUUM": False}

    def names(self) -> Iterable[str]:
        return self._state.keys()

    def set(self, name: str, on: bool) -> None:
        self._state[name] = bool(on)
        print(f"[NullOutputs] {name} -> {on} (not implemented yet)")

    def get(self, name: str) -> bool:
        return self._state.get(name, False)


class NamedIndexedCollection:
    """Bridge old index access and new semantic-name access."""

    def __init__(self, ordered_items, named_items):
        self._ordered = list(ordered_items)
        self._named = dict(named_items)

    def __getitem__(self, key):
        if isinstance(key, int):
            return self._ordered[key]
        return self._named[key]

    def __len__(self):
        return len(self._ordered)

    def keys(self):
        return self._named.keys()

    def values(self):
        return self._named.values()

    def items(self):
        return self._named.items()


class MegaDriveMotor:
    def __init__(self, mega: MegaSerialClient, terminal: str, polarity: int = 1):
        self._mega = mega
        self._terminal = terminal
        self._polarity = 1 if polarity >= 0 else -1
        self._power = 0.0

    @property
    def power(self) -> float:
        return self._power

    @power.setter
    def power(self, value: float) -> None:
        value = max(-1.0, min(1.0, float(value)))
        self._power = value
        self._mega.link_18_19(self._terminal, self._polarity * value)


class MegaServo:
    def __init__(self, write_fn, *, signed_input: bool = False):
        self._write_fn = write_fn
        self._signed_input = signed_input
        self._position = None

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        self._position = value
        if value is None:
            return

        value = float(value)
        if self._signed_input:
            value = (max(-1.0, min(1.0, value)) + 1.0) / 2.0
        else:
            value = max(0.0, min(1.0, value))
        self._write_fn(value)


class BobBotIO(IOMap):
    """
    BobBot bridge layer:
      current code shape -> final semantic convention -> hard API call

    Current code expectations preserved:
    - io.bumpers()["fl"]
    - io.ultrasonics()["front"]
    - io.reflectance()["center"]
    - io.cameras()["front"].see()
    - io.motors[0] / io.motors[1]
    - io.servos[0]

    Future convention supported in parallel:
    - io.motors["drive_front_left"]
    - io.motors["drive_front_right"]
    - io.servos["lift"]
    - io.servos["gripper"]
    """

    def __init__(self, robot, mega_client=None, uno_client=None):
        self.robot = robot
        self._cameras: Dict[str, Camera] = {}
        self._outputs = NullOutputs()

        self.mega = mega_client if mega_client is not None else self._make_mega_client()
        self.uno = uno_client if uno_client is not None else self._make_uno_client()

        self._motors = None
        self._servos = None

        self._detect_cameras()
        self._init_actuators()

    def _make_mega_client(self) -> MegaSerialClient | None:
        enabled = bool(getattr(CONFIG, "mega_enabled", True))
        if not enabled:
            return None

        cfg = MegaSerialConfig(
            port=getattr(CONFIG, "mega_port", "/dev/ttyACM0"),
            baud=getattr(CONFIG, "mega_baud", 115200),
            timeout=getattr(CONFIG, "mega_timeout", 1.0),
            open_delay_s=getattr(CONFIG, "mega_open_delay_s", 2.0),
        )
        client = MegaSerialClient(cfg)
        client.open()
        return client

    def _make_uno_client(self):
        enabled = bool(getattr(CONFIG, "uno_enabled", False))
        cfg = UnoSerialConfig(
            port=getattr(CONFIG, "uno_port", "/dev/ttyACM1"),
            baud=getattr(CONFIG, "uno_baud", 115200),
            timeout=getattr(CONFIG, "uno_timeout", 1.0),
            open_delay_s=getattr(CONFIG, "uno_open_delay_s", 2.0),
            enabled=enabled,
        )
        if not enabled:
            return StubUnoSerialClient()
        client = UnoSerialClient(cfg)
        client.open()
        return client

    def _detect_cameras(self):
        for key, camera_name in CONFIG.cameras.items():
            self._cameras[key] = resolve_camera(
                camera_name=camera_name,
                robot=self.robot,
            )

    def _init_actuators(self):
        if self.mega is None:
            return

        left = MegaDriveMotor(
            self.mega,
            "M1",
            polarity=CONFIG.motor_polarity[0],
        )
        right = MegaDriveMotor(
            self.mega,
            "M2",
            polarity=CONFIG.motor_polarity[1],
        )
        self._motors = NamedIndexedCollection(
            ordered_items=[left, right],
            named_items={
                "drive_front_left": left,
                "drive_front_right": right,
            },
        )

        lift = MegaServo(
            lambda value: self.mega.servo_write(12, value),
            signed_input=True,
        )
        gripper = MegaServo(
            lambda value: self.mega.group_write(11, value, 13, 1.0 - value),
            signed_input=False,
        )
        self._servos = NamedIndexedCollection(
            ordered_items=[lift, gripper],
            named_items={
                "lift": lift,
                "gripper": gripper,
            },
        )

    @property
    def outputs(self):
        return self._outputs

    def cameras(self) -> Dict[str, Camera]:
        return dict(self._cameras)

    def bumpers(self) -> Dict[str, bool]:
        return dict(
            fl=bool(self.uno.digital_read(10)),
            fr=False,
            rl=False,
            rr=False,
        )

    def reflectance(self) -> Dict[str, float]:
        value = self.uno.analog_read("A1")
        return dict(
            left=0.0,
            center=0.0 if value is None else float(value),
            right=0.0,
        )

    def ultrasonics(self) -> Dict[str, Optional[float]]:
        return dict(
            front=self.uno.range_read(2, 3),
            left=None,
            right=None,
            back=None,
        )

    def sense(self) -> Dict[str, Any]:
        return {
            "bumpers": self.bumpers(),
            "reflectance": self.reflectance(),
            "ultrasonics": self.ultrasonics(),
            "battery": self.battery(),
        }

    def battery(self) -> Dict[str, Optional[float]]:
        if self.mega is None:
            return {"voltage": None}
        raw = self.mega.analog_read("47")
        return {"voltage": None if raw is None else float(raw)}

    @property
    def motors(self):
        return self._motors

    @property
    def servos(self):
        return self._servos

    @property
    def buzzer(self):
        return None

    @property
    def kch(self):
        return None

    @property
    def battery_sensor(self):
        return self.battery()

    def sleep(self, secs: float) -> None:
        time.sleep(secs)

    def close(self) -> None:
        for client in (self.uno, self.mega):
            close = getattr(client, "close", None)
            if callable(close):
                close()
