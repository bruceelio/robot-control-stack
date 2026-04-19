# hw_io/bob_bot.py

from __future__ import annotations

import re
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
    def __init__(self, owner: "BobBotIO", mega: MegaSerialClient, terminal: str, polarity: int = 1):
        self._owner = owner
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

        # If starting a new motion burst, re-arm every time.
        starting_motion = (abs(self._power) < 1e-6) and (abs(value) > 1e-6)
        if starting_motion:
            self._owner.ensure_auto_mode(force=True)

        self._power = value
        self._owner._heartbeat_if_due(force=True)
        print(f"[BOBBOT MOTOR] terminal={self._terminal} value={value}")
        resp = self._mega.link_18_19(self._terminal, self._polarity * value)
        print(f"[BOBBOT MOTOR] resp={resp}")


class MegaServoSigned:
    """
    Sends signed -1..1 values through directly.

    Current working Mega sketch behavior:
      servo_write(12, +/-1.0)  # lift
      servo_write(11, +/-1.0)  # mirrored gripper
    """

    def __init__(self, owner: "BobBotIO", write_fn):
        self._owner = owner
        self._write_fn = write_fn
        self._position = None

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        self._position = value
        if value is None:
            return
        value = max(-1.0, min(1.0, float(value)))
        self._owner.ensure_auto_mode(force=True)
        self._owner._heartbeat_if_due(force=True)
        self._write_fn(value)


class BobBotIO(IOMap):
    """
    BobBot bridge layer:
      current code shape -> semantic convention -> hard API call
    """

    def __init__(self, robot, mega_client=None, uno_client=None):
        self.robot = robot
        self._cameras: Dict[str, Camera] = {}
        self._outputs = NullOutputs()

        self.mega = mega_client if mega_client is not None else self._make_mega_client()
        self.uno = uno_client if uno_client is not None else self._make_uno_client()

        self._motors = None
        self._servos = None

        self._hb_seq = 1
        self._last_hb_t = 0.0
        self._hb_period_s = float(getattr(CONFIG, "mega_heartbeat_period_s", 0.1))
        self._auto_entered = False

        self._detect_cameras()
        self._init_actuators()

    def ensure_auto_mode(self, force: bool = False) -> None:
        if self.mega is None:
            return

        now = time.monotonic()
        stale_s = float(getattr(CONFIG, "mega_auto_stale_s", 0.4))
        needs_rearm = force or (not self._auto_entered) or ((now - self._last_hb_t) > stale_s)

        if not needs_rearm:
            return

        print("[MEGA INIT] re-arming AUTO")

        try:
            print(f"[MEGA INIT] {self.mega.hello()}")
        except Exception as e:
            print(f"[MEGA INIT] hello failed: {e}")

        try:
            print(f"[MEGA INIT] {self.mega.mode_auto()}")
            self._hb_seq = 1
            self._last_hb_t = 0.0
            self._auto_entered = True
            self._heartbeat_if_due(force=True)
        except Exception as e:
            print(f"[MEGA INIT] mode_auto failed: {e}")
            self._auto_entered = False

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
            self,
            self.mega,
            "M1",
            polarity=CONFIG.motor_polarity[0],
        )
        right = MegaDriveMotor(
            self,
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

        lift = MegaServoSigned(
            self,
            lambda value: self.mega.servo_write(12, value),
        )
        gripper = MegaServoSigned(
            self,
            lambda value: self.mega.servo_write(11, value),
        )
        self._servos = NamedIndexedCollection(
            ordered_items=[lift, gripper],
            named_items={
                "lift": lift,
                "gripper": gripper,
            },
        )

    def _heartbeat_if_due(self, *, force: bool = False) -> None:
        if self.mega is None or not self._auto_entered:
            return

        now = time.monotonic()
        if (not force) and (now - self._last_hb_t < self._hb_period_s):
            return

        try:
            resp = self.mega.heartbeat(self._hb_seq)
            print(f"[MEGA HB] seq={self._hb_seq} resp={resp}")
            self._hb_seq += 1
            self._last_hb_t = now
        except Exception as e:
            print(f"[MEGA HB] error: {e}")
            self._auto_entered = False

    @staticmethod
    def _parse_last_number(raw) -> Optional[float]:
        if raw is None:
            return None
        if isinstance(raw, (int, float)):
            return float(raw)
        text = str(raw).strip()
        m = re.search(r"(-?\d+(?:\.\d+)?)\s*$", text)
        if not m:
            return None
        try:
            return float(m.group(1))
        except ValueError:
            return None

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
        parsed = self._parse_last_number(value)
        return dict(
            left=0.0,
            center=0.0 if parsed is None else float(parsed),
            right=0.0,
        )

    def ultrasonics(self) -> Dict[str, Optional[float]]:
        value = self.uno.range_read(2, 3)
        parsed = self._parse_last_number(value)
        return dict(
            front=parsed,
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
        parsed = self._parse_last_number(raw)
        return {"voltage": parsed}

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
        end = time.monotonic() + secs
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0:
                break
            self._heartbeat_if_due()
            time.sleep(min(0.05, remaining))

    def close(self) -> None:
        if self.mega is not None:
            try:
                print(f"[MEGA CLOSE] {self.mega.stop()}")
            except Exception as e:
                print(f"[MEGA CLOSE] stop failed: {e}")
            try:
                print(f"[MEGA CLOSE] {self.mega.mode_teleop()}")
            except Exception as e:
                print(f"[MEGA CLOSE] mode_teleop failed: {e}")

        for client in (self.uno, self.mega):
            close = getattr(client, "close", None)
            if callable(close):
                try:
                    close()
                except Exception as e:
                    print(f"[BOBBOT CLOSE] close failed: {e}")