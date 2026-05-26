# hw_io/bob_bot.py

from __future__ import annotations

import re
import time
from typing import Any, Callable, Dict, Iterable, Optional

from config import CONFIG
from hw_io.base import IOMap
from hw_io.cameras.base import Camera
from hw_io.cameras.resolve import resolve_camera
from hw_io.cameras.async_camera_proxy import AsyncCameraProxy
from hw_io.clients import (
    MegaSerialClient,
    MegaSerialConfig,
    StubUnoSerialClient,
    UnoSerialClient,
    UnoSerialConfig,
    UsbMediaClient,
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


class ReadOnlyCollection:
    """Named collection whose values are read directly from callables."""

    def __init__(self, getters: Dict[str, Callable[[], Any]]):
        self._getters = dict(getters)

    def __getitem__(self, key):
        return self._getters[key]()

    def keys(self):
        return self._getters.keys()

    def items(self):
        return {key: getter() for key, getter in self._getters.items()}.items()

    def values(self):
        return [getter() for getter in self._getters.values()]

    def as_dict(self):
        return {key: getter() for key, getter in self._getters.items()}


class VoltageReading:
    def __init__(self, getter: Callable[[], Optional[float]]):
        self._getter = getter

    @property
    def volts(self) -> Optional[float]:
        return self._getter()


class CurrentReading:
    def __init__(self, getter: Callable[[], Optional[float]]):
        self._getter = getter

    @property
    def amps(self) -> Optional[float]:
        return self._getter()


class EncoderReading:
    def __init__(self, getter: Callable[[], Dict[str, Any]]):
        self._getter = getter

    @property
    def count(self) -> Optional[int]:
        return self._getter().get("count")

    @property
    def timestamp_ms(self) -> Optional[int]:
        return self._getter().get("timestamp_ms")

    @property
    def valid(self) -> Optional[bool]:
        return self._getter().get("valid")

    @property
    def valid_flags(self) -> Optional[str]:
        return self._getter().get("valid_flags")


class QuadratureSnapshot:
    """
    Raw A/B encoder phase snapshot abstraction.

    Not used on BobBot.

    Retained for future robots that expose direct quadrature
    phase lines to the Pi or to a low-level MCU bridge.
    """

    def __init__(self, getter: Callable[[], tuple[Optional[bool], Optional[bool]]]):
        self._getter = getter

    @property
    def A(self) -> Optional[bool]:
        a, _ = self._getter()
        return a

    @property
    def B(self) -> Optional[bool]:
        _, b = self._getter()
        return b


class MegaSemanticMotor:
    def __init__(
        self,
        owner: "BobBotIO",
        mega: MegaSerialClient,
        name: str,
        polarity: int = 1,
    ):
        self._owner = owner
        self._mega = mega
        self._name = name
        self._polarity = 1 if polarity >= 0 else -1
        self._power = 0.0

    @property
    def power(self) -> float:
        return self._power

    @power.setter
    def power(self, value: float) -> None:
        value = max(-1.0, min(1.0, float(value)))

        starting_motion = (abs(self._power) < 1e-6) and (abs(value) > 1e-6)
        if starting_motion:
            self._owner.ensure_auto_mode(force=True)

        self._power = value
        self._owner._heartbeat_if_due(force=True)

        command_value = self._polarity * value
        print(f"[BOBBOT MOTOR] name={self._name} power={command_value}")
        resp = self._mega.motor_write(self._name, power=command_value)
        print(f"[BOBBOT MOTOR] resp={resp}")


class LedOutput:
    def __init__(self, write_fn):
        self._write_fn = write_fn
        self._brightness = 0.0

    @property
    def brightness(self) -> float:
        return self._brightness

    @brightness.setter
    def brightness(self, value: float) -> None:
        value = max(0.0, min(1.0, float(value)))
        self._brightness = value
        self._write_fn(value)


class DfPlayerAudio:
    def __init__(self, play_fn):
        self._play_fn = play_fn
        self._track = None

    @property
    def track(self):
        return self._track

    @track.setter
    def track(self, value: int) -> None:
        self._track = int(value)
        self._play_fn(self._track)


class PiezoAudio:
    def __init__(self, play_tone_fn):
        self._play_tone_fn = play_tone_fn

    def play_tone(self, tone: int, duration_ms: int = 250) -> None:
        self._play_tone_fn(int(tone), int(duration_ms))

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
      semantic io.* convention -> hard Mega/Uno API call
    """

    def __init__(self, robot, mega_client=None, uno_client=None, camera_manager=None):
        self.robot = robot
        self.camera_manager = camera_manager
        self._cameras: Dict[str, Camera] = {}
        self._outputs = NullOutputs()

        self.mega = mega_client if mega_client is not None else self._make_mega_client()
        self.uno = uno_client if uno_client is not None else self._make_uno_client()
        self.usb_media = UsbMediaClient()

        self._motor = None
        self._servo = None

        self._bumper = None
        self._reflectance = None
        self._ultrasonic = None
        self._limit = None
        self._voltage = None
        self._current = None
        self._encoder = None
        self._led = None
        self._audio = None
        self._usb = None

        self._hb_seq = 1
        self._last_hb_t = 0.0
        self._hb_period_s = float(getattr(CONFIG, "mega_heartbeat_period_s", 0.1))
        self._auto_entered = False

        self._detect_cameras()
        self._init_sensors()
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
        enabled = bool(getattr(CONFIG, "cameras_enabled", True))

        if not enabled:
            print("[CAMERA] camera detection disabled")
            return

        async_vision_enabled = bool(
            getattr(CONFIG, "async_vision_enabled", False)
        )

        if async_vision_enabled:
            if self.camera_manager is None:
                print("[CAMERA WARN] async vision enabled but no camera_manager provided")
                return

            for key in CONFIG.cameras.keys():
                self._cameras[key] = AsyncCameraProxy(
                    camera_name=key,
                    camera_manager=self.camera_manager,
                )
                print(f"[CAMERA] async proxy connected: {key}")

            return

        for key, camera_name in CONFIG.cameras.items():
            try:
                self._cameras[key] = resolve_camera(
                    camera_name=camera_name,
                    robot=self.robot,
                )
                print(f"[CAMERA] connected: {key}")

            except Exception as e:
                print(f"[CAMERA WARN] {key} unavailable: {e}")

    def _init_sensors(self):
        self._bumper = ReadOnlyCollection(
            {
                "front_left": lambda: self._read_bool(self.mega.bumper_read("front_left")),
                "front_right": lambda: self._read_bool(self.mega.bumper_read("front_right")),
            }
        )

        self._reflectance = ReadOnlyCollection(
            {
                "left": lambda: self._read_number(self.mega.reflectance_read("left")),
                "centre": lambda: self._read_number(self.mega.reflectance_read("centre")),
                "right": lambda: self._read_number(self.mega.reflectance_read("right")),
            }
        )

        self._ultrasonic = ReadOnlyCollection(
            {
                "front_left": lambda: self._read_number(self.mega.ultrasonic_read("front_left")),
                "front_right": lambda: self._read_number(self.mega.ultrasonic_read("front_right")),
            }
        )

        self._limit = ReadOnlyCollection(
            {
                "lift_high": lambda: self._read_bool(self.mega.limit_read("lift_high")),
                "lift_low": lambda: self._read_bool(self.mega.limit_read("lift_low")),
            }
        )

        self._voltage = NamedIndexedCollection(
            ordered_items=[],
            named_items={
                "battery": VoltageReading(lambda: self._read_voltage("battery"))
            },
        )

        self._current = NamedIndexedCollection(
            ordered_items=[],
            named_items={
                "gripper_right": CurrentReading(lambda: self._read_current("gripper_right")),
            },
        )

        self._encoder = NamedIndexedCollection(
            ordered_items=[],
            named_items={
                "drive_front_left": EncoderReading(lambda: self._read_encoder("drive_front_left")),
                "drive_front_right": EncoderReading(lambda: self._read_encoder("drive_front_right")),
                "deadwheel_parallel": EncoderReading(lambda: self._read_encoder("deadwheel_parallel")),
                "deadwheel_perpendicular": EncoderReading(lambda: self._read_encoder("deadwheel_perpendicular")),
                "shooter": EncoderReading(lambda: self._read_encoder("shooter")),
            },
        )

        self._usb = ReadOnlyCollection(
            {
                "match_zone": self._read_match_zone,
            }
        )

    def _init_actuators(self):
        if self.mega is None:
            return

        front_left = MegaSemanticMotor(
            self,
            self.mega,
            "drive_front_left",
            polarity=CONFIG.motor_polarity[0],
        )

        front_right = MegaSemanticMotor(
            self,
            self.mega,
            "drive_front_right",
            polarity=CONFIG.motor_polarity[1],
        )

        rear_left = MegaSemanticMotor(
            self,
            self.mega,
            "drive_rear_left",
            polarity=getattr(CONFIG, "motor_rear_left_polarity", 1),
        )

        rear_right = MegaSemanticMotor(
            self,
            self.mega,
            "drive_rear_right",
            polarity=getattr(CONFIG, "motor_rear_right_polarity", 1),
        )

        collector = MegaSemanticMotor(
            self,
            self.mega,
            "collector",
            polarity=getattr(CONFIG, "collector_motor_polarity", 1),
        )

        shooter = MegaSemanticMotor(
            self,
            self.mega,
            "shooter",
            polarity=getattr(CONFIG, "shooter_motor_polarity", 1),
        )

        self._motor = NamedIndexedCollection(
            ordered_items=[front_left, front_right],
            named_items={
                "drive_front_left": front_left,
                "drive_front_right": front_right,
                "drive_rear_left": rear_left,
                "drive_rear_right": rear_right,
                "collector": collector,
                "shooter": shooter,
            },
        )

        lift = MegaServoSigned(
            self,
            lambda value: self.mega.servo_write("lift", position=value),
        )
        gripper = MegaServoSigned(
            self,
            lambda value: self.mega.servo_write("gripper", position=value),
        )
        shooter_feed = MegaServoSigned(
            self,
            lambda value: self.mega.servo_write("shooter_feed", position=value),
        )

        self._servo = NamedIndexedCollection(
            ordered_items=[lift, gripper],
            named_items={
                "lift": lift,
                "gripper": gripper,
                "shooter_feed": shooter_feed,
            },
        )

        self._led = NamedIndexedCollection(
            ordered_items=[],
            named_items={
                "lisiparoi": LedOutput(
                    lambda brightness: self.mega.led_write("lisiparoi", brightness=brightness)
                ),
            },
        )

        self._audio = NamedIndexedCollection(
            ordered_items=[],
            named_items={
                "df_player": DfPlayerAudio(
                    lambda track: self.mega.audio_play("df_player", track=track)
                ),
                "piezo": PiezoAudio(
                    lambda tone, duration_ms: self.mega.audio_play(
                        "piezo",
                        tone=tone,
                        duration_ms=duration_ms,
                    )
                ),
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

    @staticmethod
    def _parse_quad(raw) -> tuple[Optional[bool], Optional[bool]]:
        if raw is None:
            return None, None

        if isinstance(raw, dict):
            a = raw.get("A", raw.get("a"))
            b = raw.get("B", raw.get("b"))
            return (None if a is None else bool(int(a))), (None if b is None else bool(int(b)))

        if isinstance(raw, (tuple, list)) and len(raw) >= 2:
            return bool(int(raw[0])), bool(int(raw[1]))

        text = str(raw).strip()

        # Accept common forms: "0 1", "A=0 B=1", "QUAD 22 24 0 1".
        nums = re.findall(r"-?\d+", text)
        if len(nums) >= 2:
            return bool(int(nums[-2])), bool(int(nums[-1]))

        # Accept compact two-bit response: "01".
        bits = re.findall(r"[01]", text)
        if len(bits) >= 2:
            return bool(int(bits[-2])), bool(int(bits[-1]))

        return None, None

    def _read_voltage(self, name: str) -> Optional[float]:
        if self.mega is None:
            return None

        raw = self.mega.voltage_read(name)
        parsed = self._parse_last_number(raw)

        print(f"[MEGA VOLTAGE] name={name} raw={raw!r} parsed={parsed!r}")

        return parsed

    def _read_number(self, raw) -> Optional[float]:
        return self._parse_last_number(raw)

    def _read_bool(self, raw) -> bool:
        text = str(raw or "").strip().lower()
        if text.endswith("1") or "value=1" in text:
            return True
        if text.endswith("0") or "value=0" in text:
            return False
        return text in {"true", "high", "on"}

    def _read_match_zone(self) -> int:
        source = CONFIG.match_zone_source.value.lower()

        # -------------------------
        # AUTO
        # -------------------------
        if "auto" in source:

            # Try SR API first
            try:
                if self.robot is not None:
                    zone = getattr(self.robot, "zone", None)
                    if zone is not None:
                        print(f"[MATCH ZONE] SR zone={zone}")
                        return int(zone)
            except Exception:
                pass

            # Try USB media
            try:
                zone = self.usb_media.read_int(
                    CONFIG.usb_match_zone_file
                )
                print(f"[MATCH ZONE] USB zone={zone}")
                return int(zone)
            except Exception:
                pass

            # Fallback
            print(f"[MATCH ZONE] FIXED zone={CONFIG.match_zone_fixed}")
            return int(CONFIG.match_zone_fixed)

        # -------------------------
        # SR
        # -------------------------
        if "sr" in source:
            return int(self.robot.zone)

        # -------------------------
        # USB
        # -------------------------
        if "usb" in source:
            zone = self.usb_media.read_int(
                CONFIG.usb_match_zone_file
            )
            return int(zone)

        # -------------------------
        # FIXED
        # -------------------------
        if "fixed" in source:
            return int(CONFIG.match_zone_fixed)

        raise ValueError(
            f"Unknown MATCH_ZONE_SOURCE: {CONFIG.match_zone_source}"
        )

    def _read_current(self, name: str) -> Optional[float]:
        if self.mega is None:
            return None
        raw = self.mega.current_read(name)
        parsed = self._parse_last_number(raw)
        print(f"[MEGA CURRENT] name={name} raw={raw!r} parsed={parsed!r}")
        return parsed

    def _parse_key_values(self, raw) -> Dict[str, str]:
        text = str(raw or "")
        out = {}
        for part in text.split():
            if "=" in part:
                key, value = part.split("=", 1)
                out[key] = value
        return out

    def _read_encoder(self, name: str) -> Dict[str, Any]:
        if self.mega is None:
            return {
                "count": None,
                "timestamp_ms": None,
                "valid": None,
                "valid_flags": None,
            }

        raw = self.mega.encoder_read(name)
        fields = self._parse_key_values(raw)

        print(f"[MEGA ENCODER] name={name} raw={raw!r} fields={fields!r}")

        def as_int(key):
            try:
                return int(fields[key])
            except Exception:
                return None

        return {
            "count": as_int("count"),
            "timestamp_ms": as_int("timestamp_ms"),
            "valid": None if "valid" not in fields else bool(int(fields["valid"])),
            "valid_flags": fields.get("valid_flags"),
        }

    @property
    def outputs(self):
        return self._outputs

    @property
    def camera(self) -> Dict[str, Camera]:
        return dict(self._cameras)

    def cameras(self) -> Dict[str, Camera]:
        return dict(self._cameras)

    @property
    def bumper(self):
        return self._bumper

    def bumpers(self) -> Dict[str, bool]:
        return self._bumper.as_dict()

    @property
    def reflectance(self):
        return self._reflectance

    def reflectance_values(self) -> Dict[str, float]:
        return self._reflectance.as_dict()

    @property
    def ultrasonic(self):
        return self._ultrasonic

    @property
    def limit(self):
        return self._limit

    def ultrasonics(self) -> Dict[str, Optional[float]]:
        return self._ultrasonic.as_dict()

    @property
    def voltage(self):
        return self._voltage

    @property
    def current(self):
        return self._current

    @property
    def encoder(self):
        return self._encoder

    @property
    def usb(self):
        return self._usb

    @property
    def motor(self):
        return self._motor

    @property
    def motors(self):
        return self._motor

    @property
    def servo(self):
        return self._servo

    @property
    def servos(self):
        return self._servo

    def sense(self) -> Dict[str, Any]:
        return {
            "bumper": self.bumpers(),
            "reflectance": self.reflectance_values(),
            "ultrasonic": self.ultrasonics(),
            "limit": {
                "lift_high": self.limit["lift_high"],
                "lift_low": self.limit["lift_low"],
            },
            "voltage": {
                "battery": self.voltage["battery"].volts,
            },
            "current": {
                "gripper_right": self.current["gripper_right"].amps,
            },
        }

    def battery(self) -> Dict[str, Optional[float]]:
        return {"voltage": self.voltage["battery"].volts}

    @property
    def led(self):
        return self._led

    @property
    def audio(self):
        return self._audio

    @property
    def buzzer(self):
        return self._audio["piezo"]

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
