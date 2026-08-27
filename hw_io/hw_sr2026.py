# hw_io/hw_sr2026.py

from __future__ import annotations

import time
from typing import Any, Callable, Dict, Iterable, Optional

from config import CONFIG
from hw_io.base import IOMap
from hw_io.cameras.base import Camera
from hw_io.cameras.sr_april import SRAprilCamera
from sr.robot3 import A0, A1, A2


class NamedIndexedCollection:
    """
    Bridge legacy integer-index access and canonical semantic-name access.

    Examples:
        io.motors[0]
        io.motor["drive_front_left"]

        io.servos[0]
        io.servo["lift"]
    """

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
    """Named collection whose values are obtained from read callables."""

    def __init__(self, getters: Dict[str, Callable[[], Any]]):
        self._getters = dict(getters)

    def __getitem__(self, key):
        return self._getters[key]()

    def keys(self):
        return self._getters.keys()

    def values(self):
        return [getter() for getter in self._getters.values()]

    def items(self):
        return {
            key: getter()
            for key, getter in self._getters.items()
        }.items()

    def as_dict(self):
        return {
            key: getter()
            for key, getter in self._getters.items()
        }


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


class SR2026SemanticMotor:
    """Canonical motor wrapper around one SR Motor Board channel."""

    def __init__(self, motor, *, polarity: int = 1):
        self._motor = motor
        self._polarity = 1 if polarity >= 0 else -1
        self._power = 0.0

    @property
    def power(self) -> float:
        return self._power

    @power.setter
    def power(self, value: float) -> None:
        value = max(-1.0, min(1.0, float(value)))
        self._power = value
        self._motor.power = self._polarity * value


class SR2026SemanticDrive:
    """
    Canonical paired drive interface.

    SR exposes the two Motor Board channels separately, so this wrapper
    applies the requested left/right values as one semantic operation.
    """

    def __init__(
        self,
        left_motor: SR2026SemanticMotor,
        right_motor: SR2026SemanticMotor,
    ):
        self._left_motor = left_motor
        self._right_motor = right_motor
        self._left_power = 0.0
        self._right_power = 0.0

    @property
    def left_power(self) -> float:
        return self._left_power

    @property
    def right_power(self) -> float:
        return self._right_power

    def set_power(self, *, left: float, right: float) -> None:
        left = max(-1.0, min(1.0, float(left)))
        right = max(-1.0, min(1.0, float(right)))

        self._left_power = left
        self._right_power = right

        self._left_motor.power = left
        self._right_motor.power = right


class SR2026Piezo:
    """
    Power Board piezo wrapper.

    Supports both:
        io.buzzer().buzz(...)
        io.audio["piezo"].play_tone(...)
    """

    def __init__(self, power_board):
        self._power = power_board

    def buzz(self, tone, duration: float, *, blocking: bool = False) -> None:
        if self._power is None:
            print(
                f"[SR2026 PIEZO] tone={tone} duration={duration} "
                "(no power board)"
            )
            return

        self._power.piezo.buzz(
            tone,
            float(duration),
            blocking=blocking,
        )

    def off(self) -> None:
        if self._power is None:
            return

        self._power.piezo.buzz(
            0,
            0,
            blocking=False,
        )

    def play_tone(self, tone: int, duration_ms: int = 250) -> None:
        self.buzz(
            tone,
            float(duration_ms) / 1000.0,
            blocking=False,
        )


class SR2026KCH:
    """Compatibility wrapper for older io.kch() callers."""

    def __init__(self, kch):
        self._kch = kch

    def set_colour(self, led_index, colour) -> None:
        self._kch.leds[led_index].colour = colour

    def set_rgb(self, led_index, *, r=None, g=None, b=None) -> None:
        led = self._kch.leds[led_index]

        if r is not None:
            led.r = bool(r)
        if g is not None:
            led.g = bool(g)
        if b is not None:
            led.b = bool(b)


class SR2026Outputs:
    """
    Compatibility digital-output interface.

    VACUUM is currently mapped to Power Board high-current output H0.
    """

    def __init__(self, power_board):
        self._power = power_board
        self._state = {"VACUUM": False}

    def names(self) -> Iterable[str]:
        return self._state.keys()

    def set(self, name: str, on: bool) -> None:
        if name != "VACUUM":
            raise KeyError(name)

        on = bool(on)
        self._state[name] = on

        if self._power is None:
            print(f"[SR2026Outputs] {name} -> {on} (no power board)")
            return

        try:
            from sr.robot3 import OUT_H0

            output = self._power.outputs[OUT_H0]
            output.is_enabled = on

            actual = output.is_enabled
            print(
                f"[SR2026Outputs] {name} -> {on} "
                f"via OUT_H0 (actual={actual})"
            )

        except Exception as exc:
            print(
                f"[SR2026Outputs] {name} -> {on} "
                f"failed ({exc})"
            )

    def get(self, name: str) -> bool:
        if name not in self._state:
            raise KeyError(name)

        if self._power is not None:
            try:
                from sr.robot3 import OUT_H0

                return bool(
                    self._power.outputs[OUT_H0].is_enabled
                )
            except Exception:
                pass

        return self._state[name]


class SR2026IO(IOMap):
    """
    Student Robotics 2026 / Webots IO backend.

    Public interface:
        canonical io.* semantic devices

    Hardware implementation:
        sr.robot3 boards and Arduino-compatible IO

    This backend is used by:
        - SR2026 Webots simulation
        - future physical SR2026 hardware

    All SR-specific board addressing and pin/channel numbers belong here.
    """

    # --------------------------------------------------
    # SR Arduino pin mapping
    # --------------------------------------------------

    PIN_BUMPER_FRONT_LEFT = 10
    PIN_BUMPER_FRONT_RIGHT = 11

    PIN_REFLECTANCE_LEFT = A0
    PIN_REFLECTANCE_CENTRE = A1
    PIN_REFLECTANCE_RIGHT = A2

    # --------------------------------------------------
    # SR board channel mapping
    # --------------------------------------------------

    MOTOR_FRONT_LEFT_CHANNEL = 0
    MOTOR_FRONT_RIGHT_CHANNEL = 1

    # Hardware matrix explicitly assigns gripper to Servo Board channel 0.
    SERVO_GRIPPER_CHANNEL = 0

    # The current matrix leaves the lift channel unspecified.
    # Channel 1 is the working/provisional SR mapping and is isolated here
    # so it can be changed in one place if required.
    SERVO_LIFT_CHANNEL = 1

    # --------------------------------------------------
    # Construction
    # --------------------------------------------------

    def __init__(self, robot):
        self.robot = robot

        # Current SR API name first; retain CircuitPython fallback for
        # simulator/firmware compatibility if encountered.
        self.arduino = getattr(robot, "arduino", None)
        if self.arduino is None:
            self.arduino = getattr(robot, "CircuitPython", None)

        self._power = getattr(robot, "power_board", None)
        self._motor_board = getattr(robot, "motor_board", None)
        self._servo_board = getattr(robot, "servo_board", None)
        self._kch_raw = getattr(robot, "kch", None)

        self._outputs = SR2026Outputs(self._power)

        self._cameras: Dict[str, Camera] = {}

        # Canonical collections are always present.
        # Unsupported/unavailable capabilities use empty collections.
        self._bumper = ReadOnlyCollection({})
        self._reflectance = ReadOnlyCollection({})
        self._ultrasonic = ReadOnlyCollection({})

        self._voltage = NamedIndexedCollection([], {})
        self._current = NamedIndexedCollection([], {})
        self._encoder = NamedIndexedCollection([], {})

        self._motor = NamedIndexedCollection([], {})
        self._drive = NamedIndexedCollection([], {})
        self._servo = NamedIndexedCollection([], {})

        self._led = NamedIndexedCollection([], {})
        self._audio = NamedIndexedCollection([], {})

        self._piezo: SR2026Piezo | None = None
        self._kch_compat: SR2026KCH | None = None

        self._init_arduino_pins()
        self._detect_cameras()
        self._init_sensors()
        self._init_actuators()
        self._init_leds_and_audio()

    # --------------------------------------------------
    # Arduino setup / helpers
    # --------------------------------------------------

    def _init_arduino_pins(self) -> None:
        if self.arduino is None:
            return

        try:
            from sr.robot3 import INPUT, INPUT_PULLUP
        except Exception:
            return

        pins = getattr(self.arduino, "pins", None)
        if pins is None:
            return

        # Bumpers are wired as pull-up digital inputs.
        for pin in (
            self.PIN_BUMPER_FRONT_LEFT,
            self.PIN_BUMPER_FRONT_RIGHT,
        ):
            pins[pin].mode = INPUT_PULLUP

        # Reflectance sensors are analog inputs.
        for pin in (
            self.PIN_REFLECTANCE_LEFT,
            self.PIN_REFLECTANCE_CENTRE,
            self.PIN_REFLECTANCE_RIGHT,
        ):
            pins[pin].mode = INPUT

    def _digital_read(self, pin: int) -> bool:
        if self.arduino is None:
            return False

        pins = getattr(self.arduino, "pins", None)
        if pins is not None:
            return bool(pins[pin].digital_read())

        # Compatibility fallback for older/alternate SR wrappers.
        read_fn = getattr(self.arduino, "digital_read", None)
        if callable(read_fn):
            return bool(read_fn(pin))

        raise AttributeError("SR Arduino digital read API not found")

    def _analog_read(self, pin) -> float:
        if self.arduino is None:
            return 0.0

        pins = getattr(self.arduino, "pins", None)
        if pins is not None:
            value = pins[pin].analog_read()
        else:
            # Compatibility fallback for older/alternate SR wrappers.
            read_fn = getattr(self.arduino, "analog_read", None)
            if not callable(read_fn):
                raise AttributeError(
                    "SR Arduino analog read API not found"
                )
            value = read_fn(pin)

        if isinstance(value, (tuple, list)):
            value = value[0] if value else 0.0

        return float(value)

    # --------------------------------------------------
    # Camera
    # --------------------------------------------------

    def _detect_cameras(self) -> None:
        sr_camera = getattr(self.robot, "camera", None)

        if sr_camera is not None:
            self._cameras["front"] = SRAprilCamera(sr_camera)

    # --------------------------------------------------
    # Sensors
    # --------------------------------------------------

    def _init_sensors(self) -> None:
        self._bumper = ReadOnlyCollection(
            {
                "front_left": lambda: self._digital_read(
                    self.PIN_BUMPER_FRONT_LEFT
                ),
                "front_right": lambda: self._digital_read(
                    self.PIN_BUMPER_FRONT_RIGHT
                ),
            }
        )

        self._reflectance = ReadOnlyCollection(
            {
                "left": lambda: self._analog_read(
                    self.PIN_REFLECTANCE_LEFT
                ),
                "centre": lambda: self._analog_read(
                    self.PIN_REFLECTANCE_CENTRE
                ),
                "right": lambda: self._analog_read(
                    self.PIN_REFLECTANCE_RIGHT
                ),
            }
        )

        # Webots/SR2026 profile currently declares no ultrasonic devices.
        self._ultrasonic = ReadOnlyCollection({})

        self._voltage = NamedIndexedCollection(
            ordered_items=[],
            named_items={
                "battery": VoltageReading(
                    self._read_battery_voltage
                ),
            },
        )

        self._current = NamedIndexedCollection(
            ordered_items=[],
            named_items={
                "battery": CurrentReading(
                    self._read_battery_current
                ),
            },
        )

        # No SR2026 encoder capability is currently configured.
        self._encoder = NamedIndexedCollection(
            ordered_items=[],
            named_items={},
        )

    def _read_battery_voltage(self) -> Optional[float]:
        sensor = getattr(self._power, "battery_sensor", None)
        if sensor is None:
            return None

        value = getattr(sensor, "voltage", None)
        return None if value is None else float(value)

    def _read_battery_current(self) -> Optional[float]:
        sensor = getattr(self._power, "battery_sensor", None)
        if sensor is None:
            return None

        value = getattr(sensor, "current", None)
        return None if value is None else float(value)

    # --------------------------------------------------
    # Motors / drive / servos
    # --------------------------------------------------

    def _init_actuators(self) -> None:
        raw_motors = getattr(self._motor_board, "motors", None)

        if raw_motors is not None:
            left_polarity = 1
            right_polarity = 1

            motor_polarity = getattr(
                CONFIG,
                "motor_polarity",
                [1, 1],
            )

            if len(motor_polarity) >= 2:
                left_polarity = motor_polarity[0]
                right_polarity = motor_polarity[1]

            front_left = SR2026SemanticMotor(
                raw_motors[self.MOTOR_FRONT_LEFT_CHANNEL],
                polarity=left_polarity,
            )

            front_right = SR2026SemanticMotor(
                raw_motors[self.MOTOR_FRONT_RIGHT_CHANNEL],
                polarity=right_polarity,
            )

            self._motor = NamedIndexedCollection(
                ordered_items=[
                    front_left,
                    front_right,
                ],
                named_items={
                    "drive_front_left": front_left,
                    "drive_front_right": front_right,
                },
            )

            front_drive = SR2026SemanticDrive(
                front_left,
                front_right,
            )

            self._drive = NamedIndexedCollection(
                ordered_items=[front_drive],
                named_items={
                    "front": front_drive,
                },
            )

        raw_servos = getattr(self._servo_board, "servos", None)

        if raw_servos is not None:
            gripper = raw_servos[self.SERVO_GRIPPER_CHANNEL]
            lift = raw_servos[self.SERVO_LIFT_CHANNEL]

            self._servo = NamedIndexedCollection(
                # Preserve existing Level2 compatibility:
                #   servos[0] -> lift
                #   servos[1] -> gripper
                ordered_items=[
                    lift,
                    gripper,
                ],
                named_items={
                    "lift": lift,
                    "gripper": gripper,
                },
            )

    # --------------------------------------------------
    # LEDs / audio
    # --------------------------------------------------

    def _init_leds_and_audio(self) -> None:
        if self._kch_raw is not None:
            try:
                from sr.robot3 import LED_A, LED_B, LED_C

                self._led = NamedIndexedCollection(
                    ordered_items=[],
                    named_items={
                        "a": self._kch_raw.leds[LED_A],
                        "b": self._kch_raw.leds[LED_B],
                        "c": self._kch_raw.leds[LED_C],
                    },
                )

                self._kch_compat = SR2026KCH(
                    self._kch_raw
                )

            except Exception as exc:
                print(
                    "[SR2026 LED] KCH semantic mapping unavailable: "
                    f"{exc}"
                )

        if self._power is not None:
            self._piezo = SR2026Piezo(self._power)

            self._audio = NamedIndexedCollection(
                ordered_items=[],
                named_items={
                    "piezo": self._piezo,
                },
            )

    # --------------------------------------------------
    # Canonical direct IO collections
    # --------------------------------------------------

    @property
    def bumper(self):
        return self._bumper

    @property
    def reflectance(self):
        return self._reflectance

    @property
    def ultrasonic(self):
        return self._ultrasonic

    @property
    def current(self):
        return self._current

    @property
    def voltage(self):
        return self._voltage

    @property
    def encoder(self):
        return self._encoder

    @property
    def camera(self) -> Dict[str, Camera]:
        return dict(self._cameras)

    @property
    def motor(self):
        return self._motor

    @property
    def drive(self):
        return self._drive

    @property
    def servo(self):
        return self._servo

    @property
    def led(self):
        return self._led

    @property
    def audio(self):
        return self._audio

    # --------------------------------------------------
    # Compatibility interfaces
    # --------------------------------------------------

    @property
    def outputs(self):
        return self._outputs

    def bumpers(self) -> Dict[str, bool]:
        return self._bumper.as_dict()

    def reflectance_values(self) -> Dict[str, float]:
        return self._reflectance.as_dict()

    def ultrasonics(self) -> Dict[str, Optional[float]]:
        return self._ultrasonic.as_dict()

    def cameras(self) -> Dict[str, Camera]:
        return dict(self._cameras)

    @property
    def motors(self):
        return self._motor

    @property
    def servos(self):
        return self._servo

    def battery(self) -> Dict[str, Optional[float]]:
        return {
            "voltage": self._read_battery_voltage(),
            "current": self._read_battery_current(),
        }

    @property
    def battery_sensor(self):
        return self.battery()

    def kch(self):
        return self._kch_compat

    def buzzer(self):
        return self._piezo

    def wait_start(self):
        wait_fn = getattr(self.robot, "wait_start", None)
        if callable(wait_fn):
            return wait_fn()
        return None

    # --------------------------------------------------
    # Unified sensor snapshot
    # --------------------------------------------------

    def sense(self) -> Dict[str, Any]:
        return {
            "bumper": self.bumpers(),
            "reflectance": self.reflectance_values(),
            "ultrasonic": self.ultrasonics(),
            "voltage": {
                "battery": self.voltage["battery"].volts,
            },
            "current": {
                "battery": self.current["battery"].amps,
            },
        }

    # --------------------------------------------------
    # Timing / shutdown
    # --------------------------------------------------

    def sleep(self, secs: float) -> None:
        sleep_fn = getattr(self.robot, "sleep", None)

        if callable(sleep_fn):
            sleep_fn(float(secs))
        else:
            time.sleep(float(secs))

    def close(self) -> None:
        # Best-effort safe shutdown.
        try:
            if "front" in self._drive.keys():
                self._drive["front"].set_power(
                    left=0.0,
                    right=0.0,
                )
        except Exception:
            pass

        try:
            if "lift" in self._servo.keys():
                self._servo["lift"].position = None
        except Exception:
            pass

        try:
            if "gripper" in self._servo.keys():
                self._servo["gripper"].position = None
        except Exception:
            pass

        try:
            self._outputs.set("VACUUM", False)
        except Exception:
            pass

        try:
            if self._piezo is not None:
                self._piezo.off()
        except Exception:
            pass
