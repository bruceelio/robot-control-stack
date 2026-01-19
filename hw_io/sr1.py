# hw_io/sr1.py

from __future__ import annotations
from typing import Dict, Optional, Any
from hw_io.base import IOMap
from hw_io.cameras.base import Camera
from hw_io.cameras.sr_april import SRAprilCamera
from sr.robot3 import A0, A1, A2  # put at top of file (inside try if you want)

import time

class SR1Buzzer:
    def __init__(self, power_board):
        self._power = power_board

    def on(self, note=None, duration: float = 0.5):
        if not self._power:
            print("[SR1Buzzer] no power board")
            return
        try:
            # note is optional; SR supports Note enum but keep this generic
            if note is None:
                from sr.robot3 import Note
                note = Note.A6
            self._power.piezo.buzz(note, duration)
        except Exception as e:
            print(f"[SR1Buzzer] buzz failed ({e})")

    def off(self):
        if not self._power:
            print("[SR1Buzzer] no power board")
            return
        try:
            self._power.piezo.buzz(8, 0)  # valid freq, zero duration
        except Exception as e:
            print(f"[SR1Buzzer] off failed ({e})")


class SR1KCH:
    def __init__(self, kch):
        self._kch = kch

    def set_colour(self, led_index, colour):
        self._kch.leds[led_index].colour = colour

    def set_rgb(self, led_index, *, r=None, g=None, b=None):
        led = self._kch.leds[led_index]
        if r is not None: led.r = bool(r)
        if g is not None: led.g = bool(g)
        if b is not None: led.b = bool(b)



class SR1Outputs:
    def __init__(self, power_board):
        self._power = power_board
        self._state = {"VACUUM": False}

    def names(self):
        return self._state.keys()

    def set(self, name: str, on: bool) -> None:
        if name != "VACUUM":
            raise KeyError(name)

        self._state[name] = bool(on)

        if not self._power:
            print(f"[SR1Outputs] {name} -> {on} (no power board)")
            return

        try:
            from sr.robot3 import OUT_H0
            self._power.outputs[OUT_H0].is_enabled = bool(on)
            # Read back to confirm
            actual = self._power.outputs[OUT_H0].is_enabled
            print(f"[SR1Outputs] {name} -> {on} via OUT_H0 (actual={actual})")
        except Exception as e:
            print(f"[SR1Outputs] {name} -> {on} failed ({e})")

    def get(self, name: str) -> bool:
        return self._state[name]


class SR1IO(IOMap):
    """
    IO mapping for SR1 robot.

    Binds canonical IOMap interfaces to:
    - SR API devices when available
    - SR1-compatible hardware layout

    This is the ONLY place that knows about:
    - SR API
    - pin numbers
    - board presence
    """

    # --------------------------------------------------
    # Arduino pin mapping (SR1 standard)
    # --------------------------------------------------

    PIN_BUMPER_FL = 10
    PIN_BUMPER_FR = 11
    PIN_BUMPER_RL = 12
    PIN_BUMPER_RR = 13

    PIN_REFLECT_LEFT = "A0"
    PIN_REFLECT_CENTER = "A1"
    PIN_REFLECT_RIGHT = "A2"

    PIN_US_FRONT_TRIG = 2
    PIN_US_FRONT_ECHO = 3
    PIN_US_LEFT_TRIG = 4
    PIN_US_LEFT_ECHO = 5
    PIN_US_RIGHT_TRIG = 6
    PIN_US_RIGHT_ECHO = 7
    PIN_US_BACK_TRIG = 8
    PIN_US_BACK_ECHO = 9

    # --------------------------------------------------
    # Construction
    # --------------------------------------------------

    def __init__(self, robot):
        self.robot = robot

        # Boards (may be None)
        self.arduino = getattr(robot, "arduino", None)
        self._init_arduino_pins()
        self._power = getattr(robot, "power_board", None)

        self._motors = getattr(
            getattr(robot, "motor_board", None), "motors", None
        )
        self._servos = getattr(
            getattr(robot, "servo_board", None), "servos", None
        )

        # Digital outputs (Power Board)
        self._outputs = SR1Outputs(power_board=self._power)

        # Camera registry
        self._cameras: Dict[str, Camera] = {}
        self._detect_cameras()

        # kch, buzzer
        self._kch = getattr(robot, "kch", None)
        self._buzzer = SR1Buzzer(self._power) if self._power else None
        self._kch_wrap = SR1KCH(self._kch) if self._kch else None

    def _init_arduino_pins(self):
        if not self.arduino:
            return
        try:
            from sr.robot3 import INPUT_PULLUP, INPUT, A0, A1, A2
        except Exception:
            return

        pins = getattr(self.arduino, "pins", None)
        if not pins:
            return

        # Bumpers: use pull-ups so switches can go to GND
        for p in (self.PIN_BUMPER_FL, self.PIN_BUMPER_FR, self.PIN_BUMPER_RL, self.PIN_BUMPER_RR):
            pins[p].mode = INPUT_PULLUP

        # Reflectance: analog inputs
        for p in (A0, A1, A2):
            pins[p].mode = INPUT


    # --------------------------------------------------
    # Digital Outputs
    # --------------------------------------------------

    @property
    def outputs(self):
        return self._outputs

    # --------------------------------------------------
    # Camera detection
    # --------------------------------------------------

    def _detect_cameras(self):
        """
        Detect and register available cameras.

        Semantic names are assigned here.
        """
        sr_cam = getattr(self.robot, "camera", None)
        if sr_cam is not None:
            self._cameras["front"] = SRAprilCamera(sr_cam)

        # Future examples:
        # self._cameras["rear"] = USBCamera("/dev/video1")
        # self._cameras["arm"] = CsiCamera(...)

    # --------------------------------------------------
    # Sensors
    # --------------------------------------------------

    # --------------------------------------------------
    # Arduino helpers (SR 2026: robot.arduino.pins[...])
    # --------------------------------------------------

    def _digital_read(self, pin: int) -> bool:
        if not self.arduino:
            return False

        pins = getattr(self.arduino, "pins", None)
        if pins is not None:
            return bool(pins[pin].digital_read())

        # Fallback for older APIs (if ever encountered)
        fn = getattr(self.arduino, "digital_read", None)
        if fn:
            return bool(fn(pin))

        raise AttributeError("Arduino digital read API not found")

    def _analog_read(self, pin) -> float:
        if not self.arduino:
            return 0.0

        pins = getattr(self.arduino, "pins", None)
        if pins is not None:
            v = pins[pin].analog_read()
        else:
            fn = getattr(self.arduino, "analog_read", None)
            if fn:
                v = fn(pin)
            else:
                raise AttributeError("Arduino analog read API not found")

        # SR sometimes returns (value, meta) or similar
        if isinstance(v, (tuple, list)):
            if not v:
                return 0.0
            v = v[0]

        return float(v)

    def bumpers(self) -> Dict[str, bool]:
        if not self.arduino:
            return dict(fl=False, fr=False, rl=False, rr=False)

        return {
            "fl": self._digital_read(self.PIN_BUMPER_FL),
            "fr": self._digital_read(self.PIN_BUMPER_FR),
            "rl": self._digital_read(self.PIN_BUMPER_RL),
            "rr": self._digital_read(self.PIN_BUMPER_RR),
        }

    from sr.robot3 import A0, A1, A2  # put at top of file (inside try if you want)

    def reflectance(self) -> Dict[str, float]:
        if not self.arduino:
            return dict(left=0.0, center=0.0, right=0.0)

        # Prefer SR 2026 pin API
        pins = getattr(self.arduino, "pins", None)
        if pins is None:
            # Fallback for older/alternate Arduino wrappers
            fn = getattr(self.arduino, "analog_read", None)
            if not fn:
                return dict(left=0.0, center=0.0, right=0.0)
            return {
                "left": float(fn(self.PIN_REFLECT_LEFT)),
                "center": float(fn(self.PIN_REFLECT_CENTER)),
                "right": float(fn(self.PIN_REFLECT_RIGHT)),
            }

        def _val(v):
            # SR sometimes returns (value, meta) etc.
            if isinstance(v, (tuple, list)):
                v = v[0] if v else 0.0
            return float(v)

        return {
            "left": _val(pins[A0].analog_read()),
            "center": _val(pins[A1].analog_read()),
            "right": _val(pins[A2].analog_read()),
        }

    def ultrasonics(self) -> Dict[str, Optional[float]]:
        if not self.arduino:
            return dict(front=None, left=None, right=None, back=None)

        measure = getattr(self.arduino, "ultrasound_measure", None)
        if not measure:
            return dict(front=None, left=None, right=None, back=None)

        return {
            "front": measure(self.PIN_US_FRONT_TRIG, self.PIN_US_FRONT_ECHO),
            "left": measure(self.PIN_US_LEFT_TRIG, self.PIN_US_LEFT_ECHO),
            "right": measure(self.PIN_US_RIGHT_TRIG, self.PIN_US_RIGHT_ECHO),
            "back": measure(self.PIN_US_BACK_TRIG, self.PIN_US_BACK_ECHO),
        }

    def sense(self) -> Dict[str, Any]:
        return {
            "bumpers": self.bumpers(),
            "reflectance": self.reflectance(),
            "ultrasonics": self.ultrasonics(),
            "battery": self.battery(),
        }

    # --------------------------------------------------
    # Cameras (public API)
    # --------------------------------------------------

    def cameras(self) -> Dict[str, Camera]:
        """
        Return available cameras keyed by semantic name.

        Example:
            {
                "front": Camera,
                "rear": Camera,
            }
        """
        return dict(self._cameras)

    # --------------------------------------------------
    # Actuators
    # --------------------------------------------------

    @property
    def motors(self):
        return self._motors

    @property
    def servos(self):
        return self._servos

    # --------------------------------------------------
    # Power
    # --------------------------------------------------

    def battery(self) -> Dict[str, Optional[float]]:
        if not self._power:
            return {"voltage": None, "current": None}

        sensor = getattr(self._power, "battery_sensor", None)
        if not sensor:
            return {"voltage": None, "current": None}

        return {
            "voltage": getattr(sensor, "voltage", None),
            "current": getattr(sensor, "current", None),
        }

    # --------------------------------------------------
    # kch, buzzer
    # --------------------------------------------------

    def kch(self):
        return self._kch_wrap

    def buzzer(self):
        return self._buzzer

    def wait_start(self):
        fn = getattr(self.robot, "wait_start", None)
        return fn

    # --------------------------------------------------
    # Sleep
    # --------------------------------------------------

    def sleep(self, secs: float) -> None:
        if hasattr(self.robot, "sleep"):
            self.robot.sleep(secs)
        else:
            time.sleep(secs)

