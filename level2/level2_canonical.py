# level2/level2_canonical.py

"""
level2_canonical.py

Level 2 canonical interface for robot actions.

This layer exposes high-level, robot-agnostic commands (DRIVE, ROTATE, etc.)
and maps them to the canonical hardware IOMap implementation.

Design rule:
- Level2 never talks to SR Robot directly.
- Level2 never talks to raw pinmaps.
- Level2 only talks to IOMap.

Result:
- One abstraction for real + sim + non-SR robots.
"""

from __future__ import annotations

import time
from typing import Optional

from hw_io.base import IOMap


class Level2:
    def __init__(self, io: IOMap, *, max_power: float):
        """
        io: canonical IOMap implementation (SR-backed or direct-hardware-backed)
        max_power: maximum allowed motor power (from Config)
        """
        self.io = io
        self.max_power = max_power

    # -----------------------------
    # Utility
    # -----------------------------

    def _clip(self, power: float) -> float:
        """Clip motor power to safe range."""
        return max(-self.max_power, min(self.max_power, power))

    def _stop_motors(self) -> None:
        """Safely stop motors if available."""
        motors = getattr(self.io, "motors", None)
        if motors is None:
            return
        try:
            for i in range(2):
                motors[i].power = 0
        except Exception:
            # Defensive: different motor adapters may not support indexing etc.
            pass

    # -----------------------------
    # DRIVE / ROTATE
    # -----------------------------

    def DRIVE(self, left_power: float, right_power: float, duration: Optional[float] = None):
        """Drive robot: positive = forward, negative = backward."""
        left_power = self._clip(left_power)
        right_power = self._clip(right_power)
        print(f"[Level2] DRIVE L={left_power} R={right_power} duration={duration}")

        motors = getattr(self.io, "motors", None)
        if motors is None:
            raise RuntimeError("Level2.DRIVE: io.motors is not available")

        try:
            motors[0].power = left_power
            motors[1].power = right_power
            if duration is not None:
                self.SLEEP(duration)
        finally:
            self._stop_motors()

    def ROTATE(self, angle_deg: float):
        """
        Semantic rotation request.

        Angle is in degrees.
        Positive = clockwise.
        Execution is delegated to the motion backend.
        """
        print(f"[Level2] ROTATE request angle={angle_deg}°")

    # -----------------------------
    # INDIVIDUAL MOTORS
    # -----------------------------

    def MOTOR_RIGHT(self, power: float):
        power = self._clip(power)
        print(f"[Level2] MOTOR_RIGHT power={power}")

        motors = getattr(self.io, "motors", None)
        if motors is None:
            raise RuntimeError("Level2.MOTOR_RIGHT: io.motors is not available")

        try:
            motors[1].power = power
        finally:
            self._stop_motors()

    def MOTOR_LEFT(self, power: float):
        power = self._clip(power)
        print(f"[Level2] MOTOR_LEFT power={power}")

        motors = getattr(self.io, "motors", None)
        if motors is None:
            raise RuntimeError("Level2.MOTOR_LEFT: io.motors is not available")

        try:
            motors[0].power = power
        finally:
            self._stop_motors()

    # -----------------------------
    # LEDs (optional)
    # -----------------------------
    # NOTE: Your IOMap currently doesn't expose LEDs.
    # If you later add io.leds or io.kch, wire it here.
    # For now, these methods are stubs to preserve your canonical API.

    def LED_ON(self, index: int | None = None):
        print(f"[Level2] LED_ON index={index} (stub — no io.leds exposed)")

    def LED_OFF(self, index: int | None = None):
        print(f"[Level2] LED_OFF index={index} (stub — no io.leds exposed)")

    # -----------------------------
    # BUZZER / PIEZO (optional)
    # -----------------------------

    def BUZZ(self, tone=440, duration: float = 0.25, *, blocking: bool = False):
        """
        Canonical buzzer call.
        - tone: Note enum or frequency in Hz
        - duration: seconds
        - blocking: wait until complete
        """
        buz = getattr(self.io, "buzzer", None)
        if buz is None:
            raise RuntimeError("Level2.BUZZ: io.buzzer not available")

        buz_obj = buz() if callable(buz) else buz
        if buz_obj is None:
            raise RuntimeError("Level2.BUZZ: io.buzzer returned None")

        buz_obj.buzz(tone, duration, blocking=blocking)

    # Backwards-compatible names (if older code calls BUZZER_ON/OFF)
    def BUZZER_ON(self, note=None, duration: float = 0.5):
        tone = 440 if note is None else note
        self.BUZZ(tone, duration, blocking=False)

    def BUZZER_OFF(self):
        buz = getattr(self.io, "buzzer", None)
        if buz is None:
            return
        buz_obj = buz() if callable(buz) else buz
        if buz_obj is None:
            return
        buz_obj.off()

    @property
    def patterns(self):
        """
        Semantic buzzer patterns (robot-agnostic).
        Usage:
          lvl2.patterns.cue(BuzzerCue.START)
          lvl2.patterns.rickroll_intro()
        """
        from hw_io.buzzer_patterns import BuzzerPatterns
        return BuzzerPatterns(self)


    # -----------------------------
    # SENSORS
    # -----------------------------
    # NOTE: Level2 shouldn't really do raw sensor reads anymore
    # because you already have io.bumpers/reflectance/ultrasonics.
    # But we keep these for legacy/debug callers.

    def SENSE(self):
        """Unified snapshot (pass-through)."""
        return self.io.sense()

    # -----------------------------
    # GRIPPER / LIFT (servo-based)
    # -----------------------------

    def LIFT_DOWN(self):
        print("[Level2] LIFT_DOWN")
        servos = getattr(self.io, "servos", None)
        if servos is None:
            print("[Level2] LIFT_DOWN: no servos available")
            return

        # Assumption: lift is servo 0, SR-style -1..+1
        try:
            servos[0].position = -1
            self.SLEEP(1.0)
        except Exception as e:
            print("[Level2] LIFT_DOWN failed:", e)

    def LIFT_MIDDLE(self):
        print("[Level2] LIFT_MIDDLE")
        servos = getattr(self.io, "servos", None)
        if servos is None:
            print("[Level2] LIFT_MIDDLE: no servos available")
            return

        try:
            servos[0].position = 0
            self.SLEEP(1.0)
        except Exception as e:
            print("[Level2] LIFT_MIDDLE failed:", e)

    def LIFT_UP(self):
        print("[Level2] LIFT_UP")
        servos = getattr(self.io, "servos", None)
        if servos is None:
            print("[Level2] LIFT_UP: no servos available")
            return

        try:
            servos[0].position = 1

            t_end = time.time() + 1.0
            while time.time() < t_end:
                servos[0].position = 1  # keep reasserting
                self.io.sleep(0.05)
        except Exception as e:
            print("[Level2] LIFT_UP failed:", e)

    def LIFT_DISABLE(self):
        print("[Level2] LIFT_DISABLE")
        servos = getattr(self.io, "servos", None)
        if servos is None:
            return
        try:
            servos[0].position = None
        except Exception as e:
            print("[Level2] LIFT_DISABLE failed:", e)

    def VACUUM_ON(self):
        print("[Level2] VACUUM_ON")
        outs = getattr(self.io, "outputs", None)
        if outs is None:
            print("[Level2] VACUUM_ON: no outputs available")
            return
        outs.set("VACUUM", True)

    def VACUUM_OFF(self):
        print("[Level2] VACUUM_OFF")
        outs = getattr(self.io, "outputs", None)
        if outs is None:
            print("[Level2] VACUUM_OFF: no outputs available")
            return
        outs.set("VACUUM", False)

    def GRAB(self):
        print("[Level2] GRAB")
        servos = getattr(self.io, "servos", None)
        if servos is None:
            print("[Level2] GRAB: no servos available")
            return
        try:
            servos[1].position = -0.38  # or 0.0, depending on your open/closed convention
            self.SLEEP(1.0)
        except Exception as e:
            print("[Level2] GRAB failed:", e)

    def RELEASE(self):
        print("[Level2] RELEASE")
        servos = getattr(self.io, "servos", None)
        if servos is None:
            print("[Level2] RELEASE: no servos available")
            return
        try:
            servos[1].position = 1.0  # opposite of GRAB
            self.SLEEP(1.0)
        except Exception as e:
            print("[Level2] RELEASE failed:", e)

    # -----------------------------
    # CAMERA (optional convenience)
    # -----------------------------

    def CAMERA_SEE(self, camera: str = "front"):
        print(f"[Level2] CAMERA_SEE camera={camera}")
        cams = self.io.cameras()
        if camera not in cams:
            return []
        return cams[camera].see()

    # -----------------------------
    # TIME / SLEEP
    # -----------------------------

    def SLEEP(self, secs: float):
        print(f"[Level2] SLEEP {secs}s")
        self.io.sleep(secs)

    # ---------- Optional capabilities ----------

    @property
    def leds(self):
        """
        Optional LED interface.
        Suggested shape:
          - .set(index: int, on: bool) -> None
          - .set_all(on: bool) -> None
        """
        return None

    @property
    def buzzer(self):
        """
        Optional buzzer/piezo interface.
        Suggested shape:
          - .buzz(note: Any, duration: float) -> None
          - .off() -> None
        """
        return None

    @property
    def outputs(self):
        """
        Optional digital outputs (eg vacuum solenoid).
        Suggested shape:
          - mapping/dict-like: outputs[name].is_enabled = True
          - or methods: set(name, bool)
        """
        return None