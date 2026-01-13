"""
level2_canonical.py

Level 2 canonical interface for robot actions.

This layer exposes high-level, robot-agnostic commands (DRIVE, ROTATE, etc.)
and maps them to:
- SR Robot3 APIs when available
- Level 1 canonical I/O as a fallback

Level 2 merges multiple Level 1 I/O operations into a single semantic action.
"""

import time
from hal.pinmap import canonical_to_pin
from hal.hardware import is_sr


class Level2:
    MAX_POWER = 0.8  # maximum allowed motor power

    def __init__(self, robot=None):
        """
        robot: SR Robot3 instance, or None (non-SR / fallback mode)
        """
        self.robot = robot
        self.has_sr = robot is not None

    # -----------------------------
    # Utility methods
    # -----------------------------

    def _clip(self, power):
        """Clip motor power to safe range."""
        return max(-self.MAX_POWER, min(self.MAX_POWER, power))

    def _stop_motors(self):
        """Safely stop all motors if SR Robot present."""
        if self.has_sr:
            motors = self.robot.motor_board.motors
            for i in range(2):
                motors[i].power = 0

    # -----------------------------
    # DRIVE / ROTATE
    # -----------------------------

    def DRIVE(self, left_power: float, right_power: float, duration: float = None):
        """Drive robot: positive = forward, negative = backward"""
        left_power = self._clip(left_power)
        right_power = self._clip(right_power)
        print(f"[Level2] DRIVE L={left_power} R={right_power} duration={duration}")

        if self.has_sr:
            try:
                motors = self.robot.motor_board.motors
                motors[0].power = left_power
                motors[1].power = right_power
                if duration is not None:
                    self.robot.sleep(duration)
            finally:
                self._stop_motors()
        else:
            left_pin = canonical_to_pin.get("MOT_LEFT")
            right_pin = canonical_to_pin.get("MOT_RIGHT")
            try:
                if left_pin:
                    left_pin.write(left_power)
                if right_pin:
                    right_pin.write(right_power)
                if duration:
                    time.sleep(duration)
            finally:
                if left_pin:
                    left_pin.write(0)
                if right_pin:
                    right_pin.write(0)

    def ROTATE(self, angle: float):
        """Rotate robot in degrees: positive = clockwise"""
        print(f"[Level2] ROTATE {angle}°")
        power = 0.5 if angle > 0 else -0.5
        duration = abs(angle) / 90 * 0.46  # from calibration

        if self.has_sr:
            try:
                motors = self.robot.motor_board.motors
                motors[0].power = power
                motors[1].power = -power
                self.robot.sleep(duration)
            finally:
                self._stop_motors()
        else:
            time.sleep(duration)

    # -----------------------------
    # INDIVIDUAL MOTORS
    # -----------------------------

    def MOTOR_LEFT(self, power: float):
        power = self._clip(power)
        print(f"[Level2] MOTOR_LEFT power={power}")
        if self.has_sr:
            try:
                motors = self.robot.motor_board.motors
                motors[0].power = power
            finally:
                self._stop_motors()
        else:
            pin = canonical_to_pin.get("MOT_LEFT")
            if pin:
                pin.write(power)

    def MOTOR_RIGHT(self, power: float):
        power = self._clip(power)
        print(f"[Level2] MOTOR_RIGHT power={power}")
        if self.has_sr:
            try:
                motors = self.robot.motor_board.motors
                motors[1].power = power
            finally:
                self._stop_motors()
        else:
            pin = canonical_to_pin.get("MOT_RIGHT")
            if pin:
                pin.write(power)

    # -----------------------------
    # LEDs
    # -----------------------------

    def LED_ON(self, index: int | None = None):
        print(f"[Level2] LED_ON index={index}")
        if self.has_sr:
            if index is None:
                for i in range(3):
                    self.robot.kch.set_led(i, True)
            else:
                self.robot.kch.set_led(index, True)
        else:
            pin_name = f"LED_{index}" if index is not None else "LED_ALL"
            pin = canonical_to_pin.get(pin_name)
            if pin:
                pin.write(1)

    def LED_OFF(self, index: int | None = None):
        print(f"[Level2] LED_OFF index={index}")
        if self.has_sr:
            if index is None:
                for i in range(3):
                    self.robot.kch.set_led(i, False)
            else:
                self.robot.kch.set_led(index, False)
        else:
            pin_name = f"LED_{index}" if index is not None else "LED_ALL"
            pin = canonical_to_pin.get(pin_name)
            if pin:
                pin.write(0)

    # -----------------------------
    # BUZZER / PIEZO
    # -----------------------------

    def BUZZER_ON(self, note=None, duration: float = 0.5):
        print(f"[Level2] BUZZER_ON note={note} duration={duration}")
        if self.has_sr:
            from sr.robot3 import Note
            note = note if note is not None else Note.A6
            self.robot.power_board.piezo.buzz(note, duration)
        else:
            pin = canonical_to_pin.get("BUZZER")
            if pin:
                pin.write(1)
                time.sleep(duration)
                pin.write(0)

    def BUZZER_OFF(self):
        print("[Level2] BUZZER_OFF")
        if self.has_sr:
            self.robot.power_board.piezo.buzz(0, 0)
        else:
            pin = canonical_to_pin.get("BUZZER")
            if pin:
                pin.write(0)

    # -----------------------------
    # SENSORS (ARDUINO)
    # -----------------------------

    def SENSOR_READ_DI(self, pin: int):
        if self.has_sr:
            assert isinstance(pin, int), "SR Arduino pins must be numeric"
            value = self.robot.arduino.digital_read(pin)
        else:
            p = canonical_to_pin.get(pin)
            value = p.read() if p else None
        print(f"[Level2] SENSOR_READ_DI pin={pin} value={value}")
        return value

    def SENSOR_READ_AI(self, pin: int):
        if self.has_sr:
            assert isinstance(pin, int), "SR Arduino pins must be numeric"
            value = self.robot.arduino.analog_read(pin)
        else:
            p = canonical_to_pin.get(pin)
            value = p.read() if p else None
        print(f"[Level2] SENSOR_READ_AI pin={pin} value={value}")
        return value

    # -----------------------------
    # GRIPPER
    # -----------------------------

    # Vacuum

    def VACUUM_ON(self):
        print("[Level2] VACUUM_ON")
        if is_sr():
            from sr.robot3 import OUT_H0
            self.robot.power_board.outputs[OUT_H0].is_enabled = True
        else:
            pin = canonical_to_pin.get("DO_GRIPPER_SOLENOID")
            if pin:
                pin.write(1)

    def VACUUM_OFF(self):
        print("[Level2] VACUUM_OFF")
        if is_sr():
            from sr.robot3 import OUT_H0
            self.robot.power_board.outputs[OUT_H0].is_enabled = False
        else:
            pin = canonical_to_pin.get("DO_GRIPPER_SOLENOID")
            if pin:
                pin.write(0)

    # Lift

    def LIFT_DOWN(self):
        print("[Level2] LIFT_DOWN")
        if is_sr():
            self.robot.servo_board.servos[0].position = -1
            self.robot.sleep(0.4)

    def LIFT_UP(self):
        print("[Level2] LIFT_UP")
        if is_sr():
            self.robot.servo_board.servos[0].position = 1
            self.robot.sleep(0.4)

    def LIFT_DISABLE(self):
        print("[Level2] LIFT_DISABLE")
        if is_sr():
            self.robot.servo_board.servos[0].position = None

    # -----------------------------
    # CAMERA
    # -----------------------------

    def CAMERA_SEE(self):
        print("[Level2] CAMERA_SEE")
        if self.has_sr and self.robot.camera:
            return self.robot.camera.see()
        return []

    # -----------------------------
    # TIME / SLEEP
    # -----------------------------

    def SLEEP(self, secs: float):
        print(f"[Level2] SLEEP {secs}s")
        if self.has_sr:
            self.robot.sleep(secs)
        else:
            time.sleep(secs)
