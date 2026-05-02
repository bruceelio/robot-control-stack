# hw_io/clients/mega_client.py

from __future__ import annotations

import time
from dataclasses import dataclass

import serial


@dataclass
class MegaSerialConfig:
    port: str = "/dev/ttyACM0"
    baud: int = 115200
    timeout: float = 1.0
    open_delay_s: float = 2.0


class MegaSerialClient:
    """
    Production serial client for the Arduino Mega control link.

    This is a direct move of the tested API shape out of tests/ so production
    code can depend on it without importing from the test tree.
    """

    def __init__(self, config: MegaSerialConfig):
        self.config = config
        self.ser: serial.Serial | None = None

    def open(self) -> None:
        self.ser = serial.Serial(
            self.config.port,
            self.config.baud,
            timeout=self.config.timeout,
        )
        time.sleep(self.config.open_delay_s)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def __enter__(self) -> "MegaSerialClient":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def send(self, cmd: str, *, expect_reply: bool = True) -> str:
        if self.ser is None:
            raise RuntimeError("MegaSerialClient is not open")
        self.ser.write((cmd + "\n").encode("utf-8"))
        self.ser.flush()
        if not expect_reply:
            return ""
        return self.ser.readline().decode("utf-8", errors="replace").strip()

    # -------------------------
    # control plane
    # -------------------------

    def hello(self) -> str:
        return self.send("HELLO")

    def mode_auto(self) -> str:
        return self.send("MODE AUTO")

    def mode_teleop(self) -> str:
        return self.send("MODE TELEOP")

    def heartbeat(self, seq: int) -> str:
        return self.send(f"HB {seq}")

    def stop(self) -> str:
        return self.send("STOP")

    # -------------------------
    # link / direct write API
    # -------------------------

    def link_write(self, tx_pin: int, rx_pin: int, channel: str, value: float) -> str:
        value = max(-1.0, min(1.0, float(value)))
        return self.send(f"LINK {tx_pin} {rx_pin} {channel} {value:.3f}")

    def link_18_19(self, channel: str, value: float) -> str:
        return self.link_write(18, 19, channel, value)

    def link_14_15(self, channel: str, value: float) -> str:
        return self.link_write(14, 15, channel, value)

    def servo_write(self, target: str | int, value: float | None = None, *, position: float | None = None) -> str:
        if position is not None:
            position = max(-1.0, min(1.0, float(position)))
            return self.send(f"SERVO {target} WRITE position={position:.4f}")

        if value is None:
            raise ValueError("servo_write requires either value or position")

        value = max(-1.0, min(1.0, float(value)))
        return self.send(f"SERVO_WRITE {target} {value:.3f}")

    def group_write(self, pin1: int, value1: float, pin2: int, value2: float) -> str:
        value1 = max(-1.0, min(1.0, float(value1)))
        value2 = max(-1.0, min(1.0, float(value2)))
        return self.send(f"GROUP_WRITE {pin1} {value1:.3f} {pin2} {value2:.3f}")

    def hbridge_write(self, *, ina: int, inb: int, en_diag: int, pwm: int, value: float) -> str:
        value = max(-1.0, min(1.0, float(value)))
        return self.send(f"HBRIDGE_WRITE {ina} {inb} {en_diag} {pwm} {value:.3f}")

    def motor_write(self, name: str, *, power: float) -> str:
        power = max(-1.0, min(1.0, float(power)))
        return self.send(f"MOTOR {name} WRITE power={power:.4f}")

    # -------------------------
    # read API
    # -------------------------

    def digital_read(self, pin: int) -> bool:
        resp = self.send(f"READ DI {pin}")
        return resp.strip().upper() in {"1", "HIGH", "TRUE", "ON"}

    def analog_read(self, pin_or_name: str | int) -> float | None:
        resp = self.send(f"READ AI {pin_or_name}")
        return self._parse_optional_float(resp)

    def battery_read(self, channel: str = "voltage") -> float | None:
        resp = self.send(f"READ BATTERY {channel}")
        return self._parse_optional_float(resp)

    def limit_read(self, name: str) -> bool:
        resp = self.send(f"READ LIMIT {name}")
        return resp.strip().upper() in {"1", "HIGH", "TRUE", "ON"}

    def quad_read(self, pin_a: int, pin_b: int) -> int | None:
        resp = self.send(f"READ QUAD {pin_a} {pin_b}")
        try:
            return int(resp.strip())
        except (TypeError, ValueError):
            return None

    def range_read(self, trig: int, echo: int) -> float | None:
        resp = self.send(f"READ RANGE {trig} {echo}")
        return self._parse_optional_float(resp)

    @staticmethod
    def _parse_optional_float(resp: str) -> float | None:
        text = (resp or "").strip()
        if text == "" or text.upper() in {"NONE", "NULL", "NAN", "ERR"}:
            return None
        try:
            return float(text)
        except ValueError:
            return None
