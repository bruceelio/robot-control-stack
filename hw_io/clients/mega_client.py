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

    def servo_write(self, target: str | int, value: float | None = None, *, position: float | None = None) -> str:
        if position is not None:
            position = max(-1.0, min(1.0, float(position)))
            return self.send(f"SERVO {target} WRITE position={position:.4f}")

        if value is None:
            raise ValueError("servo_write requires either value or position")

        value = max(-1.0, min(1.0, float(value)))
        return self.send(f"SERVO_WRITE {target} {value:.3f}")

    def motor_write(self, name: str, *, power: float) -> str:
        power = max(-1.0, min(1.0, float(power)))
        return self.send(f"MOTOR {name} WRITE power={power:.4f}")

    def led_write(self, name: str, *, brightness: float) -> str:
        brightness = max(0.0, min(1.0, float(brightness)))
        return self.send(f"LED {name} WRITE brightness={brightness:.4f}")

    def audio_play(self, name: str, **kwargs) -> str:
        if name == "df_player":
            track = int(kwargs["track"])
            return self.send(f"AUDIO df_player PLAY track={track}")

        if name == "piezo":
            tone = int(kwargs["tone"])
            duration_ms = int(kwargs["duration_ms"])
            return self.send(
                f"AUDIO piezo PLAY tone={tone} duration_ms={duration_ms}"
            )

        raise ValueError(f"unsupported audio device: {name}")

    # -------------------------
    # read API
    # -------------------------

    def bumper_read(self, name: str) -> str:
        return self.send(f"BUMPER {name} READ")

    def current_read(self, name: str) -> str:
        return self.send(f"CURRENT {name} READ")

    def encoder_read(self, name: str) -> str:
        return self.send(f"ENCODER {name} READ")

    def imu_read(self, name: str) -> str:
        return self.send(f"IMU {name} READ")

    def otos_read(self, name: str) -> str:
        return self.send(f"OTOS {name} READ")

    def reflectance_read(self, name: str) -> str:
        return self.send(f"REFLECTANCE {name} READ")

    def ultrasonic_read(self, name: str) -> str:
        return self.send(f"ULTRASONIC {name} READ")

    def limit_read(self, name: str) -> str:
        return self.send(f"LIMIT {name} READ")

    def voltage_read(self, name: str) -> str:
        return self.send(f"VOLTAGE {name} READ")

    @staticmethod
    def _parse_optional_float(resp: str) -> float | None:
        text = (resp or "").strip()
        if text == "" or text.upper() in {"NONE", "NULL", "NAN", "ERR"}:
            return None
        try:
            return float(text)
        except ValueError:
            return None
