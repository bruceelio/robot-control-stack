# hw_io/clients/uno_client.py

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class UnoSerialConfig:
    port: str = "/dev/ttyACM1"
    baud: int = 115200
    timeout: float = 1.0
    open_delay_s: float = 2.0
    enabled: bool = False


class StubUnoSerialClient:
    """
    Minimal placeholder until the Uno firmware and protocol mature.

    The current stack can operate with safe defaults as long as the Uno-backed
    sensors appear to exist.
    """

    def open(self) -> None:
        return None

    def close(self) -> None:
        return None

    def __enter__(self) -> "StubUnoSerialClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        return None

    def digital_read(self, pin: int) -> bool:
        return False

    def analog_read(self, pin_or_name: str | int) -> float:
        return 0.0

    def range_read(self, trig: int, echo: int) -> float | None:
        return None


class UnoSerialClient(StubUnoSerialClient):
    """
    Intentionally minimal for now.

    Mirror the Mega constructor/config shape so BobBotIO can switch from the
    stub to a real implementation later with minimal churn.
    """

    def __init__(self, config: UnoSerialConfig):
        self.config = config

    def open(self) -> None:
        raise NotImplementedError(
            "UnoSerialClient is not implemented yet. Use StubUnoSerialClient or "
            "leave uno_enabled=False."
        )
