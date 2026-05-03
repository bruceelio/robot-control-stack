# hw_io/encoder_manager.py
from __future__ import annotations

from importlib import import_module
from types import SimpleNamespace
from typing import Any

from hw_io.encoder import Encoder


class EncoderManager:
    """
    Robot-agnostic manager for configured encoders.

    Build-time:
        config.ENCODERS maps signal names to encoder config modules.

        Example:
            {
                "deadwheel_parallel": "gobilda_4bar_odometry_pod_32mm",
                "shooter": "gobilda_yellowjacket_6000rpm",
            }

    Runtime:
        io.encoder[name] -> Encoder.update(...) -> signals.encoder[name]
    """

    def __init__(self, encoder_assignments: dict[str, str] | None) -> None:
        self.encoders: dict[str, Encoder] = {}

        if not encoder_assignments:
            return

        for name, config_name in encoder_assignments.items():
            config = self._resolve_encoder_config(
                name=name,
                config_name=config_name,
            )
            self.encoders[name] = Encoder(config)

    def update(self, *, io: Any, signals: Any) -> None:
        if not hasattr(signals, "encoder") or signals.encoder is None:
            signals.encoder = {}

        try:
            io_encoders = io.encoder
        except NotImplementedError:
            return

        if io_encoders is None:
            return

        for name, encoder in self.encoders.items():
            if name not in io_encoders:
                continue

            raw = io_encoders[name]

            signals.encoder[name] = encoder.update(
                raw_count=raw.count,
                timestamp_ms=raw.timestamp_ms,
                source_valid=raw.valid,
                valid_flags=raw.valid_flags,
            )

    def reset(self, name: str | None = None) -> None:
        """
        Reset one encoder, or all encoders if name is None.
        """
        if name is None:
            for encoder in self.encoders.values():
                encoder.reset()
            return

        if name not in self.encoders:
            raise KeyError(f"No configured encoder named {name!r}")

        self.encoders[name].reset()

    def has_encoder(self, name: str) -> bool:
        return name in self.encoders

    @staticmethod
    def _resolve_encoder_config(*, name: str, config_name: str) -> dict[str, Any]:
        module = import_module(f"config.encoders.{config_name}")

        return {
            "role": name,
            "model": config_name,
            "encoder_type": module.ENCODER_TYPE,
            "counts_per_rev": module.COUNTS_PER_REV,
            "units": module.UNITS,
            "units_per_rev": module.UNITS_PER_REV,
            "invert": getattr(module, "DEFAULT_INVERT", False),
            "zero_on_start": getattr(module, "ZERO_ON_START", True),
            "max_delta_count": getattr(module, "MAX_DELTA_COUNT", None),
        }


def make_signals() -> SimpleNamespace:
    """
    Create the shared processed-signal container.

    Current shape:
        signals.encoder[name] = EncoderSignal
    """
    return SimpleNamespace(
        encoder={},
    )