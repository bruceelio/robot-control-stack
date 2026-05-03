# hw_io/encoder.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class EncoderSignal:
    role: str
    model: str

    # position
    position_counts: int
    position: float

    # delta
    delta_counts: int
    delta: float

    # timing
    dt: float | None
    timestamp: float  # seconds

    # velocity
    velocity: float | None

    # validity
    valid: bool
    source_valid: bool
    valid_flags: int

    # state
    initialized: bool
    units: str


class Encoder:
    def __init__(self, config: dict[str, Any]) -> None:
        self.role = str(config["role"])
        self.model = str(config["model"])
        self.encoder_type = str(config["encoder_type"])

        self.counts_per_rev = float(config["counts_per_rev"])
        self.units = str(config["units"])
        self.units_per_rev = float(config["units_per_rev"])

        self.invert = bool(config["invert"])
        self.zero_on_start = bool(config["zero_on_start"])

        self.max_delta_count = config.get("max_delta_count")

        if self.counts_per_rev <= 0:
            raise ValueError(
                f"Encoder '{self.role}' invalid counts_per_rev={self.counts_per_rev}"
            )

        self._units_per_count = self.units_per_rev / self.counts_per_rev

        self._zero_count: int | None = None
        self._prev_count: int | None = None
        self._prev_timestamp: float | None = None

        self._initialized = False

    def reset(self) -> None:
        self._zero_count = None
        self._prev_count = None
        self._prev_timestamp = None
        self._initialized = False

    def update(
        self,
        raw_count: int,
        timestamp_ms: int,
        source_valid: bool,
        valid_flags: int,
    ) -> EncoderSignal:
        timestamp = float(timestamp_ms) * 1e-3
        raw_count = int(raw_count)

        # initialize zero
        if self._zero_count is None:
            self._zero_count = raw_count if self.zero_on_start else 0

        # apply zero + invert
        count = raw_count - self._zero_count
        if self.invert:
            count = -count

        # first sample
        if self._prev_count is None or self._prev_timestamp is None:
            self._prev_count = count
            self._prev_timestamp = timestamp
            self._initialized = True

            return EncoderSignal(
                role=self.role,
                model=self.model,
                position_counts=count,
                position=count * self._units_per_count,
                delta_counts=0,
                delta=0.0,
                dt=None,
                timestamp=timestamp,
                velocity=None,
                valid=source_valid,
                source_valid=source_valid,
                valid_flags=valid_flags,
                initialized=True,
                units=self.units,
            )

        # compute delta
        dt = timestamp - self._prev_timestamp
        delta_counts = count - self._prev_count

        # validity checks
        valid = True

        if not source_valid:
            valid = False

        if dt <= 0.0:
            valid = False

        if self.max_delta_count is not None and abs(delta_counts) > int(self.max_delta_count):
            valid = False

        # compute values
        delta = delta_counts * self._units_per_count
        position = count * self._units_per_count

        velocity = None
        if valid and dt > 0.0:
            velocity = delta / dt

        # 🔑 IMPORTANT DESIGN CHOICE:
        # only advance state if valid
        if valid:
            self._prev_count = count
            self._prev_timestamp = timestamp

        return EncoderSignal(
            role=self.role,
            model=self.model,
            position_counts=count,
            position=position,
            delta_counts=delta_counts,
            delta=delta,
            dt=dt if dt > 0.0 else None,
            timestamp=timestamp,
            velocity=velocity,
            valid=valid,
            source_valid=source_valid,
            valid_flags=valid_flags,
            initialized=True,
            units=self.units,
        )