# hw_io/encoder.py

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class EncoderReading:
    """
    Processed reading from a single encoder update.

    Notes:
    - count is the zeroed, sign-corrected absolute count
    - delta_count is the change since the previous update
    - position_units and delta_units are scaled using the encoder config
    - rate_units_s is None on first update or when dt_s <= 0
    - valid=False means the reading should not be trusted for derived use
    """
    role: str
    model: str
    count: int
    delta_count: int
    position_units: float
    delta_units: float
    dt_s: float | None
    rate_units_s: float | None
    units: str
    timestamp_s: float
    valid: bool
    initialized: bool


class Encoder:
    """
    Generic encoder processor.

    This class sits above io.* and below subsystem logic such as localisation
    or shooter control.

    Expected config fields:
        role: str
        model: str
        encoder_type: str
        counts_per_rev: int
        units: str
        units_per_rev: float
        invert: bool
        zero_on_start: bool

    Optional config fields:
        max_delta_count: int
    """

    def __init__(self, config: dict[str, Any]) -> None:
        self.role = str(config["role"])
        self.model = str(config["model"])
        self.encoder_type = str(config["encoder_type"])
        self.counts_per_rev = int(config["counts_per_rev"])
        self.units = str(config["units"])
        self.units_per_rev = float(config["units_per_rev"])
        self.invert = bool(config["invert"])
        self.zero_on_start = bool(config["zero_on_start"])
        self.max_delta_count = config.get("max_delta_count")

        if self.counts_per_rev <= 0:
            raise ValueError(
                f"Encoder '{self.role}' has invalid counts_per_rev={self.counts_per_rev}"
            )

        self._units_per_count = self.units_per_rev / self.counts_per_rev

        self._zero_count: int | None = None
        self._prev_count: int | None = None
        self._prev_timestamp_s: float | None = None
        self._initialized = False

    def reset(self) -> None:
        """
        Forget history and force the next update() call to reinitialize state.
        """
        self._zero_count = None
        self._prev_count = None
        self._prev_timestamp_s = None
        self._initialized = False

    @property
    def initialized(self) -> bool:
        return self._initialized

    def update(self, raw_count: int, timestamp_s: float) -> EncoderReading:
        """
        Consume a raw encoder count and return a processed reading.

        The first update initializes the encoder and returns:
        - delta_count = 0
        - delta_units = 0.0
        - rate_units_s = None
        - valid = True

        If zero_on_start is enabled, the first raw count becomes the zero reference.
        """
        raw_count = int(raw_count)
        timestamp_s = float(timestamp_s)

        if self._zero_count is None:
            self._zero_count = raw_count if self.zero_on_start else 0

        count = raw_count - self._zero_count
        if self.invert:
            count = -count

        if self._prev_count is None or self._prev_timestamp_s is None:
            self._prev_count = count
            self._prev_timestamp_s = timestamp_s
            self._initialized = True

            return EncoderReading(
                role=self.role,
                model=self.model,
                count=count,
                delta_count=0,
                position_units=count * self._units_per_count,
                delta_units=0.0,
                dt_s=None,
                rate_units_s=None,
                units=self.units,
                timestamp_s=timestamp_s,
                valid=True,
                initialized=True,
            )

        delta_count = count - self._prev_count
        dt_s = timestamp_s - self._prev_timestamp_s

        valid = dt_s > 0.0

        if self.max_delta_count is not None and abs(delta_count) > int(self.max_delta_count):
            valid = False

        position_units = count * self._units_per_count
        delta_units = delta_count * self._units_per_count
        rate_units_s = (delta_units / dt_s) if valid and dt_s > 0.0 else None

        self._prev_count = count
        self._prev_timestamp_s = timestamp_s

        return EncoderReading(
            role=self.role,
            model=self.model,
            count=count,
            delta_count=delta_count,
            position_units=position_units,
            delta_units=delta_units,
            dt_s=dt_s if dt_s > 0.0 else None,
            rate_units_s=rate_units_s,
            units=self.units,
            timestamp_s=timestamp_s,
            valid=valid,
            initialized=True,
        )