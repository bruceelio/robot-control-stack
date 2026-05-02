# primitives/manipulation/shooter.py

import time
from primitives.base import Primitive, PrimitiveStatus


class Shooter(Primitive):
    """
    Maintain shooter wheel speed using encoder velocity feedback.

    Expected inputs:
        signals.encoder["shooter"].velocity  # rev/s if encoder units="rev"

    Expected outputs via lvl2:
        lvl2.SHOOTER_POWER(power)
        or lvl2.SHOOTER_STOP()

    This primitive returns SUCCEEDED once the shooter is at speed and settled
    """

    def __init__(
        self,
        *,
        target_rpm: float,
        tolerance_rpm: float = 150.0,
        settle_time: float = 0.25,
        timeout_s: float | None = None,
        kP: float = 0.00025,
        kF: float = 0.00020,
        min_power: float = 0.0,
        max_power: float = 1.0,
    ):
        super().__init__()
        self.target_rpm = float(target_rpm)
        self.tolerance_rpm = float(tolerance_rpm)
        self.settle_time = float(settle_time)
        self.timeout_s = timeout_s

        self.kP = float(kP)
        self.kF = float(kF)
        self.min_power = float(min_power)
        self.max_power = float(max_power)

        self._start_time = None
        self._at_speed_since = None
        self.measured_rpm = None
        self.power = 0.0

    def start(self, *, lvl2, **_):
        print(f"[Shooter] start target_rpm={self.target_rpm}")
        self._start_time = time.time()
        self._at_speed_since = None
        self.measured_rpm = None
        self.power = 0.0

    def update(self, *, lvl2, signals, **_):
        if self._start_time is None:
            return PrimitiveStatus.FAILED

        if self.timeout_s is not None:
            if time.time() - self._start_time > self.timeout_s:
                self._stop(lvl2)
                print("[Shooter] failed: timeout")
                return PrimitiveStatus.FAILED

        enc = signals.encoder.get("shooter")
        if enc is None or not enc.valid or enc.velocity is None:
            self._stop(lvl2)
            print("[Shooter] failed: invalid encoder signal")
            return PrimitiveStatus.FAILED

        self.measured_rpm = enc.velocity * 60.0
        error_rpm = self.target_rpm - self.measured_rpm

        power = (self.kF * self.target_rpm) + (self.kP * error_rpm)
        power = max(self.min_power, min(self.max_power, power))
        self.power = power

        if hasattr(lvl2, "SHOOTER_POWER"):
            lvl2.SHOOTER_POWER(power)
        else:
            print("[Shooter] No shooter motor available on this robot")
            return PrimitiveStatus.FAILED

        if abs(error_rpm) <= self.tolerance_rpm:
            if self._at_speed_since is None:
                self._at_speed_since = time.time()

            if time.time() - self._at_speed_since >= self.settle_time:
                return PrimitiveStatus.SUCCEEDED
        else:
            self._at_speed_since = None

        return PrimitiveStatus.RUNNING

    def stop(self, *, lvl2, **_):
        self._stop(lvl2)

    def _stop(self, lvl2):
        if hasattr(lvl2, "SHOOTER_STOP"):
            lvl2.SHOOTER_STOP()
        elif hasattr(lvl2, "SHOOTER_POWER"):
            lvl2.SHOOTER_POWER(0.0)