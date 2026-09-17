# motion_backends/encoder.py

from __future__ import annotations

import time

from importlib import import_module

from motion_backends.motor_output_conditioner import (
    MotorOutputConditioner,
    MotorPowerCommand,
)


class EncoderMotionBackend:
    """
    Encoder-controlled position motion backend.

    Uses:
        - resolved Config
        - resolved Calibration
        - cumulative drive encoder counts from Level2 / IOMap

    Provides:
        - drive(distance_mm)
        - rotate(angle_deg)

    The Arduino owns quadrature acquisition and cumulative counting.
    This backend consumes those counts to control robot motion.
    """

    def __init__(
        self,
        lvl2,
        config,
        calibration,
    ):
        self.lvl2 = lvl2
        self.cfg = config
        self.cal = calibration

        self.output_conditioner = MotorOutputConditioner(
            config=config,
        )

        self.left_encoder = self._resolve_encoder_profile(
            "drive_front_left"
        )

        self.right_encoder = self._resolve_encoder_profile(
            "drive_front_right"
        )

    # --------------------------------------------------
    # Encoder configuration
    # --------------------------------------------------

    def _resolve_encoder_profile(self, name: str):
        try:
            profile_name = self.cfg.encoders[name]
        except KeyError as exc:
            raise RuntimeError(
                f"No encoder profile configured for {name!r}"
            ) from exc

        return import_module(
            f"config.encoders.{profile_name}"
        )

    # --------------------------------------------------
    # Encoder input
    # --------------------------------------------------

    def _read_count(self, name: str) -> int:
        encoder = self.lvl2.io.encoder[name]
        snapshot = encoder.read()

        if not snapshot.get("valid"):
            raise RuntimeError(
                f"Encoder {name!r} is not valid "
                f"(valid_flags={snapshot.get('valid_flags')})"
            )

        raw_count = snapshot.get("count")

        if raw_count is None:
            raise RuntimeError(
                f"Encoder {name!r} returned no count"
            )

        try:
            sign = int(self.cfg.encoder_sign[name])
        except KeyError as exc:
            raise RuntimeError(
                f"No encoder sign configured for {name!r}"
            ) from exc

        if sign not in (-1, 1):
            raise RuntimeError(
                f"Invalid encoder sign for {name!r}: {sign}"
            )

        return sign * int(raw_count)

    def _read_drive_counts(self) -> tuple[int, int]:
        return (
            self._read_count("drive_front_left"),
            self._read_count("drive_front_right"),
        )

    def _counts_to_mm(
            self,
            name: str,
            counts: int | float,
    ) -> float:
        encoder_profile = self._resolve_encoder_profile(name)

        try:
            wheel_diameter_mm = float(
                self.cfg.encoder_wheel_diameter_mm[name]
            )
        except KeyError as exc:
            raise RuntimeError(
                f"No encoder wheel diameter configured for {name!r}"
            ) from exc

        counts_per_rev = float(
            encoder_profile.COUNTS_PER_REV
        )

        if counts_per_rev <= 0:
            raise RuntimeError(
                f"Invalid COUNTS_PER_REV for {name!r}: "
                f"{counts_per_rev}"
            )

        if wheel_diameter_mm <= 0:
            raise RuntimeError(
                f"Invalid wheel diameter for {name!r}: "
                f"{wheel_diameter_mm}"
            )

        wheel_circumference_mm = (
                3.141592653589793 * wheel_diameter_mm
        )

        return (
                float(counts)
                * wheel_circumference_mm
                / counts_per_rev
        )

    def _drive_progress_mm(
            self,
            start_left: int,
            start_right: int,
    ) -> tuple[float, float, float]:
        current_left, current_right = self._read_drive_counts()

        left_counts = current_left - start_left
        right_counts = current_right - start_right

        left_mm = self._counts_to_mm(
            "drive_front_left",
            left_counts,
        )

        right_mm = self._counts_to_mm(
            "drive_front_right",
            right_counts,
        )

        average_mm = (left_mm + right_mm) / 2.0

        return left_mm, right_mm, average_mm

    # --------------------------------------------------
    # Motor output
    # --------------------------------------------------

    def _set_power(
        self,
        left: float,
        right: float,
    ):
        raw_command = MotorPowerCommand(
            front_left=left,
            front_right=right,
        )

        command = self.output_conditioner.condition(
            raw_command
        )

        self.lvl2.DRIVE_POWER(
            left_power=command.front_left,
            right_power=command.front_right,
        )

    # --------------------------------------------------
    # Primitive compatibility
    # --------------------------------------------------

    def is_busy(self) -> bool:
        """
        Initial encoder backend will be blocking.

        When drive() or rotate() returns, motion is complete.
        """
        return False

    def stop(self):
        self.lvl2.DRIVE_STOP()

    # --------------------------------------------------
    # Public API
    # --------------------------------------------------

    def drive(
            self,
            distance_mm: float,
    ):
        if distance_mm == 0:
            return

        direction = 1.0 if distance_mm > 0 else -1.0
        target_mm = abs(float(distance_mm))

        # Initial version: use the existing known-good low drive power.
        power = float(self.cal.drive_power_short)

        # Allow for encoder polling, startup and real-world variation.
        if target_mm <= self.cal.drive_switch_mm:
            expected_s = (
                    self.cal.drive_m_short * target_mm
                    + self.cal.drive_b_short
            )
        else:
            expected_s = (
                    self.cal.drive_m_long * target_mm
                    + self.cal.drive_b_long
            )

        timeout_s = max(
            2.0,
            expected_s * 3.0 + 1.0,
        )

        tolerance_mm = 10.0
        stall_timeout_s = 1.0
        stall_progress_mm = 2.0

        start_left, start_right = self._read_drive_counts()

        start_time = time.monotonic()
        last_progress_time = start_time
        last_progress_mm = 0.0

        print(
            f"[ENCODER] DRIVE "
            f"target={distance_mm:.1f}mm "
            f"power={power:.2f} "
            f"timeout={timeout_s:.2f}s"
        )

        try:
            self._set_power(
                direction * power,
                direction * power,
            )

            while True:
                left_mm, right_mm, average_mm = (
                    self._drive_progress_mm(
                        start_left,
                        start_right,
                    )
                )

                # Convert reverse motion into positive progress
                # towards the requested target.
                left_progress = direction * left_mm
                right_progress = direction * right_mm
                progress_mm = direction * average_mm

                now = time.monotonic()

                print(
                    f"[ENCODER] DRIVE "
                    f"L={left_progress:.1f}mm "
                    f"R={right_progress:.1f}mm "
                    f"AVG={progress_mm:.1f}mm"
                )

                # Target reached.
                if progress_mm >= target_mm - tolerance_mm:
                    return

                # Catch a badly stalled / disconnected side rather
                # than allowing the other wheel to drive indefinitely.
                disagreement_mm = abs(
                    left_progress - right_progress
                )

                disagreement_limit_mm = max(
                    30.0,
                    target_mm * 0.20,
                )

                if disagreement_mm > disagreement_limit_mm:
                    raise RuntimeError(
                        "Drive encoder disagreement: "
                        f"left={left_progress:.1f}mm "
                        f"right={right_progress:.1f}mm"
                    )

                # Overall movement stall detection.
                if progress_mm >= (
                        last_progress_mm + stall_progress_mm
                ):
                    last_progress_mm = progress_mm
                    last_progress_time = now

                elif (
                        now - last_progress_time
                        > stall_timeout_s
                ):
                    raise RuntimeError(
                        "Drive stalled: "
                        f"progress={progress_mm:.1f}mm"
                    )

                if now - start_time > timeout_s:
                    raise RuntimeError(
                        "Drive timeout: "
                        f"target={target_mm:.1f}mm "
                        f"progress={progress_mm:.1f}mm"
                    )

                time.sleep(0.01)

        finally:
            self.stop()

    def rotate(
        self,
        angle_deg: float,
    ):
        raise NotImplementedError(
            "Encoder rotate control not implemented yet"
        )