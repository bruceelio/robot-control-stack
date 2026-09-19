# motion_backends/odometry.py

from __future__ import annotations

import math
import time


from navigation.odometry.base import OdometrySource
from motion_backends.motor_output_conditioner import (
    MotorOutputConditioner,
    MotorPowerCommand,
)


class OdometryMotionBackend:
    """
    Odometry-controlled position dead_reckoning backend.

    Uses:
        - resolved Config
        - resolved Calibration
        - robot-relative dead_reckoning feedback

    Provides:
        - drive(distance_mm)
        - rotate(angle_deg)

    Current implementation:
    - consumes an injected robot-relative odometry source
    - preserves the proven drive control loop
    - remains independent of the underlying odometry sensors
    """

    def __init__(
            self,
            lvl2,
            config,
            calibration,
            *,
            odometry: OdometrySource,
    ):
        self.lvl2 = lvl2
        self.cfg = config
        self.cal = calibration

        self.output_conditioner = MotorOutputConditioner(
            config=config,
        )

        self.odometry = odometry


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
        Initial odometry backend will be blocking.

        When drive() or rotate() returns, dead_reckoning is complete.
        """
        return False

    def stop(self):
        self.lvl2.DRIVE_STOP()

    # --------------------------------------------------
    # Estimation
    # --------------------------------------------------

    def estimate_drive_duration(
            self,
            *,
            distance_mm: float,
    ) -> tuple[float, float]:
        """
        Return (clamped_distance_mm, expected_duration_s).

        Duration is an estimate for localisation and timeout purposes.
        Odometry feedback determines the actual stopping point.
        """

        if abs(distance_mm) < self.cfg.min_drive_mm:
            return 0.0, 0.0

        distance_mm = max(
            -self.cfg.max_drive_mm,
            min(self.cfg.max_drive_mm, distance_mm),
        )

        abs_distance_mm = abs(distance_mm)

        if abs_distance_mm < self.cal.drive_switch_mm:
            m = self.cal.drive_m_short
            b = self.cal.drive_b_short
        else:
            m = self.cal.drive_m_long
            b = self.cal.drive_b_long

        duration_s = (
                             m * abs_distance_mm + b
                     ) * self.cfg.drive_factor

        return distance_mm, duration_s

    def estimate_rotate_duration(
            self,
            *,
            angle_deg: float,
    ) -> tuple[float, float]:
        """
        Return (clamped_angle_deg, expected_duration_s).

        Duration is an estimate for localisation and timeout purposes.
        Odometry feedback determines the actual stopping point.
        """

        if abs(angle_deg) < self.cfg.min_rotate_deg:
            return 0.0, 0.0

        angle_deg = max(
            -self.cfg.max_rotate_deg,
            min(self.cfg.max_rotate_deg, angle_deg),
        )

        abs_angle_deg = abs(angle_deg)

        if abs_angle_deg < self.cal.rotate_switch_deg:
            m = self.cal.rotate_m_small
            b = self.cal.rotate_b_small
        else:
            m = self.cal.rotate_m_large
            b = self.cal.rotate_b_large

        duration_s = (
                             m * abs_angle_deg + b
                     ) * self.cfg.rotate_factor

        return angle_deg, duration_s

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

        self.odometry.reset()

        start_time = time.monotonic()
        last_progress_time = start_time
        last_progress_mm = 0.0

        print(
            f"[ODOMETRY] DRIVE "
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
                motion = self.odometry.read()

                # Convert reverse dead_reckoning into positive progress
                # towards the requested target.
                progress_mm = direction * motion.forward_mm

                now = time.monotonic()

                print(
                    f"[ODOMETRY] DRIVE "
                    f"progress={progress_mm:.1f}mm"
                )

                # Target reached.
                if progress_mm >= target_mm - tolerance_mm:
                    return

                # Let the odometry implementation decide whether
                # its measurements are internally consistent.
                disagreement_limit_mm = max(
                    30.0,
                    target_mm * 0.20,
                )

                self.odometry.check_consistency(
                    limit_mm=disagreement_limit_mm,
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
        if angle_deg == 0:
            return

        # Canonical convention:
        #   positive = left / counter-clockwise
        #   negative = right / clockwise
        direction = 1.0 if angle_deg > 0 else -1.0
        target_deg = abs(float(angle_deg))

        if target_deg < self.cal.rotate_switch_deg:
            power = float(self.cal.rotate_power_small)

            expected_s = (
                    self.cal.rotate_m_small * target_deg
                    + self.cal.rotate_b_small
            )
        else:
            power = float(self.cal.rotate_power_large)

            expected_s = (
                    self.cal.rotate_m_large * target_deg
                    + self.cal.rotate_b_large
            )

        timeout_s = max(
            2.0,
            expected_s * 3.0 + 1.0,
        )

        tolerance_deg = 3.0
        stall_timeout_s = 1.0
        stall_progress_deg = 1.0

        self.odometry.reset()

        start_time = time.monotonic()
        last_progress_time = start_time
        last_progress_deg = 0.0

        print(
            f"[ODOMETRY] ROTATE "
            f"target={angle_deg:.1f}deg "
            f"power={power:.2f} "
            f"timeout={timeout_s:.2f}s"
        )

        try:
            self._set_power(
                -direction * power,
                direction * power,
            )

            while True:
                motion = self.odometry.read()

                if motion.heading_rad is None:
                    raise RuntimeError(
                        "Odometry source does not provide heading"
                    )

                heading_deg = math.degrees(
                    motion.heading_rad
                )

                # Convert either rotation direction into
                # positive progress toward the target.
                progress_deg = direction * heading_deg

                remaining_deg = target_deg - progress_deg

                now = time.monotonic()

                print(
                    f"[ODOMETRY] ROTATE "
                    f"progress={progress_deg:.1f}deg"
                )

                if progress_deg >= (
                        target_deg - tolerance_deg
                ):
                    return

                if (
                        power != float(self.cal.rotate_power_small)
                        and remaining_deg <= 15.0
                ):
                    power = float(self.cal.rotate_power_small)

                    print(
                        f"[ODOMETRY] ROTATE "
                        f"slowing remaining={remaining_deg:.1f}deg "
                        f"power={power:.2f}"
                    )

                    self._set_power(
                        -direction * power,
                        direction * power,
                    )

                # Do not call check_consistency() here.
                # Opposite wheel travel is expected during rotation.

                if progress_deg >= (
                        last_progress_deg + stall_progress_deg
                ):
                    last_progress_deg = progress_deg
                    last_progress_time = now

                elif (
                        now - last_progress_time
                        > stall_timeout_s
                ):
                    raise RuntimeError(
                        "Rotate stalled: "
                        f"progress={progress_deg:.1f}deg"
                    )

                if now - start_time > timeout_s:
                    raise RuntimeError(
                        "Rotate timeout: "
                        f"target={target_deg:.1f}deg "
                        f"progress={progress_deg:.1f}deg"
                    )

                time.sleep(0.01)

        finally:
            self.stop()