# autonomous/SR2026/pickup_object.py

"""Final pickup following the vision-based approach handoff.

LOW:
    fresh single/two-face alignment -> low pickup height -> drive THROUGH
    -> pre-grasp range/contact diagnostics -> grab -> low retreat height
    -> reverse full commanded commitment -> ultrasonic at low retreat
    -> robot-specific carry height.

HIGH:
    high pickup height -> marker alignment -> blind commitment -> grab
    -> high retreat height -> reverse full commanded commitment
    -> lower to low retreat height -> same ultrasonic verification
    -> robot-specific carry height.

The ultrasonic and ToFs do NOT control the final LOW drive. Bumpers remain
contact diagnostics, NOT grip verification. All lift positions and verification
limits come from resolved per-robot config.
"""

from __future__ import annotations

import math
import statistics
import time

from autonomous.SR2026.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus
from primitives.motion import Drive
from primitives.manipulation.grab import Grab
from motion_backends.velocity import VelocityMotionBackend
from navigation.command.velocity_arbiter import VelocityCommand


from perception.providers.pickup_range_resolver import resolve_pickup_range
from skills.navigation.align_to_target import AlignToTarget


class PickupObject(Behavior):
    # The current successful blind-drive calibration is retained:
    # final_drive_mm = handoff distance + final_approach_marker_push.

    LOW_FINAL_ALIGNMENT_TOLERANCE_DEG = 1.0

    LOW_READY_SAMPLES = 3
    LOW_READY_MAX_TICKS = 5
    LOW_READY_SETTLE_S = 0.10
    LOW_READY_MAX_SPREAD_MM = 10.0
    # Even before exact calibration, a metre-away return cannot be
    # the cube pressed into the gripper. This is a conservative guard.
    LOW_READY_GROSS_MAX_MM = 250.0

    # Not calibrated yet. Set both limits in the robot configuration ONLY
    # after logging the ultrasonic reading for successful pressed-up cubes:
    # pickup_low_ready_ultrasonic_min_mm = ...
    # pickup_low_ready_ultrasonic_max_mm = ...
    #
    # An unset window means "log/calibrate; do not reject the grasp".
    LOW_READY_ULTRASONIC_MIN_MM = None
    LOW_READY_ULTRASONIC_MAX_MM = None

    # Bump switches are configured by the active robot profile, not by
    # robot identity. SR2026 simulator asserts True on physical contact.
    # Both must read True in the same sample during or after the push.
    #
    # Optional, separately calibrated common window for fitted ToFs.
    # With two fitted ToFs BOTH must pass; we never average their readings.
    LOW_READY_TOF_MIN_MM = None
    LOW_READY_TOF_MAX_MM = None

    LOW_RECOVERY_EXTRA_MM = 100.0

    # LOW pickup contact alignment.
    #
    # A single bumper may legitimately arrive one or two sensor samples
    # before the other, so do not react immediately.
    LOW_BUMPER_GRACE_S = 0.15

    # If only one bumper remains pressed after the grace period, make a
    # strong pivot toward the contacted side.
    LOW_BUMPER_CORRECTION_MAX_S = 0.40

    # Sampling interval during the bounded post-drive correction.
    LOW_BUMPER_SAMPLE_S = 0.05

    # Once both bumpers contact, drive straight briefly to seat the cube.
    LOW_BUMPER_SEAT_S = 0.08

    def __init__(self):
        super().__init__()
        self.config = None
        self.target_id = None
        self.distance_mm = 0.0
        self.bearing_deg = 0.0
        self.target_is_high = False
        self.final_drive_mm = 0.0

        self._step = None
        self._align = None
        self._drive = None
        self._recovery = None
        self._grasp = None
        self._retreat = None
        self.grip_verified = None  # True, False, or None (not configured)
        self.grip_measurement_mm = None



        self._ready_started_s = None
        self._ready_ticks = 0
        self._ready_samples = {"centre": [], "left": [], "right": []}
        self._recovery_reason = None
        self.ready_measurement_mm = None  # Median of pressed-up raw ultrasonic samples.
        self._pickup_lvl2 = None
        self._bumper_io = None
        self._bumper_seen = {"front_left": False, "front_right": False}
        self._bumper_both_contact = False
        self._bumper_samples = 0
        self._bumper_single_side = None
        self._bumper_single_since_s = None
        self._bumper_correction_active = False
        self._bumper_correction_used = False
        self._bumper_velocity_backend = None
        self._bumper_velocity_active = False
        self._bumper_nominal_wheel_mps = None
        self._bumper_localisation = None

    def start(
        self,
        *,
        config,
        lvl2,
        motion_backend,
        distance_mm: float,
        bearing_deg: float,
        target_is_high: bool,
        target_id: int | None = None,
        **_,
    ):
        self.config = config
        self.status = BehaviorStatus.RUNNING
        self.target_id = None if target_id is None else int(target_id)
        self.distance_mm = float(distance_mm)
        self.bearing_deg = float(bearing_deg)
        self.target_is_high = bool(target_is_high)

        # Resolved from the active robot profile via config/schema.py.
        push_mm = float(config.final_approach_marker_push)
        self.final_drive_mm = max(0.0, self.distance_mm + push_mm)
        if not math.isfinite(self.final_drive_mm) or self.final_drive_mm <= 0.0:
            raise ValueError("pickup final commitment must be finite and > 0 mm")

        self._align = None
        self._drive = None
        self._recovery = None
        self._grasp = None
        self._retreat = None
        self.grip_verified = None
        self.grip_measurement_mm = None

        self._ready_started_s = None
        self._ready_ticks = 0
        self._ready_samples = {"centre": [], "left": [], "right": []}
        self._recovery_reason = None
        self.ready_measurement_mm = None
        self._pickup_lvl2 = lvl2
        self._bumper_io = getattr(lvl2, "io", None)
        self._bumper_seen = {"front_left": False, "front_right": False}
        self._bumper_both_contact = False
        self._bumper_samples = 0
        self._bumper_single_side = None
        self._bumper_single_since_s = None
        self._bumper_correction_active = False
        self._bumper_correction_used = False
        self._bumper_velocity_backend = None
        self._bumper_velocity_active = False
        self._bumper_nominal_wheel_mps = None
        self._bumper_localisation = None

        print(
            f"[[PICKUP_OBJECT]] start id={self.target_id} "
            f"high={self.target_is_high} dist={self.distance_mm:.0f}mm "
            f"bearing={self.bearing_deg:+.1f}deg "
            f"commitment={self.final_drive_mm:.0f}mm"
        )

        if self.target_is_high:
            # The high position is camera-compatible for the HIGH align.
            if not self._command_lift(
                    lvl2,
                    "lift_high_pickup_position",
                    "HIGH_PICKUP",
            ):
                return self.status

            self._start_align(
                bearing_deg=self.bearing_deg,
                tolerance_deg=0.0,
                motion_backend=motion_backend,
                next_step="ALIGN_HIGH",
            )

        else:
            # Perception has already resolved the physical object heading.
            # PickupObject only executes the supplied final alignment.
            self._start_align(
                bearing_deg=self.bearing_deg,
                tolerance_deg=(
                    self.LOW_FINAL_ALIGNMENT_TOLERANCE_DEG
                ),
                motion_backend=motion_backend,
                next_step="ALIGN_LOW",
            )

        return self.status

    def _command_lift(self, lvl2, config_attr, label):
        """Command a named robot-profile position through canonical IO."""
        try:
            position = float(getattr(self.config, config_attr))
            if not math.isfinite(position) or not -1.0 <= position <= 1.0:
                raise ValueError(f"{config_attr} out of servo range: {position}")
            io = lvl2.io
            io.servo["lift"].position = position
            settle_s = float(self.config.pickup_lift_settle_s)
            if not math.isfinite(settle_s) or settle_s < 0.0:
                raise ValueError("invalid pickup_lift_settle_s")
            print(f"[PICKUP_LIFT][{label}] position={position:+.2f} "
                  f"settle={settle_s:.2f}s")
            if settle_s > 0.0:
                io.sleep(settle_s)
            return True
        except (AttributeError, KeyError, TypeError, ValueError,
                RuntimeError) as exc:
            print(f"[PICKUP_LIFT][{label}] FAILED: {exc}")
            self.status = BehaviorStatus.FAILED
            return False

    def _start_retreat(self, *, lvl2, motion_backend):
        position = ("lift_high_retreat_position" if self.target_is_high
                    else "lift_low_retreat_position")
        if not self._command_lift(lvl2, position, "RETREAT"):
            return self.status
        print(f"[PICKUP_RETREAT] reverse full commitment "
              f"{-self.final_drive_mm:.1f}mm")
        self._retreat = Drive(distance_mm=-self.final_drive_mm)
        self._retreat.start(motion_backend=motion_backend)
        self._step = "RETREAT"
        return PrimitiveStatus.RUNNING

    def _finish_after_retreat(
            self,
            *,
            lvl2,
            io,
            perception,
            now_s,
    ):
        """Verify at LOW_RETREAT on both paths, then restore camera view."""
        if not self.target_is_high:
            print("[PICKUP_LIFT][VERIFY] already at LOW_RETREAT")
        elif not self._command_lift(lvl2, "lift_low_retreat_position",
                                    "HIGH_TO_VERIFY"):
            return self.status

        # --------------------------------------------------
        # Post-retreat verification
        #
        # Only positive evidence may fail the pickup:
        #   1. ultrasonic clearly says the gripper is empty, or
        #   2. fresh vision sees the selected cube left behind.
        #
        # Anything inconclusive defaults to success.
        # --------------------------------------------------

        self.grip_verified = None

        verify_enabled = bool(
            self.config.pickup_grip_verify_enabled
        )

        # --------------------------------------------------
        # Ultrasonic evidence
        # --------------------------------------------------

        if verify_enabled and self._fitted("centre"):
            if io is None:
                io = lvl2.io

            readings = []

            n = int(
                self.config.pickup_grip_verify_samples
            )
            min_valid = int(
                self.config.pickup_grip_verify_min_valid_samples
            )

            if n <= 0 or not 0 < min_valid <= n:
                raise ValueError(
                    "invalid pickup grip sample configuration"
                )

            low = float(
                self.config.pickup_grip_verify_min_mm
            )
            high = float(
                self.config.pickup_grip_verify_max_mm
            )
            spread_limit = float(
                self.config.pickup_grip_verify_max_spread_mm
            )
            delay_s = float(
                self.config.pickup_grip_verify_sample_delay_s
            )

            for index in range(n):
                try:
                    observation = resolve_pickup_range(
                        config=self.config,
                        io=io,
                    )
                    reading = self._raw_valid_mm(
                        observation,
                        "centre",
                    )
                except (
                    AttributeError,
                    KeyError,
                    TypeError,
                    ValueError,
                    RuntimeError,
                ) as exc:
                    print(
                        f"[PICKUP_GRIP][READ] {exc}"
                    )
                    reading = None

                print(
                    f"[PICKUP_GRIP][SAMPLE] "
                    f"{index + 1}/{n} "
                    f"ultrasonic_mm={reading}"
                )

                if reading is not None:
                    readings.append(reading)

                if index + 1 < n and delay_s > 0.0:
                    io.sleep(delay_s)

            self.grip_measurement_mm = (
                statistics.median(readings)
                if readings
                else None
            )

            spread = (
                max(readings) - min(readings)
                if readings
                else None
            )

            held = (
                len(readings) >= min_valid
                and low
                <= self.grip_measurement_mm
                <= high
                and spread <= spread_limit
            )

            gross_failure_mm = float(getattr(
                self.config,
                "pickup_low_ready_gross_max_mm",
                self.LOW_READY_GROSS_MAX_MM,
            ))

            clearly_empty = (
                len(readings) >= min_valid
                and all(
                    reading > gross_failure_mm
                    for reading in readings
                )
            )

            if held:
                self.grip_verified = True
                evidence = "HELD"

            elif clearly_empty:
                self.grip_verified = False
                evidence = "EMPTY"

            else:
                evidence = "INCONCLUSIVE"

            print(
                f"[PICKUP_GRIP][RESULT] "
                f"valid={len(readings)}/{n} "
                f"median_mm={self.grip_measurement_mm} "
                f"spread_mm={spread} "
                f"expected=({low},{high}) "
                f"evidence={evidence}"
            )

        elif verify_enabled:
            print(
                "[PICKUP_GRIP] ultrasonic not fitted "
                "-> vision verification only"
            )

        else:
            print(
                "[PICKUP_GRIP] ultrasonic verification disabled "
                "-> vision verification only"
            )

        # --------------------------------------------------
        # Vision evidence
        #
        # Only needed when ultrasonic has NOT already
        # positively confirmed held/empty.
        #
        # A cube left behind should be approximately:
        #
        #     final commitment + 100 mm buffer
        #
        # from the robot after retreat.
        #
        # Accept +/- 100 mm around that known geometry.
        # --------------------------------------------------

        if (
            self.grip_verified is None
            and perception is not None
            and self.target_id is not None
        ):
            target = None

            for memory in perception.objects.values():
                candidate = memory.get(self.target_id)

                if candidate is None:
                    continue

                last_seen = float(
                    candidate.get("last_seen", 0.0)
                )

                if (
                    now_s - last_seen
                    > float(self.config.visible_max_age_s)
                ):
                    continue

                if "distance" not in candidate:
                    continue

                target = candidate
                break

            expected_mm = (
                self.final_drive_mm
                + self.LOW_RECOVERY_EXTRA_MM
            )

            min_expected_mm = expected_mm - 100.0
            max_expected_mm = expected_mm + 100.0

            if target is not None:
                distance_mm = float(
                    target["distance"]
                )

                if (
                    min_expected_mm
                    <= distance_mm
                    <= max_expected_mm
                ):
                    self.grip_verified = False

                    print(
                        "[PICKUP_GRIP][VISION] "
                        f"id={self.target_id} "
                        f"distance={distance_mm:.0f}mm "
                        f"expected_left_behind="
                        f"{expected_mm:.0f}+/-100mm "
                        "-> CONFIRMED FAILED"
                    )

                else:
                    print(
                        "[PICKUP_GRIP][VISION] "
                        f"id={self.target_id} "
                        f"distance={distance_mm:.0f}mm "
                        "outside left-behind position "
                        "-> inconclusive"
                    )

            else:
                print(
                    "[PICKUP_GRIP][VISION] "
                    f"id={self.target_id} not freshly visible "
                    "-> no failure evidence"
                )

        # Always restore the carry height before leaving PickupObject,
        # including a failed verification: camera visibility is needed for
        # either realignment or target reacquisition.
        if not self._command_lift(lvl2, "lift_carry_position", "CARRY"):
            return self.status

        self.status = (BehaviorStatus.FAILED if self.grip_verified is False
                       else BehaviorStatus.SUCCEEDED)
        print(f"[PICKUP_OBJECT] {'complete' if self.status == BehaviorStatus.SUCCEEDED else 'FAILED'} "
              f"id={self.target_id} grip_verified={self.grip_verified} "
              f"held_ultrasonic_mm={self.grip_measurement_mm} "
              f"pressed_ultrasonic_mm={self.ready_measurement_mm} "
              f"bumper_both_contact={self._bumper_both_contact}")
        return self.status

    def _start_align(
        self,
        *,
        bearing_deg,
        tolerance_deg,
        motion_backend,
        next_step,
    ):
        self._align = AlignToTarget(
            bearing_deg=float(
                bearing_deg
            ),
            tolerance_deg=float(
                tolerance_deg
            ),
            max_rotate_deg=float(
                self.config.max_rotate_deg
            ),
        )
        self._align.start(motion_backend=motion_backend)
        self._step = next_step

    def _start_low_commitment(
            self,
            *,
            lvl2,
            motion_backend,
    ):
        """
        Lower to the calibrated LOW pickup position only after
        final heading alignment, then begin the blind commitment.
        """

        if not self._command_lift(
                lvl2,
                "lift_low_pickup_position",
                "LOW_PICKUP",
        ):
            return self.status

        return self._start_commitment(
            motion_backend
        )



    def _bumper_fitted(self, name):
        """Use only the capability map from the resolved robot configuration."""
        has_io = getattr(self.config, "has_io", None)
        if callable(has_io):
            try:
                return bool(has_io("bumper", name))
            except KeyError:
                return False  # An older or non-SR profile has no such key.
        return bool(getattr(self.config, "io", {}).get(f"bumper.{name}"))

    def _sample_bumpers(self, *, phase, count_for_contact=False):
        """Log each fitted input. Only a simultaneous pair confirms contact.

        This deliberately reads via the canonical io.bumper collection rather
        than via raw Arduino pins; Bobbot is gated out by resolved config.
        """
        fitted = {
            name: self._bumper_fitted(name)
            for name in ("front_left", "front_right")
        }
        if not any(fitted.values()):
            return

        io = self._bumper_io
        bumpers = getattr(io, "bumper", None)
        readings = {}
        for name, present in fitted.items():
            if not present:
                readings[name] = None
                continue
            if bumpers is None:
                readings[name] = None
                continue
            try:
                raw = bumpers[name]
                readings[name] = None if raw is None else bool(raw)
            except (KeyError, AttributeError, TypeError, RuntimeError) as exc:
                print(f"[PICKUP_BUMPER][READ] {name} unavailable: {exc}")
                readings[name] = None

        left = readings["front_left"]
        right = readings["front_right"]
        both = left is True and right is True
        if count_for_contact:
            self._bumper_samples += 1
            for name in self._bumper_seen:
                self._bumper_seen[name] |= readings[name] is True
            self._bumper_both_contact |= both
        print(
            f"[PICKUP_BUMPER][{phase}] "
            f"left={left} right={right} both={both} "
            f"confirmed={self._bumper_both_contact}"
        )
        return readings

    def _bumper_summary(self):
        if not (self._bumper_fitted("front_left")
                or self._bumper_fitted("front_right")):
            return
        print(
            "[PICKUP_BUMPER][SUMMARY] "
            f"id={self.target_id} "
            f"left_seen={self._bumper_seen['front_left']} "
            f"right_seen={self._bumper_seen['front_right']} "
            f"both_simultaneously={self._bumper_both_contact} "
            f"correction_used={self._bumper_correction_used} "
            f"contact_samples={self._bumper_samples}"
        )

    def _bumper_now(self):
        io = self._bumper_io
        clock = None if io is None else getattr(io, "time", None)

        if callable(clock):
            return float(clock())

        return time.monotonic()

    def _pickup_contact_power(self, motion_backend):
        """
        Recover the normal forward power selected by the timed backend for
        this particular pickup distance.

        Contact alignment is currently intended for profiles which expose
        both front bumpers (Webots at present).
        """
        cal = getattr(motion_backend, "cal", None)
        if cal is None:
            raise RuntimeError(
                "Pickup bumper alignment requires timed motion calibration"
            )

        if abs(self.final_drive_mm) < float(cal.drive_switch_mm):
            power = cal.drive_power_short
        else:
            power = cal.drive_power_long

        return abs(float(power))

    def _ensure_bumper_velocity_backend(
            self,
            *,
            lvl2,
            motion_backend,
    ):
        if self._bumper_velocity_backend is not None:
            return

        calibration = getattr(
            motion_backend,
            "cal",
            None,
        )

        if calibration is None:
            raise RuntimeError(
                "Pickup bumper velocity control requires "
                "resolved motion calibration"
            )

        self._bumper_velocity_backend = VelocityMotionBackend(
            lvl2=lvl2,
            config=self.config,
            calibration=calibration,
            localisation=self._bumper_localisation,
            io=self._bumper_io,
        )

    def _pickup_contact_wheel_speed_mps(
            self,
            *,
            motion_backend,
            nominal_power,
    ):
        """
        Return the calibrated wheel speed corresponding to the same
        nominal motor power used by the pickup timed drive.
        """
        cal = getattr(
            motion_backend,
            "cal",
            None,
        )

        if cal is None:
            raise RuntimeError(
                "Pickup bumper velocity control requires "
                "resolved motion calibration"
            )

        for power, velocity_mm_s in cal.drive_velocity_curve:
            if math.isclose(
                    abs(float(power)),
                    abs(float(nominal_power)),
                    rel_tol=1e-6,
                    abs_tol=1e-6,
            ):
                return abs(float(velocity_mm_s)) / 1000.0

        raise RuntimeError(
            "Pickup bumper nominal power has no matching "
            "DRIVE_VELOCITY_CURVE point: "
            f"power={nominal_power}"
        )

    def _command_bumper_wheels(
            self,
            *,
            left_scale,
            right_scale,
    ):
        """
        Express pickup contact steering as a canonical differential-drive
        body velocity.

        left_scale/right_scale:
            1.0 = normal pickup wheel speed
            0.0 = stopped wheel
        """
        backend = self._bumper_velocity_backend
        wheel_mps = self._bumper_nominal_wheel_mps

        if backend is None or wheel_mps is None:
            raise RuntimeError(
                "Pickup bumper velocity backend is not initialised"
            )

        left_mps = (
                float(left_scale)
                * float(wheel_mps)
        )

        right_mps = (
                float(right_scale)
                * float(wheel_mps)
        )

        linear_x_mps = (
                               left_mps + right_mps
                       ) / 2.0

        track_width_m = (
                float(self.config.drive_track_width_mm)
                / 1000.0
        )

        angular_z_rps = (
                                right_mps - left_mps
                        ) / track_width_m

        now_s = self._bumper_now()

        backend.update(
            VelocityCommand(
                linear_x_mps=linear_x_mps,
                angular_z_rps=angular_z_rps,
                lateral_y_mps=0.0,
                timestamp=now_s,
            )
        )

        self._bumper_velocity_active = True

    def _stop_bumper_velocity(self):
        if (
                not self._bumper_velocity_active
                or self._bumper_velocity_backend is None
        ):
            return

        now_s = self._bumper_now()

        self._bumper_velocity_backend.update(
            VelocityCommand(
                linear_x_mps=0.0,
                angular_z_rps=0.0,
                lateral_y_mps=0.0,
                timestamp=now_s,
            )
        )

        self._bumper_velocity_active = False

    def _bumper_contact_control_sample(
        self,
        *,
        lvl2,
        nominal_power,
        phase,
    ):
        """
        Sample the two front bumpers and apply LOW-pickup contact steering.

        States:
          neither -> straight
          one only -> grace, then strong pivot toward that bumper
          both     -> equalise motors
        """
        readings = self._sample_bumpers(
            phase=phase,
            count_for_contact=True,
        )

        if readings is None:
            return "unavailable"

        left = readings["front_left"]
        right = readings["front_right"]
        now_s = self._bumper_now()

        # --------------------------------------------------
        # Both bumpers: aligned enough. Return to straight.
        # --------------------------------------------------
        if left is True and right is True:
            if self._bumper_correction_active:
                self._command_bumper_wheels(
                    left_scale=1.0,
                    right_scale=1.0,
                )
                print(
                    "[PICKUP_BUMPER][EQUALISE] "
                    f"both contact -> "
                    f"L={nominal_power:.2f} "
                    f"R={nominal_power:.2f}"
                )

            self._bumper_correction_active = False
            self._bumper_single_side = None
            self._bumper_single_since_s = None
            return "both"

        # --------------------------------------------------
        # Determine whether exactly one bumper is pressed.
        # Require the opposite sensor to explicitly report False;
        # do not steer on an unavailable/None reading.
        # --------------------------------------------------
        if left is True and right is False:
            side = "left"

        elif right is True and left is False:
            side = "right"

        elif left is False and right is False:
            if self._bumper_correction_active:
                self._command_bumper_wheels(
                    left_scale=1.0,
                    right_scale=1.0,
                )
                print(
                    "[PICKUP_BUMPER][STRAIGHT] "
                    "contact lost -> restore equal power"
                )

            self._bumper_correction_active = False
            self._bumper_single_side = None
            self._bumper_single_since_s = None
            return "none"

        else:
            # One or both sensor readings are unavailable.
            # Do not turn based on incomplete information.
            if self._bumper_correction_active:
                self._command_bumper_wheels(
                    left_scale=1.0,
                    right_scale=1.0,
                )

            self._bumper_correction_active = False
            self._bumper_single_side = None
            self._bumper_single_since_s = None
            return "unknown"

        # --------------------------------------------------
        # First observation of one-sided contact:
        # continue straight for the grace period.
        # --------------------------------------------------
        if self._bumper_single_side != side:
            self._bumper_single_side = side
            self._bumper_single_since_s = now_s

            if self._bumper_correction_active:
                self._command_bumper_wheels(
                    left_scale=1.0,
                    right_scale=1.0,
                )
                self._bumper_correction_active = False

            print(
                "[PICKUP_BUMPER][GRACE] "
                f"{side}=True other=False "
                f"grace={self.LOW_BUMPER_GRACE_S:.2f}s"
            )
            return "grace"

        elapsed_s = now_s - self._bumper_single_since_s

        if elapsed_s < self.LOW_BUMPER_GRACE_S:
            return "grace"

        # --------------------------------------------------
        # Persistent one-sided contact:
        # pivot strongly TOWARD the contacted bumper.
        #
        # Right bumper -> stop right wheel -> swing right.
        # Left bumper  -> stop left wheel  -> swing left.
        # --------------------------------------------------
        if side == "right":
            left_power = nominal_power
            right_power = 0.0
            left_scale = 1.0
            right_scale = 0.0
        else:
            left_power = 0.0
            right_power = nominal_power
            left_scale = 0.0
            right_scale = 1.0

        self._command_bumper_wheels(
            left_scale=left_scale,
            right_scale=right_scale,
        )

        if not self._bumper_correction_active:
            self._bumper_correction_active = True
            self._bumper_correction_used = True

            print(
                "[PICKUP_BUMPER][CORRECT] "
                f"side={side} "
                f"single_for={elapsed_s:.3f}s "
                f"L={left_power:.2f} "
                f"R={right_power:.2f}"
            )

        return "correcting"

    def _run_bumper_alignment_extension(
        self,
        *,
        lvl2,
        nominal_power,
    ):
        """
        If the normal calibrated drive ends with only one bumper contacting,
        allow a bounded pickup-specific continuation.

        This is deliberately NOT part of generic Level2.DRIVE().
        """
        if self._bumper_both_contact:
            return

        if self._bumper_single_side is None:
            return

        io = self._bumper_io
        if io is None:
            return

        print(
            "[PICKUP_BUMPER][EXTEND] "
            f"side={self._bumper_single_side} "
            f"max={self.LOW_BUMPER_CORRECTION_MAX_S:.2f}s"
        )

        # Level2's normal timed drive has just stopped. Resume straight while
        # any remaining grace period expires.
        self._bumper_correction_active = False

        self._command_bumper_wheels(
            left_scale=1.0,
            right_scale=1.0,
        )

        deadline_s = (
            self._bumper_now()
            + self.LOW_BUMPER_CORRECTION_MAX_S
        )

        try:
            while self._bumper_now() < deadline_s:
                state = self._bumper_contact_control_sample(
                    lvl2=lvl2,
                    nominal_power=nominal_power,
                    phase="ALIGN",
                )

                if state == "both":
                    print(
                        "[PICKUP_BUMPER][SEAT] "
                        f"both contact -> straight "
                        f"{self.LOW_BUMPER_SEAT_S:.2f}s"
                    )

                    self._command_bumper_wheels(
                        left_scale=1.0,
                        right_scale=1.0,
                    )

                    io.sleep(self.LOW_BUMPER_SEAT_S)

                    self._sample_bumpers(
                        phase="SEAT_END",
                        count_for_contact=True,
                    )
                    return

                if state in ("none", "unknown", "unavailable"):
                    print(
                        "[PICKUP_BUMPER][EXTEND] "
                        "contact lost/unavailable -> stop correction"
                    )
                    return

                remaining_s = deadline_s - self._bumper_now()

                if remaining_s <= 0.0:
                    break

                io.sleep(
                    min(
                        self.LOW_BUMPER_SAMPLE_S,
                        remaining_s,
                    )
                )

            print(
                "[PICKUP_BUMPER][CORRECT_TIMEOUT] "
                f"both_contact={self._bumper_both_contact}"
            )

        finally:
            self._stop_bumper_velocity()

    def _start_commitment(self, motion_backend):
        self._drive = Drive(distance_mm=self.final_drive_mm)
        self._sample_bumpers(phase="BEFORE_DRIVE")

        lvl2 = getattr(motion_backend, "lvl2", None)

        bumper_names = (
            "front_left",
            "front_right",
        )

        any_bumper_fitted = any(
            self._bumper_fitted(name)
            for name in bumper_names
        )

        bumper_pair_fitted = all(
            self._bumper_fitted(name)
            for name in bumper_names
        )

        nominal_power = None

        if bumper_pair_fitted:
            nominal_power = self._pickup_contact_power(
                motion_backend
            )

            self._ensure_bumper_velocity_backend(
                lvl2=lvl2,
                motion_backend=motion_backend,
            )

            self._bumper_nominal_wheel_mps = (
                self._pickup_contact_wheel_speed_mps(
                    motion_backend=motion_backend,
                    nominal_power=nominal_power,
                )
            )

        previous_observer = None

        if lvl2 is not None and any_bumper_fitted:
            previous_observer = getattr(
                lvl2,
                "_timed_drive_observer",
                None,
            )

            if bumper_pair_fitted:
                lvl2._timed_drive_observer = lambda: (
                    self._bumper_contact_control_sample(
                        lvl2=lvl2,
                        nominal_power=nominal_power,
                        phase="DURING_DRIVE",
                    )
                )

            else:
                # A profile with only one fitted bumper may still log it,
                # but cannot perform two-bumper alignment.
                lvl2._timed_drive_observer = lambda: (
                    self._sample_bumpers(
                        phase="DURING_DRIVE",
                        count_for_contact=True,
                    )
                )

        try:
            self._drive.start(
                motion_backend=motion_backend
            )

        finally:
            if lvl2 is not None and any_bumper_fitted:
                lvl2._timed_drive_observer = previous_observer

        # Level2's blocking timed drive has physically stopped.

        # If bumper steering took over during that drive, close the

        # continuous commanded-velocity state at the same boundary.

        if self._bumper_velocity_active:
            self._stop_bumper_velocity()

        # Level2 has stopped the normal calibrated drive at this point.
        end_readings = self._sample_bumpers(
            phase="END_DRIVE",
            count_for_contact=True,
        )

        # It is possible for first contact to occur on the very last sample.
        # Seed the grace timer here so that case can still use the bounded
        # alignment extension.
        if (
                bumper_pair_fitted
                and not self._bumper_both_contact
                and self._bumper_single_side is None
                and end_readings is not None
        ):
            left = end_readings["front_left"]
            right = end_readings["front_right"]

            if left is True and right is False:
                self._bumper_single_side = "left"
                self._bumper_single_since_s = self._bumper_now()

            elif right is True and left is False:
                self._bumper_single_side = "right"
                self._bumper_single_since_s = self._bumper_now()

        if (
                bumper_pair_fitted
                and lvl2 is not None
                and not self._bumper_both_contact
                and self._bumper_single_side is not None
        ):
            self._run_bumper_alignment_extension(
                lvl2=lvl2,
                nominal_power=nominal_power,
            )

        print(
            "[[PICKUP_OBJECT]] final drive-through commitment "
            f"d={self.final_drive_mm:.1f}mm"
        )

        self._step = "DRIVE"
        return PrimitiveStatus.RUNNING

    def _start_grasp(self, lvl2):
        # New pickup path owns the selected retreat height. Keep the old
        # Grab -> LiftUp default for other GraspObject callers.
        self._grasp = Grab()
        self._grasp.start(lvl2=lvl2, config=self.config)
        self._step = "GRASP"
        return PrimitiveStatus.RUNNING

    def _start_recovery(self, *, motion_backend, reason):
        self._recovery_reason = str(reason)
        distance_mm = self.final_drive_mm + self.LOW_RECOVERY_EXTRA_MM
        print(
            f"[[PICKUP_OBJECT]][RECOVER] {self._recovery_reason}; "
            f"reverse={distance_mm:.1f}mm"
        )
        self._recovery = Drive(distance_mm=-distance_mm)
        self._recovery.start(motion_backend=motion_backend)
        self._step = "RECOVER_PICKUP"
        return PrimitiveStatus.RUNNING

    def _fitted(self, channel):
        cat, name = {
            "centre": ("ultrasonic", "front"),
            "left": ("tof", "front_left"),
            "right": ("tof", "front_right"),
        }[channel]
        has_io = getattr(self.config, "has_io", None)
        if callable(has_io):
            try:
                return bool(has_io(cat, name))
            except KeyError:
                return False
        return bool(getattr(self.config, "io", {}).get(f"{cat}.{name}"))

    @staticmethod
    def _raw_valid_mm(observation, channel):
        reading = None if observation is None else getattr(observation, channel, None)
        if reading is None or not reading.valid or reading.distance_mm is None:
            return None
        value = float(reading.distance_mm)
        return value if math.isfinite(value) and value > 0.0 else None

    def _window(self, category):
        prefix = (
            "pickup_low_ready_ultrasonic"
            if category == "ultrasonic"
            else "pickup_low_ready_tof"
        )
        fallback_low = (
            self.LOW_READY_ULTRASONIC_MIN_MM
            if category == "ultrasonic"
            else self.LOW_READY_TOF_MIN_MM
        )
        fallback_high = (
            self.LOW_READY_ULTRASONIC_MAX_MM
            if category == "ultrasonic"
            else self.LOW_READY_TOF_MAX_MM
        )
        minimum = getattr(self.config, prefix + "_min_mm", fallback_low)
        maximum = getattr(self.config, prefix + "_max_mm", fallback_high)
        if minimum is None and maximum is None:
            return None
        if minimum is None or maximum is None:
            raise ValueError(f"{prefix}: configure BOTH min_mm and max_mm")
        low, high = float(minimum), float(maximum)
        if not 0.0 < low <= high or not all(map(math.isfinite, (low, high))):
            raise ValueError(f"{prefix}: invalid calibrated window")
        return low, high

    def _confirm_ready(self, *, lvl2, io, motion_backend, now_s):
        if now_s - self._ready_started_s < self.LOW_READY_SETTLE_S:
            return PrimitiveStatus.RUNNING

        centre_fitted = self._fitted("centre")
        left_fitted = self._fitted("left")
        right_fitted = self._fitted("right")

        if not (centre_fitted or left_fitted or right_fitted):
            self._sample_bumpers(phase="READY_NO_RANGE", count_for_contact=True)
            self._bumper_summary()
            if self._bumper_both_contact:
                print("[PICKUP_BUMPER] simultaneous contact observed (diagnostic only)")
            print("[PICKUP_READY] no range hardware -> unverified grasp")
            return self._start_grasp(lvl2)

        # These are *pressed-up* measurements, not final-approach commands.
        observation = (
            resolve_pickup_range(config=self.config, io=io)
            if io is not None else None
        )
        self._ready_ticks += 1
        self._sample_bumpers(
            phase=f"READY_{self._ready_ticks}",
            count_for_contact=True,
        )
        for channel in self._ready_samples:
            if self._fitted(channel):
                value = self._raw_valid_mm(observation, channel)
                if value is not None:
                    self._ready_samples[channel].append(value)

        print(
            "[PICKUP_READY][SAMPLE] "
            f"tick={self._ready_ticks} "
            f"centre={self._raw_valid_mm(observation, 'centre')} "
            f"left={self._raw_valid_mm(observation, 'left')} "
            f"right={self._raw_valid_mm(observation, 'right')}"
        )

        if (
            self._ready_ticks < self.LOW_READY_SAMPLES
            and self._ready_ticks < self.LOW_READY_MAX_TICKS
        ):
            return PrimitiveStatus.RUNNING

        for channel, readings in self._ready_samples.items():
            if readings:
                median = statistics.median(readings)
                print(
                    "[PICKUP_READY][CALIBRATE] "
                    f"channel={channel} readings={readings} "
                    f"median={median:.1f}mm "
                    f"spread={max(readings) - min(readings):.1f}mm"
                )
                if channel == "centre":
                    self.ready_measurement_mm = median
            elif self._fitted(channel):
                print(f"[PICKUP_READY][CALIBRATE] channel={channel} no return")

        self._bumper_summary()
        if self._bumper_both_contact:
            # Diagnostic only for the first Webots trials. Do not override
            # existing ultrasonic checks or alter the pickup decision yet.
            print("[PICKUP_BUMPER] simultaneous contact observed (diagnostic only)")

        print(
            "[PICKUP_READY] range/contact diagnostics complete "
            "-> grasp; final decision is post-retreat"
        )
        return self._start_grasp(lvl2)

    def update(
            self,
            *,
            lvl2,
            motion_backend,
            io,
            localisation=None,
            perception=None,
            **_,
    ):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        now_s = float(io.time())
        if io is not None:
            self._bumper_io = io

        if localisation is not None:
            self._bumper_localisation = localisation

        if self._step in (
            "ALIGN_LOW",
            "ALIGN_HIGH",
        ):
            st = self._align.update(
                motion_backend=motion_backend
            )

            if st == PrimitiveStatus.RUNNING:
                return st

            if st == PrimitiveStatus.FAILED:
                print(
                    "[[PICKUP_OBJECT]] "
                    "final alignment FAILED"
                )

                self.status = (
                    BehaviorStatus.FAILED
                )

                return self.status

            print(
                "[[PICKUP_OBJECT]] "
                "final alignment complete "
                f"bearing={self.bearing_deg:+.2f}deg"
            )

            if self._step == "ALIGN_LOW":
                return self._start_low_commitment(
                    lvl2=lvl2,
                    motion_backend=motion_backend,
                )

            return self._start_commitment(
                motion_backend
            )

        if self._step == "DRIVE":
            st = self._drive.update(motion_backend=motion_backend)
            if st == PrimitiveStatus.RUNNING:
                return st
            if st == PrimitiveStatus.FAILED:
                print("[[PICKUP_OBJECT]] final drive FAILED")
                self.status = BehaviorStatus.FAILED
                return self.status

            print("[[PICKUP_OBJECT]] final drive-through complete")

            if self.target_is_high:
                return self._start_grasp(lvl2)

            self._ready_started_s = now_s
            self._ready_ticks = 0
            self._ready_samples = {"centre": [], "left": [], "right": []}
            self._step = "CONFIRM_GRASP_READY"
            return PrimitiveStatus.RUNNING

        if self._step == "CONFIRM_GRASP_READY":
            return self._confirm_ready(
                lvl2=lvl2,
                io=io,
                motion_backend=motion_backend,
                now_s=now_s,
            )

        if self._step == "RECOVER_PICKUP":
            st = self._recovery.update(motion_backend=motion_backend)
            if st == PrimitiveStatus.RUNNING:
                return st
            if st == PrimitiveStatus.FAILED:
                print("[[PICKUP_OBJECT]][RECOVER] reverse FAILED")
            else:
                print("[[PICKUP_OBJECT]][RECOVER] reverse complete; reacquire cube")
            self.status = BehaviorStatus.FAILED
            return self.status

        if self._step == "GRASP":
            st = self._grasp.update(lvl2=lvl2)
            if st == PrimitiveStatus.RUNNING:
                return st
            if st == PrimitiveStatus.FAILED:
                print("[[PICKUP_OBJECT]] GraspObject FAILED")
                self.status = BehaviorStatus.FAILED
                return self.status

            print("[[PICKUP_OBJECT]] GraspObject complete")
            return self._start_retreat(lvl2=lvl2,
                                       motion_backend=motion_backend)

        if self._step == "RETREAT":
            st = self._retreat.update(motion_backend=motion_backend)
            if st == PrimitiveStatus.RUNNING:
                return st
            if st == PrimitiveStatus.FAILED:
                print("[PICKUP_RETREAT] FAILED")
                self.status = BehaviorStatus.FAILED
                return self.status
            print("[PICKUP_RETREAT] complete")
            return self._finish_after_retreat(
                lvl2=lvl2,
                io=io,
                perception=perception,
                now_s=now_s,
            )

        self.status = BehaviorStatus.FAILED
        return self.status

    def stop(self, *, motion_backend=None):
        for child in (
            self._align,
            self._drive,
            self._recovery,
            self._grasp,
            self._retreat,
        ):
            if child is None:
                continue
            try:
                if motion_backend is not None:
                    child.stop(motion_backend=motion_backend)
                else:
                    child.stop()
            except TypeError:
                try:
                    child.stop()
                except Exception:
                    pass
            except Exception:
                pass
