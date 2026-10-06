# localisation/providers/dead_reckoning/commanded_motion.py

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional

from localisation.providers.base import PoseProvider, PoseObservation

MIN_EFFECTIVE_ROTATE_DEG = 3
DEAD_RECKONING_MAX_DISTANCE_MM = 2000.0
DEAD_RECKONING_MAX_ROTATION_DEG = 180.0

# Motion-frame calibration for synthetic pose propagation
DRIVE_HEADING_OFFSET_RAD = 0.0   # use +/- math.pi/2 if heading zero-axis is off by 90°
DRIVE_Y_SIGN = 1.0         # set to -1.0 if Y is inverted in the arena frame

@dataclass
class _Segment:
    kind: str
    start_s: float
    duration_s: float
    total_drive_mm: float = 0.0
    total_rotate_deg: float = 0.0
    applied_drive_mm: float = 0.0
    applied_rotate_deg: float = 0.0


class CommandedMotionProvider(PoseProvider):
    def __init__(self):
        super().__init__("commanded_motion", base_weight=0.25)

        self._clock_fn: Callable[[], float] | None = None

        self._x = 0.0
        self._y = 0.0
        self._heading = None

        self._position_valid = False
        self._heading_valid = False

        self._last_reseed_s = 0.0

        self._distance_since_reseed_mm = 0.0
        self._rotation_since_reseed_deg = 0.0

        self._active: Optional[_Segment] = None
        # Continuous commanded velocity currently active.
        #
        # These are canonical robot velocities:
        #   +vx = forward
        #   +wz = left / CCW
        self._velocity_linear_x_mps = 0.0
        self._velocity_angular_z_rps = 0.0
        self._velocity_last_s: Optional[float] = None

    # --------------------------------------------------
    # Lifecycle
    # --------------------------------------------------

    def set_clock(self, clock_fn: Callable[[], float]) -> None:
        """Use the robot clock for motion propagation and elapsed time."""
        self._clock_fn = clock_fn

    def _motion_time(self, fallback_now_s: float) -> float:
        if self._clock_fn is not None:
            return float(self._clock_fn())
        return float(fallback_now_s)

    def reseed(self, pose) -> None:
        if pose.position_valid:
            self._x = pose.x
            self._y = pose.y
            self._position_valid = True

        # Only accept heading if the incoming pose explicitly says it is valid.
        if pose.heading_valid and pose.heading is not None:
            self._heading = pose.heading
            self._heading_valid = True
        else:
            # Keep position, but do not pretend heading is trustworthy.
            self._heading = None
            self._heading_valid = False

        self._last_reseed_s = self._motion_time(pose.timestamp)

        self._distance_since_reseed_mm = 0.0
        self._rotation_since_reseed_deg = 0.0

        self._active = None
        self._velocity_linear_x_mps = 0.0
        self._velocity_angular_z_rps = 0.0
        self._velocity_last_s = None

        print(
            f"[CMD_MOTION][RESEED] pos_valid={pose.position_valid} "
            f"x={pose.x:.1f} y={pose.y:.1f} "
            f"hdg_valid={pose.heading_valid} heading={pose.heading}"
        )

    def invalidate(self) -> None:
        self._position_valid = False
        self._heading_valid = False

        self._active = None

        self._velocity_linear_x_mps = 0.0
        self._velocity_angular_z_rps = 0.0
        self._velocity_last_s = None

    # --------------------------------------------------
    # Motion input
    # --------------------------------------------------

    def begin_drive(self, *, distance_mm: float, duration_s: float, now_s: float):
        motion_now_s = self._motion_time(now_s)
        self._advance(motion_now_s)

        self._active = _Segment(
            kind="drive",
            start_s=motion_now_s,
            duration_s=max(1e-6, duration_s),
            total_drive_mm=distance_mm,
        )

        print(
            f"[CMD_MOTION][BEGIN_DRIVE] d={distance_mm:.1f} "
            f"t={duration_s:.3f} now={motion_now_s:.3f} pos_valid={self._position_valid} "
            f"heading_valid={self._heading_valid} heading={self._heading}"
        )

    def begin_rotate(self, *, angle_deg: float, duration_s: float, now_s: float):
        motion_now_s = self._motion_time(now_s)
        self._advance(motion_now_s)

        if abs(angle_deg) < MIN_EFFECTIVE_ROTATE_DEG:
            print(
                f"[CMD_MOTION][BEGIN_ROTATE] SUPPRESSED a={angle_deg:.1f} "
                f"(threshold={MIN_EFFECTIVE_ROTATE_DEG:.1f})"
            )
            return

        self._active = _Segment(
            kind="rotate",
            start_s=motion_now_s,
            duration_s=max(1e-6, duration_s),
            total_rotate_deg=angle_deg,
        )

        print(
            f"[CMD_MOTION][BEGIN_ROTATE] a={angle_deg:.1f} "
            f"t={duration_s:.3f} now={motion_now_s:.3f} pos_valid={self._position_valid} "
            f"heading_valid={self._heading_valid} heading={self._heading}"
        )

    def observe_velocity(
            self,
            *,
            linear_x_mps: float,
            angular_z_rps: float,
            now_s: float,
    ) -> None:
        now_s = float(now_s)

        # Complete any outstanding timed-command propagation.
        self._advance(now_s)

        # Integrate the velocity which was active up to this instant.
        self._advance_velocity(now_s)

        self._active = None

        self._velocity_linear_x_mps = float(
            linear_x_mps
        )
        self._velocity_angular_z_rps = float(
            angular_z_rps
        )
        self._velocity_last_s = now_s

    # --------------------------------------------------
    # Core propagation
    # --------------------------------------------------

    def _advance(self, now_s: float):
        if self._active is None:
            return

        if not self._position_valid:
            return

        seg = self._active

        progress = max(
            0.0,
            min(1.0, (now_s - seg.start_s) / seg.duration_s),
        )

        target_drive = seg.total_drive_mm * progress
        target_rotate = seg.total_rotate_deg * progress

        delta_drive = target_drive - seg.applied_drive_mm
        delta_rotate = target_rotate - seg.applied_rotate_deg

        seg.applied_drive_mm = target_drive
        seg.applied_rotate_deg = target_rotate

        # Apply segment-specific dead_reckoning
        if seg.kind == "rotate":
            if self._heading is not None and abs(delta_rotate) > 0.0:
                self._heading = self._wrap(
                    self._heading + math.radians(delta_rotate)
                )
                self._rotation_since_reseed_deg += abs(delta_rotate)

        elif seg.kind == "drive":
            if self._heading is not None and abs(delta_drive) > 0.0:
                h = self._heading + DRIVE_HEADING_OFFSET_RAD
                self._x += delta_drive * math.cos(h)
                self._y += DRIVE_Y_SIGN * delta_drive * math.sin(h)
                self._distance_since_reseed_mm += abs(delta_drive)

        if progress >= 1.0:
            self._active = None

        self._apply_validity_limits()

        print(
            f"[CMD_MOTION][ADVANCE] kind={seg.kind} progress={progress:.2f} "
            f"dx={delta_drive:.1f} drot={delta_rotate:.1f} "
            f"x={self._x:.1f} y={self._y:.1f} hdg={self._heading}"
        )

    def _advance_velocity(
        self,
        now_s: float,
    ) -> None:
        if self._velocity_last_s is None:
            return

        now_s = float(now_s)

        dt_s = max(
            0.0,
            now_s - self._velocity_last_s,
        )

        self._velocity_last_s = now_s

        if dt_s <= 0.0:
            return

        vx_mps = self._velocity_linear_x_mps
        wz_rps = self._velocity_angular_z_rps

        if (
            abs(vx_mps) <= 1e-9
            and abs(wz_rps) <= 1e-9
        ):
            return

        delta_heading_rad = (
            wz_rps * dt_s
        )

        delta_distance_mm = (
            vx_mps * dt_s * 1000.0
        )

        # ----------------------------------------------
        # Heading propagation
        # ----------------------------------------------

        if (
            self._heading_valid
            and self._heading is not None
        ):
            old_heading = self._heading

            new_heading = self._wrap(
                old_heading
                + delta_heading_rad
            )

            # Midpoint heading gives a much better SE(2)
            # approximation when vx and wz are both non-zero.
            midpoint_heading = self._wrap(
                old_heading
                + 0.5 * delta_heading_rad
            )

            self._heading = new_heading

            self._rotation_since_reseed_deg += abs(
                math.degrees(
                    delta_heading_rad
                )
            )

        else:
            midpoint_heading = None

        # ----------------------------------------------
        # Position propagation
        # ----------------------------------------------

        if abs(delta_distance_mm) > 0.0:

            if (
                self._position_valid
                and midpoint_heading is not None
            ):
                h = (
                    midpoint_heading
                    + DRIVE_HEADING_OFFSET_RAD
                )

                self._x += (
                    delta_distance_mm
                    * math.cos(h)
                )

                self._y += (
                    DRIVE_Y_SIGN
                    * delta_distance_mm
                    * math.sin(h)
                )

                self._distance_since_reseed_mm += abs(
                    delta_distance_mm
                )

            elif self._position_valid:
                # We cannot propagate x/y through translation
                # without a trustworthy heading.
                self._position_valid = False

        self._apply_validity_limits()

    def _apply_validity_limits(
        self,
    ) -> None:
        """
        Temporary binary uncertainty model.

        Later this becomes covariance propagation.
        """

        if (
            self._rotation_since_reseed_deg
            >= DEAD_RECKONING_MAX_ROTATION_DEG
        ):
            if self._heading_valid:
                print(
                    "[CMD_MOTION][INVALID] "
                    "heading dead-reckoning limit exceeded "
                    f"rotation="
                    f"{self._rotation_since_reseed_deg:.1f}deg"
                )

            self._heading_valid = False

        if (
            self._distance_since_reseed_mm
            >= DEAD_RECKONING_MAX_DISTANCE_MM
            or
            self._rotation_since_reseed_deg
            >= DEAD_RECKONING_MAX_ROTATION_DEG
        ):
            if self._position_valid:
                print(
                    "[CMD_MOTION][INVALID] "
                    "position dead-reckoning limit exceeded "
                    f"distance="
                    f"{self._distance_since_reseed_mm:.0f}mm "
                    f"rotation="
                    f"{self._rotation_since_reseed_deg:.1f}deg"
                )

            self._position_valid = False

    # --------------------------------------------------
    # Output
    # --------------------------------------------------

    def get_observation(self, now_s: float) -> PoseObservation | None:
        motion_now_s = self._motion_time(now_s)
        self._advance(now_s)

        # Continuous vx/wz propagation is advanced only by
        # observe_velocity(), which uses the canonical motion clock.

        if (
                not self._position_valid
                and not self._heading_valid
        ):
            return None

        age_s = max(0.0, motion_now_s - self._last_reseed_s)

        confidence = max(
            0.0,
            0.5
            - 0.0002 * self._distance_since_reseed_mm
            - 0.002 * self._rotation_since_reseed_deg
            - 0.02 * age_s,
        )

        print(
            f"[CMD_MOTION][OBS] pos_valid={self._position_valid} "
            f"heading_valid={self._heading_valid} x={self._x:.1f} y={self._y:.1f} "
            f"heading={self._heading} active={None if self._active is None else self._active.kind}"
        )

        return PoseObservation(
            x=self._x,
            y=self._y,
            heading=self._heading,
            position_valid=self._position_valid,
            heading_valid=self._heading_valid,
            confidence=confidence,
            source=self.name,
            timestamp=now_s,
            is_absolute=False,
            diagnostics={
                "distance_since_reseed_mm": self._distance_since_reseed_mm,
                "rotation_since_reseed_deg": self._rotation_since_reseed_deg,
                "age_s": age_s,
                "active": None if self._active is None else self._active.kind,
            },
        )

    # --------------------------------------------------

    @staticmethod
    def _wrap(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi