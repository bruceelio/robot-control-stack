# localisation/providers/motion/commanded_motion.py

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from localisation.providers.base import PoseProvider, PoseObservation

MIN_EFFECTIVE_ROTATE_DEG = 7.5

# Motion-frame calibration for synthetic pose propagation
ROTATE_SIGN = -1.0          # set to -1.0 if commanded +rotate is physically opposite
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

        self._x = 0.0
        self._y = 0.0
        self._heading = None

        self._position_valid = False
        self._heading_valid = False

        self._last_reseed_s = 0.0

        self._distance_since_reseed_mm = 0.0
        self._rotation_since_reseed_deg = 0.0

        self._active: Optional[_Segment] = None

    # --------------------------------------------------
    # Lifecycle
    # --------------------------------------------------

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

        self._last_reseed_s = pose.timestamp

        self._distance_since_reseed_mm = 0.0
        self._rotation_since_reseed_deg = 0.0

        self._active = None

        print(
            f"[CMD_MOTION][RESEED] pos_valid={pose.position_valid} "
            f"x={pose.x:.1f} y={pose.y:.1f} "
            f"hdg_valid={pose.heading_valid} heading={pose.heading}"
        )

    def invalidate(self) -> None:
        self._heading_valid = False
        self._active = None

    # --------------------------------------------------
    # Motion input
    # --------------------------------------------------

    def begin_drive(self, *, distance_mm: float, duration_s: float, now_s: float):
        self._advance(now_s)

        self._active = _Segment(
            kind="drive",
            start_s=now_s,
            duration_s=max(1e-6, duration_s),
            total_drive_mm=distance_mm,
        )

        print(
            f"[CMD_MOTION][BEGIN_DRIVE] d={distance_mm:.1f} "
            f"t={duration_s:.3f} now={now_s:.3f} pos_valid={self._position_valid} "
            f"heading_valid={self._heading_valid} heading={self._heading}"
        )

    def begin_rotate(self, *, angle_deg: float, duration_s: float, now_s: float):
        self._advance(now_s)

        if abs(angle_deg) < MIN_EFFECTIVE_ROTATE_DEG:
            print(
                f"[CMD_MOTION][BEGIN_ROTATE] SUPPRESSED a={angle_deg:.1f} "
                f"(threshold={MIN_EFFECTIVE_ROTATE_DEG:.1f})"
            )
            return

        self._active = _Segment(
            kind="rotate",
            start_s=now_s,
            duration_s=max(1e-6, duration_s),
            total_rotate_deg=angle_deg,
        )

        print(
            f"[CMD_MOTION][BEGIN_ROTATE] a={angle_deg:.1f} "
            f"t={duration_s:.3f} now={now_s:.3f} pos_valid={self._position_valid} "
            f"heading_valid={self._heading_valid} heading={self._heading}"
        )

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

        # Apply segment-specific motion
        if seg.kind == "rotate":
            if self._heading is not None and abs(delta_rotate) > 0.0:
                self._heading = self._wrap(
                    self._heading + ROTATE_SIGN * math.radians(delta_rotate)
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

        print(
            f"[CMD_MOTION][ADVANCE] kind={seg.kind} progress={progress:.2f} "
            f"dx={delta_drive:.1f} drot={delta_rotate:.1f} "
            f"x={self._x:.1f} y={self._y:.1f} hdg={self._heading}"
        )

    # --------------------------------------------------
    # Output
    # --------------------------------------------------

    def get_observation(self, now_s: float) -> PoseObservation | None:
        self._advance(now_s)

        if not self._position_valid:
            return None

        age_s = max(0.0, now_s - self._last_reseed_s)

        confidence = max(
            0.0,
            0.5
            - 0.0002 * self._distance_since_reseed_mm
            - 0.002 * self._rotation_since_reseed_deg
            - 0.02 * age_s,
        )

        if confidence <= 0.05:
            quality = "bad"
        elif confidence <= 0.25:
            quality = "poor"
        else:
            quality = "good"

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
            quality=quality,
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