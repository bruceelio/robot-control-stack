# navigation/dog_leg_side_step.py

from __future__ import annotations

import math
from dataclasses import dataclass

from primitives.base import Primitive, PrimitiveStatus
from primitives.motion import Rotate, Drive


@dataclass(frozen=True)
class DogLegPlan:
    rotate1_deg: float
    drive_mm: float
    rotate2_deg: float


def compute_dog_leg_plan(
    *,
    distance_mm: float,
    bearing_deg: float,
    min_sidestep_mm: float = 50.0,
    min_drive_mm: float = 5.0,
    max_drive_mm: float = 2500.0,
) -> DogLegPlan:
    """
    Compute rotate–drive–rotate-back dog-leg to remove lateral offset to marker.
    Bearing is direction-to-marker (deg). Distance is mm.
    """

    r = float(distance_mm)
    b_deg = float(bearing_deg)
    b = math.radians(b_deg)

    # Decompose marker position in robot frame
    x = r * math.cos(b)  # forward component
    y = r * math.sin(b)  # lateral component (+left, -right)

    # Choose what bearing you want AFTER the sidestep (so Align doesn't need a huge turn)
    target_bearing_deg = 0.0  # tune: 3-7 degrees
    bt = math.radians(target_bearing_deg)

    # Maintain the same sign (reduce toward +/- target rather than crossing over)
    bt = bt if y >= 0 else -bt

    # Lateral translation needed so that atan2(y - s, x) == bt  =>  s = y - x*tan(bt)
    s = y - x * math.tan(bt)

    if abs(s) < min_sidestep_mm:
        return DogLegPlan(0.0, 0.0, 0.0)

    rotate1 = 90.0 if s > 0 else -90.0
    drive = max(min_drive_mm, min(max_drive_mm, abs(s))) * 2.2
    rotate2 = -rotate1

    return DogLegPlan(rotate1, drive, rotate2)


class DogLegSideStep(Primitive):
    """
    Primitive: rotate ±90°, drive sideways distance, rotate back.

    After completion, caller resumes normal ALIGN/APPROACH logic.
    """

    def __init__(self, *, distance_mm: float, bearing_deg: float):
        super().__init__()
        self.distance_mm = float(distance_mm)
        self.bearing_deg = float(bearing_deg)

        self.plan: DogLegPlan | None = None
        self.phase = 0
        self.child = None
        self.status = PrimitiveStatus.RUNNING

    # ---------- safe stop helper ----------
    def _safe_stop(self, prim, *, motion_backend=None):
        if prim is None:
            return
        try:
            prim.stop(motion_backend=motion_backend) if motion_backend is not None else prim.stop()
        except TypeError:
            try:
                prim.stop()
            except Exception:
                pass
        except Exception:
            pass

    # ---------- lifecycle ----------
    def start(self, *, motion_backend):
        self.plan = compute_dog_leg_plan(
            distance_mm=self.distance_mm,
            bearing_deg=self.bearing_deg,
        )

        if self.plan.drive_mm <= 0.0:
            self.status = PrimitiveStatus.SUCCEEDED
            return

        self.phase = 1
        self.child = Rotate(angle_deg=self.plan.rotate1_deg)
        self.child.start(motion_backend=motion_backend)
        self.status = PrimitiveStatus.RUNNING

    def update(self, *, motion_backend):
        if self.status != PrimitiveStatus.RUNNING:
            return self.status

        if self.child is None:
            self.status = PrimitiveStatus.FAILED
            return self.status

        st = self.child.update(motion_backend=motion_backend)

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            self.status = PrimitiveStatus.FAILED
            return self.status

        # ----- phase transitions -----
        if self.phase == 1:
            self.phase = 2
            self.child = Drive(distance_mm=self.plan.drive_mm)
            self.child.start(motion_backend=motion_backend)
            return self.status

        if self.phase == 2:
            self.phase = 3
            self.child = Rotate(angle_deg=self.plan.rotate2_deg)
            self.child.start(motion_backend=motion_backend)
            return self.status

        if self.phase == 3:
            self.child = None
            self.status = PrimitiveStatus.SUCCEEDED
            return self.status

        self.status = PrimitiveStatus.FAILED
        return self.status

    def stop(self, *, motion_backend=None):
        self._safe_stop(self.child, motion_backend=motion_backend)
        self.child = None
