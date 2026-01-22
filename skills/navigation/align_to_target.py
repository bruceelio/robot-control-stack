# skills/navigation/align_to_target.py

"""
AlignToTarget skill (control-loop program).

Stateful wrapper around alignment utilities:
- RUNNING while rotating
- SUCCEEDED when within tolerance or rotation completes
- FAILED if rotate fails

This is the "main program" for alignment in your architecture.
"""

from __future__ import annotations

from primitives.base import PrimitiveStatus
from primitives.motion import Rotate

from skills.navigation.align_to_target_utils import (
    is_aligned,
    clamp_rotation_deg,
)


class AlignToTarget:
    """
    Program skill: align robot to face a target by rotating in place.

    Inputs:
      - bearing_deg: signed bearing to target in degrees (+right/-left or vice versa per your convention)
      - tolerance_deg: deadband for "close enough"
      - max_rotate_deg: clamp for safety/comfort

    Contract:
      - start() arms the plan (may immediately succeed if already aligned)
      - update() advances the plan
      - PrimitiveStatus.SUCCEEDED when aligned / rotation complete
      - PrimitiveStatus.FAILED if rotation fails
      - PrimitiveStatus.RUNNING otherwise
    """

    def __init__(
        self,
        *,
        bearing_deg: float,
        tolerance_deg: float,
        max_rotate_deg: float,
        label: str = "ALIGN_TO_TARGET",
    ):
        self.bearing_deg = float(bearing_deg)
        self.tolerance_deg = float(tolerance_deg)
        self.max_rotate_deg = float(max_rotate_deg)
        self.label = label

        self._child: Rotate | None = None
        self._done = False
        self._started = False

    def start(self, *, motion_backend, **_):
        self._child = None
        self._done = False
        self._started = True

        # If already aligned, succeed immediately (no motion).
        if is_aligned(self.bearing_deg, tolerance_deg=self.tolerance_deg):
            self._done = True
            return PrimitiveStatus.SUCCEEDED

        angle = clamp_rotation_deg(self.bearing_deg, max_rotate_deg=self.max_rotate_deg)
        self._child = Rotate(angle_deg=angle)
        self._child.start(motion_backend=motion_backend)
        return PrimitiveStatus.RUNNING

    def update(self, *, motion_backend, **_):
        if not self._started:
            # Defensive: require start() before update()
            return PrimitiveStatus.FAILED

        if self._done:
            return PrimitiveStatus.SUCCEEDED

        if self._child is None:
            # Defensive: start() should have created a child unless already aligned
            return PrimitiveStatus.FAILED

        st = self._child.update(motion_backend=motion_backend)
        if st == PrimitiveStatus.SUCCEEDED:
            self._done = True
            return PrimitiveStatus.SUCCEEDED
        if st == PrimitiveStatus.FAILED:
            return PrimitiveStatus.FAILED
        return PrimitiveStatus.RUNNING

    def stop(self):
        if self._child is not None:
            self._child.stop()
        self._child = None
        self._done = False
        self._started = False
