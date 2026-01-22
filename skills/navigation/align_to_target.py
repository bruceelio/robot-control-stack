# skills/align_to_target.py

from primitives.base import Primitive, PrimitiveStatus
from primitives.motion import Rotate


class AlignToTarget(Primitive):
    """
    Skill: AlignToTarget

    Responsibility:
      - Orient robot to face target (bearing -> heading correction)
    Inputs:
      - bearing_deg
      - tolerance_deg (deadband)
      - max_rotate_deg (clamp)
    Outputs:
      - SUCCEEDED when within tolerance
      - FAILED if rotate fails
      - RUNNING while rotating
    """

    def __init__(self, *, bearing_deg: float, tolerance_deg: float, max_rotate_deg: float):
        super().__init__()
        self.bearing_deg = float(bearing_deg)
        self.tolerance_deg = float(tolerance_deg)
        self.max_rotate_deg = float(max_rotate_deg)

        self._child = None
        self._done = False

    def start(self, *, motion_backend, **_):
        # If already aligned, succeed immediately (no motion).
        if abs(self.bearing_deg) < self.tolerance_deg:
            self._done = True
            return

        angle = max(-self.max_rotate_deg, min(self.max_rotate_deg, self.bearing_deg))
        self._child = Rotate(angle_deg=angle)
        self._child.start(motion_backend=motion_backend)

    def update(self, *, motion_backend, **_):
        if self._done:
            return PrimitiveStatus.SUCCEEDED

        if self._child is None:
            # Defensive: start() should have created a child unless already done
            return PrimitiveStatus.FAILED

        st = self._child.update(motion_backend=motion_backend)
        if st == PrimitiveStatus.SUCCEEDED:
            return PrimitiveStatus.SUCCEEDED
        if st == PrimitiveStatus.FAILED:
            return PrimitiveStatus.FAILED
        return PrimitiveStatus.RUNNING

    def stop(self):
        if self._child is not None:
            self._child.stop()
