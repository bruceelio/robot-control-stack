# skills/approach_target.py

from primitives.base import Primitive, PrimitiveStatus
from primitives.motion import Drive


class ApproachTarget(Primitive):
    """
    Skill: ApproachTarget
    Responsibility:
      - Execute a single forward (or backward) drive step
      - No perception logic; just motion execution
    Inputs:
      - distance_mm
    Outputs:
      - SUCCEEDED when drive completes
      - FAILED if drive fails
    """

    def __init__(self, *, distance_mm: float):
        super().__init__()
        self.distance_mm = float(distance_mm)
        self._child = None

    def start(self, *, motion_backend, **_):
        self._child = Drive(distance_mm=self.distance_mm)
        self._child.start(motion_backend=motion_backend)

    def update(self, *, motion_backend, **_):
        if self._child is None:
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
