# navigation/rotate_then_drive.py

from primitives.base import Primitive, PrimitiveStatus
from primitives.motion import Rotate, Drive


class RotateThenDrive(Primitive):
    """
    Composite primitive: Rotate an angle, then drive a distance.

    Sequencing logic only. Delegates execution to Rotate and Drive primitives.

    Returns:
      - RUNNING while child primitives are running
      - SUCCEEDED when both complete
      - FAILED if a child fails
    """

    def __init__(self, *, angle_deg: float, distance_mm: float):
        super().__init__()
        self.angle_deg = angle_deg
        self.distance_mm = distance_mm

        self._phase = "ROTATE"
        self._child = None

    def start(self, *, motion_backend, **_):
        self.status = PrimitiveStatus.RUNNING
        self._phase = "ROTATE"
        self._child = None

    def update(self, *, motion_backend, **_):
        if self.status != PrimitiveStatus.RUNNING:
            return self.status

        # -----------------
        # Phase: ROTATE
        # -----------------
        if self._phase == "ROTATE":
            if self._child is None:
                self._child = Rotate(angle_deg=self.angle_deg)
                self._child.start(motion_backend=motion_backend)

            child_status = self._child.update(motion_backend=motion_backend)

            if child_status == PrimitiveStatus.SUCCEEDED:
                self._child = None
                self._phase = "DRIVE"
            elif child_status == PrimitiveStatus.FAILED:
                self.status = PrimitiveStatus.FAILED
                return self.status

        # -----------------
        # Phase: DRIVE
        # -----------------
        if self._phase == "DRIVE":
            if self._child is None:
                self._child = Drive(distance_mm=self.distance_mm)
                self._child.start(motion_backend=motion_backend)

            child_status = self._child.update(motion_backend=motion_backend)

            if child_status == PrimitiveStatus.SUCCEEDED:
                self.status = PrimitiveStatus.SUCCEEDED
            elif child_status == PrimitiveStatus.FAILED:
                self.status = PrimitiveStatus.FAILED

        return self.status
