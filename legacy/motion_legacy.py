from primitives.base import Primitive, PrimitiveStatus


class Drive(Primitive):
    def __init__(self, *, distance_mm):
        super().__init__()
        self.distance_mm = distance_mm
        self._started = False

    def start(self, *, motion_backend):
        motion_backend.drive(distance_mm=self.distance_mm)
        self._started = True

    def update(self, *, motion_backend):
        if not self._started:
            return PrimitiveStatus.FAILED

        if motion_backend.is_busy():
            return PrimitiveStatus.RUNNING

        return PrimitiveStatus.SUCCEEDED


class Rotate(Primitive):
    def __init__(self, *, angle_deg: float):
        super().__init__()
        self.angle_deg = angle_deg
        self._started = False

    def start(self, *, motion_backend):
        motion_backend.rotate(self.angle_deg)
        self._started = True

    def update(self, *, motion_backend):
        if not self._started:
            return PrimitiveStatus.FAILED

        if motion_backend.is_busy():
            return PrimitiveStatus.RUNNING

        return PrimitiveStatus.SUCCEEDED

    def stop(self, *, motion_backend):
        motion_backend.stop()


class Stop(Primitive):
    def start(self, *, motion_backend):
        motion_backend.stop()

    def update(self, **kwargs):
        return PrimitiveStatus.SUCCEEDED
