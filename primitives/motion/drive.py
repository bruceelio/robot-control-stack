# primitives/motion/drive.py

from primitives.base import Primitive, PrimitiveStatus


class Drive(Primitive):
    def __init__(self, *, distance_mm):
        super().__init__()
        self.distance_mm = distance_mm
        self._started = False

    def start(self, *, motion_backend):
        distance_mm, duration_s = motion_backend.estimate_drive_duration(
            distance_mm=self.distance_mm
        )

        if duration_s > 0.0:
            localisation = getattr(motion_backend, "localisation", None)
            now_s = getattr(motion_backend, "now_s", None)

            if localisation is not None and now_s is not None:
                localisation.begin_commanded_drive(
                    distance_mm=distance_mm,
                    duration_s=duration_s,
                    now_s=now_s,
                )

            motion_backend.drive(distance_mm=distance_mm)

        self._started = True

    def update(self, *, motion_backend):
        if not self._started:
            return PrimitiveStatus.FAILED

        if motion_backend.is_busy():
            return PrimitiveStatus.RUNNING

        return PrimitiveStatus.SUCCEEDED