# primitives/motion/rotate.py

from primitives.base import Primitive, PrimitiveStatus
import time


class Rotate(Primitive):
    def __init__(self, *, angle_deg: float):
        super().__init__()
        self.angle_deg = angle_deg
        self._started = False

    def start(self, *, motion_backend):
        angle_deg, duration_s = motion_backend.estimate_rotate_duration(
            angle_deg=self.angle_deg
        )

        if duration_s > 0.0:
            localisation = getattr(motion_backend, "localisation", None)
            now_s = time.time()

            if localisation is not None:
                localisation.begin_commanded_rotate(
                    angle_deg=angle_deg,
                    duration_s=duration_s,
                    now_s=now_s,
                )

            motion_backend.rotate(angle_deg=angle_deg)

        self._started = True

    def update(self, *, motion_backend):
        if not self._started:
            return PrimitiveStatus.FAILED

        if motion_backend.is_busy():
            return PrimitiveStatus.RUNNING

        return PrimitiveStatus.SUCCEEDED

    def stop(self, *, motion_backend):
        motion_backend.stop()