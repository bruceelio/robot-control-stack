# primitives/motion/stop.py

from primitives.base import Primitive, PrimitiveStatus


class Stop(Primitive):
    def start(self, *, motion_backend):
        motion_backend.stop()

    def update(self, **kwargs):
        return PrimitiveStatus.SUCCEEDED