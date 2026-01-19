# behaviors/post_dropoff_realign.py

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus
from composites.drive_then_rotate import DriveThenRotate


class PostDropoffRealign(Behavior):
    """
    Post-dropoff cleanup behavior:
    - reverse to clear the drop zone / wall
    - rotate to re-establish a useful heading (typically to re-localise on wall markers)
    """

    def __init__(self):
        super().__init__()
        self.config = None
        self.primitive = None

    def start(self, *, config, motion_backend=None, **_):
        print("[POST_DROPOFF_REALIGN] start")
        self.config = config
        self.primitive = None
        self.status = BehaviorStatus.RUNNING

    def update(self, *, motion_backend, **_):
        if self.primitive is None:
            self.primitive = DriveThenRotate(
                distance_mm=-self.config.post_dropoff_reverse_mm,
                angle_deg=self.config.post_dropoff_rotate_deg,
            )
            self.primitive.start(motion_backend=motion_backend)

        prim_status = self.primitive.update(motion_backend=motion_backend)

        if prim_status == PrimitiveStatus.SUCCEEDED:
            self.status = BehaviorStatus.SUCCEEDED
        elif prim_status == PrimitiveStatus.FAILED:
            self.status = BehaviorStatus.FAILED

        return self.status