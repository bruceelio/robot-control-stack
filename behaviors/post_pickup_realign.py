# behaviors/post_pickup_realign.py

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus
from composites.drive_then_rotate import DriveThenRotate


class PostPickupRealign(Behavior):
    """
    Post-pickup cleanup behavior:
    - reverse to clear the marker / centre cluster
    - rotate to re-establish a useful heading
    """

    def __init__(self):
        super().__init__()
        self.config = None
        self.primitive = None

    def start(self, *, config, motion_backend=None, **_):
        print("[POST_PICKUP_REALIGN] start")
        self.config = config
        self.primitive = None
        self.status = BehaviorStatus.RUNNING

    def update(self, *, motion_backend, **_):
        if self.primitive is None:
            # Reverse is a negative drive
            self.primitive = DriveThenRotate(
                distance_mm=-self.config.post_pickup_reverse_mm,
                angle_deg=self.config.post_pickup_rotate_deg,
            )
            self.primitive.start(motion_backend=motion_backend)

        prim_status = self.primitive.update(motion_backend=motion_backend)

        if prim_status == PrimitiveStatus.SUCCEEDED:
            self.status = BehaviorStatus.SUCCEEDED
        elif prim_status == PrimitiveStatus.FAILED:
            self.status = BehaviorStatus.FAILED

        return self.status
