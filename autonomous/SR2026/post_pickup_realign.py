# autonomous/SR2026/post_pickup_realign.py

from autonomous.SR2026.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus
from primitives.motion import Rotate



class PostPickupRealign(Behavior):
    """
    Rotate towards the estimated homeward direction.

    PickupObject has already reversed, verified the grip,
    and moved the lift to its carrying position.
    """

    def __init__(self):
        super().__init__()

        self.config = None
        self.step = None
        self.active = None

    def start(
        self,
        *,
        config,
        motion_backend=None,
        **_,
    ):
        print("[POST_PICKUP_REALIGN] start")

        self.config = config
        self.step = "ROTATE"
        self.active = None
        self.status = BehaviorStatus.RUNNING

    def update(self, *, motion_backend, **_):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        if self.active is None:
            print("[POST_PICKUP_REALIGN] rotate")

            self.active = Rotate(
                angle_deg=self.config.post_pickup_rotate_deg
            )

            self.active.start(
                motion_backend=motion_backend
            )

        st = self.active.update(
            motion_backend=motion_backend
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            print("[POST_PICKUP_REALIGN] rotate FAILED")

            self.status = BehaviorStatus.FAILED
            return self.status

        print("[POST_PICKUP_REALIGN] rotate complete")

        self.step = "DONE"
        self.status = BehaviorStatus.SUCCEEDED

        return self.status
