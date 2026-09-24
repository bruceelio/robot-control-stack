# behaviors/post_dropoff_realign.py

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus
from primitives.motion import Drive, Rotate
from primitives.manipulation.liftdown import LiftDown


class PostDropoffRealign(Behavior):
    """
    Post-dropoff cleanup behavior:

        reverse
        -> lower lift
        -> rotate to re-establish a useful heading
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
        print("[POST_DROPOFF_REALIGN] start")

        self.config = config
        self.step = "REVERSE"
        self.active = None
        self.status = BehaviorStatus.RUNNING

    def update(
        self,
        *,
        motion_backend,
        lvl2,
        **_,
    ):
        if self.active is None:

            if self.step == "REVERSE":
                print("[POST_DROPOFF_REALIGN] reverse")

                self.active = Drive(
                    distance_mm=-self.config.post_dropoff_reverse_mm
                )

                try:
                    self.active.start(
                        motion_backend=motion_backend
                    )
                except RuntimeError as exc:
                    print(
                        "[POST_DROPOFF_REALIGN] "
                        f"reverse ended early: {exc} "
                        "-> continuing to lift down"
                    )

                    self.active = None
                    self.step = "LIFT_DOWN"
                    return self.status

            elif self.step == "LIFT_DOWN":
                print("[POST_DROPOFF_REALIGN] lift down")

                self.active = LiftDown(
                    settle_time=0.0
                )

                self.active.start(
                    lvl2=lvl2
                )

            elif self.step == "ROTATE":
                print("[POST_DROPOFF_REALIGN] rotate")

                self.active = Rotate(
                    angle_deg=self.config.post_dropoff_rotate_deg
                )

                self.active.start(
                    motion_backend=motion_backend
                )

            elif self.step == "DONE":
                print("[POST_DROPOFF_REALIGN] complete")

                self.status = BehaviorStatus.SUCCEEDED
                return self.status

        if self.step == "LIFT_DOWN":
            prim_status = self.active.update()

        else:
            prim_status = self.active.update(
                motion_backend=motion_backend
            )

        if prim_status == PrimitiveStatus.RUNNING:
            return self.status

        if prim_status == PrimitiveStatus.FAILED:

            if self.step == "REVERSE":
                print(
                    "[POST_DROPOFF_REALIGN] "
                    "reverse FAILED -> continuing to lift down"
                )

                self.active = None
                self.step = "LIFT_DOWN"
                return self.status

            print(
                f"[POST_DROPOFF_REALIGN] "
                f"{self.step} FAILED"
            )

            self.status = BehaviorStatus.FAILED
            return self.status

        if self.step == "REVERSE":
            print("[POST_DROPOFF_REALIGN] reverse complete")
            self.step = "LIFT_DOWN"

        elif self.step == "LIFT_DOWN":
            print("[POST_DROPOFF_REALIGN] lift down complete")
            self.step = "ROTATE"

        elif self.step == "ROTATE":
            print("[POST_DROPOFF_REALIGN] rotate complete")
            self.step = "DONE"

        self.active = None
        return self.status