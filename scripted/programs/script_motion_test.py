# scripted/programs/script_motion_test.py

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus
from primitives.motion import Drive, Rotate


class ScriptMotionTest(Behavior):

    def __init__(self):
        super().__init__()
        self.step = None
        self.active = None

    def start(self, **_):
        self.step = "DRIVE"
        self.active = None
        self.status = BehaviorStatus.RUNNING

        print("[SCRIPT_MOTION_TEST] start")
        return self.status

    def update(self, *, motion_backend=None, **_):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        if self.step == "DONE":
            print("[SCRIPT_MOTION_TEST] done")
            self.status = BehaviorStatus.SUCCEEDED
            return self.status

        if self.active is None:

            if self.step == "DRIVE":
                print("[SCRIPT_MOTION_TEST] drive 1000 mm")
                self.active = Drive(distance_mm=1000)
                self.active.start(motion_backend=motion_backend)

            elif self.step == "ROTATE":
                print("[SCRIPT_MOTION_TEST] rotate 180 deg")
                self.active = Rotate(angle_deg=180)
                self.active.start(motion_backend=motion_backend)

        status = self.active.update(
            motion_backend=motion_backend
        )

        if status == PrimitiveStatus.RUNNING:
            return self.status

        if status == PrimitiveStatus.FAILED:
            print(f"[SCRIPT_MOTION_TEST] {self.step} FAILED")
            self.status = BehaviorStatus.FAILED
            return self.status

        if self.step == "DRIVE":
            self.step = "ROTATE"
        else:
            self.step = "DONE"

        self.active = None
        return self.status