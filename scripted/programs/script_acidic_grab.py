# scripted/programs/script_acidic_grab.py

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus

from primitives.motion import Drive, Rotate
from primitives.manipulation import LiftUp, LiftDown, Grab, Release


class ScriptAcidicGrab(Behavior):
    """
    Second scripted program (acidic variant).
    Same mini-controller pattern:
      - step success advances
      - any failure returns FAILED (controller exits to autonomous)
    """

    def __init__(self):
        super().__init__()
        self.config = None
        self.step = None
        self.active = None

    def start(self, *, config=None, **_):
        self.config = config
        self.step = "DRIVE"
        self.active = None
        self.status = BehaviorStatus.RUNNING
        print("[SCRIPT_ACIDIC_GRAB] start")
        return self.status

    def update(self, *, motion_backend=None, lvl2=None, **_):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        if self.step == "DONE":
            print("[SCRIPT_ACIDIC_GRAB] done")
            self.status = BehaviorStatus.SUCCEEDED
            return self.status

        if self.active is None:
            if self.step == "DRIVE":
                # slightly different to BASIC so it's obvious in logs
                self.active = Drive(distance_mm=250.0)
                self.active.start(motion_backend=motion_backend)

            elif self.step == "ROTATE":
                self.active = Rotate(angle_deg=-90.0)
                self.active.start(motion_backend=motion_backend)

            elif self.step == "LIFT_UP":
                self.active = LiftUp()
                self.active.start(lvl2=lvl2)

            elif self.step == "LIFT_DOWN":
                self.active = LiftDown()
                self.active.start(lvl2=lvl2)

            elif self.step == "GRAB":
                self.active = Grab()
                self.active.start(lvl2=lvl2)

            elif self.step == "RELEASE":
                self.active = Release()
                self.active.start(lvl2=lvl2)

            else:
                print(f"[SCRIPT_ACIDIC_GRAB] unknown step={self.step} -> FAILED")
                self.status = BehaviorStatus.FAILED
                return self.status

            print(f"[SCRIPT_ACIDIC_GRAB] step={self.step}")

        if isinstance(self.active, (Drive, Rotate)):
            st = self.active.update(motion_backend=motion_backend)
        else:
            st = self.active.update()

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            print(f"[SCRIPT_ACIDIC_GRAB] step {self.step} FAILED -> exit to autonomous")
            self.active = None
            self.status = BehaviorStatus.FAILED
            return self.status

        self.active = None
        self._advance()
        return self.status

    def _advance(self):
        if self.step == "DRIVE":
            self.step = "ROTATE"
        elif self.step == "ROTATE":
            self.step = "LIFT_UP"
        elif self.step == "LIFT_UP":
            self.step = "LIFT_DOWN"
        elif self.step == "LIFT_DOWN":
            self.step = "GRAB"
        elif self.step == "GRAB":
            self.step = "RELEASE"
        elif self.step == "RELEASE":
            self.step = "DONE"
        else:
            self.step = "DONE"

    def stop(self, *, motion_backend=None, **_):
        try:
            if self.active is not None:
                try:
                    self.active.stop(motion_backend=motion_backend)
                except TypeError:
                    self.active.stop()
        except Exception:
            pass
        self.active = None
        self.status = BehaviorStatus.FAILED
        return self.status
