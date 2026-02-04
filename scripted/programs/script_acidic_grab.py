# scripted/programs/script_acidic_grab.py

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus

from primitives.motion import Drive, Rotate
from primitives.manipulation import LiftUp, LiftDown, Grab, Release

from hw_io.base import IOMap



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

    def _get_io_from_context(self, *, motion_backend=None, lvl2=None, io=None):
        if io is not None:
            return io
        if motion_backend is not None:
            io2 = getattr(motion_backend, "io", None)
            if io2 is not None:
                return io2
            lvl2b = getattr(motion_backend, "lvl2", None)
            if lvl2b is not None:
                return getattr(lvl2b, "io", None)
        if lvl2 is not None:
            return getattr(lvl2, "io", None)
        return None

    def start(self, *, config=None, **_):
        self.config = config
        self.step = "DRIVE01"
        self.active = None
        self.status = BehaviorStatus.RUNNING
        print("[SCRIPT_ACIDIC_GRAB] start")
        return self.status

    def update(self, *, motion_backend=None, lvl2=None, io: IOMap | None = None, **_):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        if self.step == "DONE":
            print("[SCRIPT_ACIDIC_GRAB] done")
            self.status = BehaviorStatus.SUCCEEDED
            return self.status

        if self.active is None:
            if self.step == "DRIVE01":
                # away from wall
                self.active = Drive(distance_mm=350)
                self.active.start(motion_backend=motion_backend)

            elif self.step == "DRIVE02":
                # halfway to target
                self.active = Drive(distance_mm=600)
                self.active.start(motion_backend=motion_backend)

            elif self.step == "DRIVE03":
                # commit to target
                self.active = Drive(distance_mm=800)
                self.active.start(motion_backend=motion_backend)

            elif self.step == "DRIVE04":
                # backup after pickup
                self.active = Drive(distance_mm=-200)
                self.active.start(motion_backend=motion_backend)

            elif self.step == "DRIVE05":
                # put marker in home zone
                self.active = Drive(distance_mm=1300)
                self.active.start(motion_backend=motion_backend)

            elif self.step == "DRIVE06":
                # backup after delivering marker
                self.active = Drive(distance_mm=-300)
                self.active.start(motion_backend=motion_backend)

            elif self.step == "ROTATE01":
                # rotate towards target
                self.active = Rotate(angle_deg=45)
                self.active.start(motion_backend=motion_backend)

            elif self.step == "ROTATE02":
                # rotate towards home
                self.active = Rotate(angle_deg=60)
                self.active.start(motion_backend=motion_backend)

            elif self.step == "ROTATE03":
                # Rotate in small increments until front ultrasonic becomes valid AND < 1200mm.
                io2 = self._get_io_from_context(motion_backend=motion_backend, lvl2=lvl2, io=io)
                front = io2.ultrasonics().get("front") if io2 else None

                # If we have a valid reading and we're close enough, finish this step.
                if front is not None and front != 0 and front < 1200:
                    print(f"[SCRIPT_ACIDIC_GRAB] ROTATE03: front ultrasonic = {front} (<1200) -> step done")
                    self._advance()
                    return self.status

                # Otherwise rotate a little and re-check next tick.
                if front is None or front == 0:
                    print(f"[SCRIPT_ACIDIC_GRAB] ROTATE03: invalid ultrasonic reading ({front}) -> keep rotating")
                else:
                    print(f"[SCRIPT_ACIDIC_GRAB] ROTATE03: front ultrasonic = {front} (>=1200) -> keep rotating")

                self.active = Rotate(angle_deg=5)
                self.active.start(motion_backend=motion_backend)

            elif self.step == "ROTATE04":
                # last bit towards home
                self.active = Rotate(angle_deg=3)
                self.active.start(motion_backend=motion_backend)

            elif self.step == "LIFT_UP01":
                self.active = LiftUp()
                self.active.start(lvl2=lvl2)

            elif self.step == "LIFT_UP02":
                self.active = LiftUp()
                self.active.start(lvl2=lvl2)

            elif self.step == "LIFT_DOWN01":
                self.active = LiftDown()
                self.active.start(lvl2=lvl2)

            elif self.step == "GRAB01":
                self.active = Grab()
                self.active.start(lvl2=lvl2)

            elif self.step == "LIFT_UP03":
                self.active = LiftUp()
                self.active.start(lvl2=lvl2)

            elif self.step == "RELEASE01":
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

        # Primitive succeeded
        finished = self.active
        self.active = None

        io2 = self._get_io_from_context(motion_backend=motion_backend, lvl2=lvl2, io=io)

        if isinstance(finished, (Drive, Rotate)) and io2 is not None:
            try:
                front = io2.ultrasonics().get("front")
                print(
                    f"[SCRIPT_ACIDIC_GRAB] {finished.__class__.__name__} complete, "
                    f"front ultrasonic = {front}"
                )
            except Exception as e:
                print(f"[SCRIPT_ACIDIC_GRAB] ultrasonic read failed: {e}")

        self._advance()
        return self.status

    def _advance(self):
        if self.step == "DRIVE01":
            self.step = "ROTATE01"
        elif self.step == "ROTATE01":
            self.step = "DRIVE02"
        elif self.step == "DRIVE02":
            self.step = "LIFT_UP01"
        elif self.step == "LIFT_UP01":
            self.step = "DRIVE03"
        elif self.step == "DRIVE03":
            self.step = "LIFT_DOWN01"
        elif self.step == "LIFT_DOWN01":
            self.step = "GRAB01"
        elif self.step == "GRAB01":
            self.step = "LIFT_UP02"
        elif self.step == "LIFT_UP02":
            self.step = "DRIVE04"
        elif self.step == "DRIVE04":
            self.step = "ROTATE02"
        elif self.step == "ROTATE02":
            self.step = "ROTATE03"
        elif self.step == "ROTATE03":
            self.step = "ROTATE04"
        elif self.step == "ROTATE04":
            self.step = "DRIVE05"

        elif self.step == "DRIVE05":
            self.step = "RELEASE01"
        elif self.step == "RELEASE01":
            self.step = "DRIVE06"
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
