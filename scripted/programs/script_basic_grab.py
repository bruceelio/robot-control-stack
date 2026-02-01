import time

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus

from primitives.motion import Drive, Rotate
from primitives.manipulation import LiftUp, LiftDown, Grab, Release

from scripted.programs.script_basic_grab_steps import list_of_steps


class ScriptBasicGrab(Behavior):
    def __init__(self):
        super().__init__()
        self.config = None

        self.step = None
        self.active = None
        self._step_started_at_s = None

        # per-step timeout (you can tune step-by-step)
        self.timeouts_s = {
            "DRIVE01": 5.0,
            "ROTATE01": 3.0,
            "LIFT_UP01": 2.0,
            "LIFT_DOWN01": 2.0,
            "GRAB01": 3.0,
            "RELEASE01": 2.0,
            # future examples:
            # "ALIGN01": 3.0,
            # "APPROACH01": 10.0,
        }

    def start(self, *, config=None, **_):
        self.config = config
        self.step = "START"
        idx = 0
        self.active = None
        self._step_started_at_s = None
        self.status = BehaviorStatus.RUNNING
        print("[SCRIPT_BASIC_GRAB] start")
        return self.status

    def update(self, *, motion_backend=None, lvl2=None, perception=None, localisation=None, **_):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        if self.step == "DONE":
            print("[SCRIPT_BASIC_GRAB] done")
            self.status = BehaviorStatus.SUCCEEDED
            return self.status

        # (1) Start a new step if needed
        if self.active is None:
            self._step_started_at_s = time.monotonic()
            self.active = self._make_active_for_step(
                step=self.step,
                motion_backend=motion_backend,
                lvl2=lvl2,
                perception=perception,
                localisation=localisation,
            )
            if self.active is None:
                print(f"[SCRIPT_BASIC_GRAB] unknown step={self.step} -> FAILED")
                self.status = BehaviorStatus.FAILED
                return self.status
            print(f"[SCRIPT_BASIC_GRAB] step={self.step} timeout={self.timeouts_s.get(self.step)}s")

        # (2) Timeout check
        timeout = self.timeouts_s.get(self.step)
        if timeout is not None and self._step_started_at_s is not None:
            elapsed = time.monotonic() - self._step_started_at_s
            if elapsed > timeout:
                print(f"[SCRIPT_BASIC_GRAB] step {self.step} TIMEOUT after {elapsed:.2f}s -> FAILED")
                self._safe_stop_active(motion_backend=motion_backend)
                self.active = None
                self.status = BehaviorStatus.FAILED
                return self.status

        # (3) Tick active and interpret status
        st = self._tick_active(
            active=self.active,
            motion_backend=motion_backend,
            lvl2=lvl2,
            perception=perception,
            localisation=localisation,
        )

        if st == "RUNNING":
            return self.status

        if st == "FAILED":
            print(f"[SCRIPT_BASIC_GRAB] step {self.step} FAILED -> exit to autonomous")
            self.active = None
            self.status = BehaviorStatus.FAILED
            return self.status

        # SUCCEEDED
        self.active = None
        self._step_started_at_s = None
        self._advance()
        return self.status

    # -------------------------
    # Step logic (explicit)
    # -------------------------

    def _make_active_for_step(self, *, step, motion_backend, lvl2, perception, localisation):
        # Primitives
        if step == "DRIVE01":
            p = Drive(distance_mm=200.0)
            p.start(motion_backend=motion_backend)
            return p

        if step == "ROTATE01":
            p = Rotate(angle_deg=27.0)
            p.start(motion_backend=motion_backend)
            return p

        if step == "LIFT_UP01":
            p = LiftUp()
            p.start(lvl2=lvl2)
            return p

        if step == "LIFT_DOWN01":
            p = LiftDown()
            p.start(lvl2=lvl2)
            return p

        if step == "GRAB01":
            p = Grab()
            p.start(lvl2=lvl2)
            return p

        if step == "RELEASE01":
            p = Release()
            p.start(lvl2=lvl2)
            return p

        if step == "START":
            p = Drive(distance_mm=1.0)
            p.start(motion_backend=motion_backend)
            return p

        # Future: drop in skills/behaviors here
        # if step == "ALIGN01":
        #     b = AlignToTarget(...)
        #     b.start(config=self.config, motion_backend=motion_backend)
        #     return b

        return None

    def _advance(self):
        '''
        if self.step == "DRIVE01":
            self.step = "ROTATE01"
        elif self.step == "ROTATE01":
            self.step = "LIFT_UP01"
        elif self.step == "LIFT_UP01":
            self.step = "LIFT_DOWN01"
        elif self.step == "LIFT_DOWN01":
            self.step = "GRAB01"
        elif self.step == "GRAB01":
            self.step = "RELEASE01"
        elif self.step == "RELEASE01":
            self.step = "DONE"
        else:
            self.step = "DONE"
        '''

        global idx
        if self.step == "START":
            idx = -1
        idx = idx+1
        self.step = list_of_steps[idx]



    # -------------------------
    # Active ticking (primitive or behavior)
    # -------------------------

    def _tick_active(self, *, active, motion_backend, lvl2, perception, localisation):
        # Primitive path (your primitives return PrimitiveStatus)
        if isinstance(active, (Drive, Rotate, LiftUp, LiftDown, Grab, Release)):
            if isinstance(active, (Drive, Rotate)):
                pst = active.update(motion_backend=motion_backend)
            else:
                pst = active.update()

            if pst == PrimitiveStatus.RUNNING:
                return "RUNNING"
            if pst == PrimitiveStatus.FAILED:
                return "FAILED"
            return "SUCCEEDED"

        # Behavior path (future): returns BehaviorStatus
        # bst = active.update(
        #     motion_backend=motion_backend,
        #     lvl2=lvl2,
        #     perception=perception,
        #     localisation=localisation,
        # )
        # if bst == BehaviorStatus.RUNNING: return "RUNNING"
        # if bst == BehaviorStatus.FAILED: return "FAILED"
        # return "SUCCEEDED"

        # Unknown active type
        return "FAILED"

    def _safe_stop_active(self, *, motion_backend=None):
        try:
            if self.active is None:
                return
            try:
                self.active.stop(motion_backend=motion_backend)
            except TypeError:
                self.active.stop()
        except Exception:
            pass
