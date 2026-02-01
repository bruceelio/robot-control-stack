# scripted/programs/script_basic_grab.py

from __future__ import annotations

import time

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus

from primitives.motion import Drive, Rotate
from primitives.manipulation import LiftUp, LiftDown, Grab, Release


class ScriptBasicGrab(Behavior):
    """
    List-driven scripted program with per-step parameter overrides.

    - Edit SEQUENCE (order)
    - Optionally edit STEP_PARAMS (per-step params)
    - Defaults come from DEFAULT_PARAMS per operation type
    - Timeouts come from STEP_TIMEOUTS_S per operation type
    - Any failure/timeout -> FAILED (caller exits to autonomous)
    """

    # -------------------------
    # Edit this: the script order
    # -------------------------
    SEQUENCE = [
        "DRIVE01",
        "ROTATE01",
        "LIFT_UP01",
        "DRIVE02",
        "LIFT_DOWN01",
        "GRAB01",
        "LIFT_UP02",
        "DRIVE03",
        "RELEASE01",
    ]

    # -------------------------
    # Defaults per operation type (used if not overridden per-step)
    # -------------------------
    DEFAULT_PARAMS = {
        "DRIVE": {"distance_mm": 300.0},
        "ROTATE": {"angle_deg": 90.0},
    }

    # -------------------------
    # Per-step overrides (only put entries you want to override)
    #
    # Examples:
    #   "DRIVE02": {"distance_mm": 150.0}
    #   "DRIVE03": {"distance_mm": -200.0}   # reverse
    #   "ROTATE01": {"angle_deg": -45.0}
    # -------------------------
    STEP_PARAMS = {
        "DRIVE01": {"distance_mm": 300.0},
        "ROTATE01": {"angle_deg": 30.0},
        "DRIVE02": {"distance_mm": 600.0},
        "DRIVE03": {"distance_mm": -200.0},
        # You can add more like "ROTATE02": {"angle_deg": -90.0}
    }

    # -------------------------
    # Timeouts per operation type (seconds)
    # -------------------------
    STEP_TIMEOUTS_S = {
        "DRIVE": 2.0,
        "ROTATE": 2.0,
        "LIFT_UP": 2.0,
        "LIFT_DOWN": 2.0,
        "GRAB": 3.0,
        "RELEASE": 2.0,
    }

    def __init__(self):
        super().__init__()
        self.config = None

        self._idx = 0
        self.step = None
        self.active = None
        self._step_started_at_s: float | None = None

    def start(self, *, config=None, **_):
        self.config = config
        self._idx = 0
        self.step = self.SEQUENCE[0] if self.SEQUENCE else "DONE"
        self.active = None
        self._step_started_at_s = None

        self.status = BehaviorStatus.RUNNING
        print(f"[SCRIPT_BASIC_GRAB] start steps={len(self.SEQUENCE)}")
        return self.status

    def update(self, *, motion_backend=None, lvl2=None, **_):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        if self.step == "DONE":
            print("[SCRIPT_BASIC_GRAB] done")
            self.status = BehaviorStatus.SUCCEEDED
            return self.status

        # Create/start primitive for this step
        if self.active is None:
            self._step_started_at_s = time.monotonic()
            op = self._op_from_step(self.step)

            params = self._params_for_step(self.step, op)

            if op == "DRIVE":
                dist = float(params["distance_mm"])
                self.active = Drive(distance_mm=dist)
                self.active.start(motion_backend=motion_backend)

            elif op == "ROTATE":
                ang = float(params["angle_deg"])
                self.active = Rotate(angle_deg=ang)
                self.active.start(motion_backend=motion_backend)

            elif op == "LIFT_UP":
                self.active = LiftUp()
                self.active.start(lvl2=lvl2)

            elif op == "LIFT_DOWN":
                self.active = LiftDown()
                self.active.start(lvl2=lvl2)

            elif op == "GRAB":
                self.active = Grab()
                self.active.start(lvl2=lvl2)

            elif op == "RELEASE":
                self.active = Release()
                self.active.start(lvl2=lvl2)

            else:
                print(f"[SCRIPT_BASIC_GRAB] unknown step={self.step} -> FAILED")
                self.status = BehaviorStatus.FAILED
                return self.status

            timeout = self.STEP_TIMEOUTS_S.get(op)
            extra = ""
            if op in ("DRIVE", "ROTATE"):
                extra = f" params={params}"
            print(f"[SCRIPT_BASIC_GRAB] step={self.step} op={op} timeout={timeout}s{extra}")

        # Timeout check
        op = self._op_from_step(self.step)
        timeout_s = self.STEP_TIMEOUTS_S.get(op)

        if timeout_s is not None and self._step_started_at_s is not None:
            elapsed = time.monotonic() - self._step_started_at_s
            if elapsed > timeout_s:
                print(
                    f"[SCRIPT_BASIC_GRAB] step {self.step} TIMEOUT after {elapsed:.2f}s -> exit to autonomous"
                )
                self._safe_stop_active(motion_backend=motion_backend)
                self.active = None
                self.status = BehaviorStatus.FAILED
                return self.status

        # Update primitive
        if isinstance(self.active, (Drive, Rotate)):
            st = self.active.update(motion_backend=motion_backend)
        else:
            st = self.active.update()

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            print(f"[SCRIPT_BASIC_GRAB] step {self.step} FAILED -> exit to autonomous")
            self.active = None
            self.status = BehaviorStatus.FAILED
            return self.status

        # SUCCEEDED -> next step
        self.active = None
        self._step_started_at_s = None
        self._advance()
        return self.status

    def _advance(self):
        self._idx += 1
        if self._idx >= len(self.SEQUENCE):
            self.step = "DONE"
        else:
            self.step = self.SEQUENCE[self._idx]

    def stop(self, *, motion_backend=None, **_):
        self._safe_stop_active(motion_backend=motion_backend)
        self.active = None
        self._step_started_at_s = None
        self.status = BehaviorStatus.FAILED
        return self.status

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

    @classmethod
    def _params_for_step(cls, step: str, op: str) -> dict:
        """
        Merge DEFAULT_PARAMS[op] with STEP_PARAMS[step] (step wins).
        Returns a new dict.
        """
        base = dict(cls.DEFAULT_PARAMS.get(op, {}))
        override = cls.STEP_PARAMS.get(step, {})
        base.update(override)
        return base

    @staticmethod
    def _op_from_step(step: str) -> str:
        for op in ("DRIVE", "ROTATE", "LIFT_UP", "LIFT_DOWN", "GRAB", "RELEASE"):
            if step.startswith(op):
                return op
        return step
