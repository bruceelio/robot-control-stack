# skills/manipulation/grasp_object.py

from primitives.base import PrimitiveStatus
from primitives.manipulation import Grab, LiftUp, LiftDown, Release


class GraspObject:
    """
    Composite primitive which performs the *exact same* grab sequence that
    previously lived inside AcquireObject:

        LiftUp -> LiftDown -> Grab -> LiftUp

    This is intentionally non-vision: it assumes ApproachTarget has already
    positioned the robot at the commit point.

    IMPORTANT:
    - Implements the Primitive interface (returns PrimitiveStatus), because
      AcquireObject._grab() is a primitive-runner.
    """

    def __init__(self):
        self.io = None
        self.config = None
        self.lvl2 = None

        self._actions = None
        self._index = 0
        self._active = None

    def start(self, *, lvl2, config, io=None, **_):
        self.lvl2 = lvl2
        self.config = config
        self.io = io

        # Keep historical sequence exactly:
        self._actions = [Release(), LiftUp(), LiftDown(), Grab(), LiftUp()]
        self._index = 0
        self._active = None

        print("[GRASP_OBJECT] start")
        return PrimitiveStatus.RUNNING

    def _start_next(self) -> bool:
        if self._actions is None or self._index >= len(self._actions):
            return False

        self._active = self._actions[self._index]
        name = self._active.__class__.__name__
        print(f"[GRASP_OBJECT] starting {name}")

        # Match AcquireObject's convention for starting primitives
        self._active.start(lvl2=self.lvl2, config=self.config, io=self.io)
        return True

    def update(self, *, lvl2=None, **_):
        if self._actions is None:
            return PrimitiveStatus.FAILED

        # Kick off next primitive as needed
        if self._active is None:
            if not self._start_next():
                print("[GRASP_OBJECT] complete")
                return PrimitiveStatus.SUCCEEDED

        status = self._active.update()

        if status == PrimitiveStatus.RUNNING:
            return PrimitiveStatus.RUNNING

        name = self._active.__class__.__name__

        if status == PrimitiveStatus.FAILED:
            print(f"[GRASP_OBJECT] {name} FAILED")
            return PrimitiveStatus.FAILED

        # SUCCEEDED
        print(f"[GRASP_OBJECT] {name} complete")
        self._active = None
        self._index += 1
        return PrimitiveStatus.RUNNING
