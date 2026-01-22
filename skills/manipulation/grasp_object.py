# skills/manipulation/grasp_object.py

from skills.base import Skill, SkillStatus
from primitives.base import PrimitiveStatus
from primitives.manipulation import Grab, LiftUp, LiftDown


class GraspObject(Skill):
    """
    Execute the standard SR1 pickup sequence:
      LiftUp -> LiftDown -> Vacuum/Grab -> LiftUp

    This is intentionally "blind" / non-vision: it assumes the robot is already
    positioned at the final commit point by ApproachTarget.
    """

    def __init__(self):
        super().__init__()
        self.io = None
        self.config = None

        self._actions = None
        self._index = 0
        self._active_primitive = None

    def start(self, *, io, config, **_):
        self.io = io
        self.config = config

        self._actions = [LiftUp(), LiftDown(), Grab(), LiftUp()]
        self._index = 0
        self._active_primitive = None

        print("[GRASP_OBJECT] start")
        return SkillStatus.RUNNING

    def _start_next(self):
        if self._index >= len(self._actions):
            return False

        prim = self._actions[self._index]
        self._active_primitive = prim

        name = prim.__class__.__name__
        print(f"[GRASP_OBJECT] starting {name}")

        # All these primitives in your existing code accept io/config.
        prim.start(io=self.io, config=self.config)
        return True

    def update(self, **_):
        if self._actions is None:
            return SkillStatus.FAILED

        # Start the next primitive if needed
        if self._active_primitive is None:
            if not self._start_next():
                print("[GRASP_OBJECT] complete")
                return SkillStatus.SUCCEEDED

        status = self._active_primitive.update()

        if status == PrimitiveStatus.RUNNING:
            return SkillStatus.RUNNING

        # Primitive finished (success or fail)
        prim_name = self._active_primitive.__class__.__name__

        if status == PrimitiveStatus.FAILED:
            print(f"[GRASP_OBJECT] {prim_name} FAILED")
            self._active_primitive = None
            return SkillStatus.FAILED

        # SUCCEEDED
        print(f"[GRASP_OBJECT] {prim_name} complete")
        self._active_primitive = None
        self._index += 1
        return SkillStatus.RUNNING
