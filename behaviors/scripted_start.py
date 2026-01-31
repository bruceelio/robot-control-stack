# behaviors/scripted_start.py

from behaviors.base import Behavior, BehaviorStatus
from config.strategy import STARTUP_SCRIPT, StartupScript
from scripted.registry import SCRIPT_REGISTRY


class ScriptedStart(Behavior):
    """
    Runs one selected scripted program, then returns terminal status.
    Controller policy decides: on SUCCEEDED or FAILED -> exit to autonomous.
    """

    def __init__(self):
        super().__init__()
        self._program = None

    def start(self, *, config, **_):
        super().start(config=config)
        self._program = None
        self.status = BehaviorStatus.RUNNING
        print(f"[SCRIPTED_START] start script={STARTUP_SCRIPT.name}")
        return self.status

    def update(self, *, motion_backend, lvl2, **_):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        if STARTUP_SCRIPT == StartupScript.NONE:
            self.status = BehaviorStatus.SUCCEEDED
            return self.status

        if self._program is None:
            program_cls = SCRIPT_REGISTRY.get(STARTUP_SCRIPT)
            if program_cls is None:
                print(f"[SCRIPTED_START] unknown script={STARTUP_SCRIPT} -> FAILED")
                self.status = BehaviorStatus.FAILED
                return self.status

            self._program = program_cls()
            self._program.start(config=self.config)

        st = self._program.update(motion_backend=motion_backend, lvl2=lvl2)

        # Program returns BehaviorStatus; pass it through
        if st != BehaviorStatus.RUNNING:
            self.status = st
        return self.status

    def stop(self, *, motion_backend=None, **_):
        if self._program is not None:
            self._program.stop(motion_backend=motion_backend)
        self._program = None
        self.status = BehaviorStatus.FAILED
        return self.status
