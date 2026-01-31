# behaviors/global_recovery_stub.py

"""Aspirational GlobalRecovery stub.

Current behavior (intentionally narrow):
  - If localisation pose is missing/invalid, attempt to recover it by running
    RecoverLocalisation once.

RecoverLostTarget can escalate to this stub when local rungs fail, without
taking on the broader responsibilities of a future global recovery ladder.
"""

from __future__ import annotations

from typing import Optional

from behaviors.base import Behavior, BehaviorStatus
from behaviors.recover_localisation import RecoverLocalisation


class GlobalRecoveryStub(Behavior):
    """Placeholder behavior for the future GlobalRecovery ladder."""

    def __init__(self):
        super().__init__()
        self._recover_localisation: Optional[RecoverLocalisation] = None

    def start(self, *, config):
        super().start(config=config)
        self._recover_localisation = None

    def update(self, *, perception, localisation, motion_backend) -> BehaviorStatus:
        # Nothing to do if we already have a pose.
        if localisation is not None and localisation.has_pose():
            self.status = BehaviorStatus.SUCCEEDED
            return self.status

        if self._recover_localisation is None:
            self._recover_localisation = RecoverLocalisation()
            self._recover_localisation.start(config=self.config)

        st = self._recover_localisation.update(
            perception=perception,
            localisation=localisation,
            motion_backend=motion_backend,
        )

        if st == BehaviorStatus.SUCCEEDED and localisation is not None and localisation.has_pose():
            self.status = BehaviorStatus.SUCCEEDED
            return self.status

        if st == BehaviorStatus.FAILED:
            self.status = BehaviorStatus.FAILED
            return self.status

        self.status = BehaviorStatus.RUNNING
        return self.status

    def stop(self, *, motion_backend=None):
        if self._recover_localisation:
            self._recover_localisation.stop(motion_backend=motion_backend)
        super().stop(motion_backend=motion_backend)
