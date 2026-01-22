# skills/manipulation/verify_grip.py

import time
from skills.base import Skill, SkillStatus


class VerifyGrip(Skill):
    """
    First-pass placeholder.

    Later versions should check vacuum sensor / current / pressure decay, etc.
    For now: wait a short settle time and declare success.
    """

    def __init__(self):
        super().__init__()
        self._done_at = None

    def start(self, *, settle_s=0.2, **_):
        self._done_at = time.time() + float(settle_s)
        print("[VERIFY_GRIP] start")
        return SkillStatus.RUNNING

    def update(self, **_):
        if self._done_at is None:
            return SkillStatus.FAILED

        if time.time() >= self._done_at:
            print("[VERIFY_GRIP] ok (stub)")
            return SkillStatus.SUCCEEDED

        return SkillStatus.RUNNING
