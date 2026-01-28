# behaviors/global_search.py

import time

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus

from skills.navigation.search_rotate import SearchRotate
from skills.perception.select_target import SelectTarget


class GlobalSearchStub(Behavior):
    """
    Stub global search behavior.

    Goal:
      - Try to find *any* target of the desired kind while running a scan pattern.
      - If found -> SUCCEEDED (caller should re-enter SELECT/LOCK pipeline).
      - If not found by timeout -> FAILED.

    This is intentionally simple so we can finalize the failure ladder now.
    """

    def __init__(self):
        super().__init__()
        self.config = None
        self.kind = None
        self.exclude_ids = set()

        self._deadline_s = None
        self.found_target = None

        self._select_skill = None
        self._search_rotate_skill = None

    def start(self, *, config, kind, exclude_ids=None, motion_backend=None, **_):
        self.config = config
        self.kind = kind
        self.exclude_ids = set(exclude_ids) if exclude_ids else set()

        # Use recover_max_sweep_deg as a reasonable "global" scan sweep; keep a hard timeout.
        timeout_s = float(getattr(self.config, "recover_global_search_timeout_s", 8.0))
        self._deadline_s = time.time() + timeout_s

        self.found_target = None

        self._select_skill = SelectTarget(
            kind=self.kind,
            max_age_s=self.config.vision_loss_timeout_s,
            log_every_s=1.0,
            label="GLOBAL_SEARCH][SELECT",
        )
        self._select_skill.start(seed_target=None, exclude_ids=self.exclude_ids)

        self._search_rotate_skill = SearchRotate(
            kinds=[self.kind],
            step_deg=float(getattr(self.config, "recover_step_deg", 15.0)),
            max_deg=float(getattr(self.config, "recover_max_sweep_deg", 180.0)),
            timeout_s=3.0,
            max_age_s=self.config.vision_loss_timeout_s,
            label="GLOBAL_SEARCH][SCAN",
        )
        if motion_backend is not None:
            self._search_rotate_skill.start(motion_backend=motion_backend)

        self.status = BehaviorStatus.RUNNING
        return self.status

    def update(self, *, perception, motion_backend, **_):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        if time.time() > self._deadline_s:
            print("[GLOBAL_SEARCH] timeout -> FAILED")
            self.status = BehaviorStatus.FAILED
            return self.status

        # 1) Try select (fast path if something appears)
        st = self._select_skill.update(perception=perception)
        if st == PrimitiveStatus.SUCCEEDED:
            self.found_target = self._select_skill.selected_target
            if self.found_target is not None:
                print(
                    f"[GLOBAL_SEARCH] found id={self.found_target.get('id', 'REL')} "
                    f"dist={self.found_target.get('distance', -1):.0f} "
                    f"bearing={self.found_target.get('bearing', 0.0):.1f}"
                )
                self.status = BehaviorStatus.SUCCEEDED
                return self.status

        # 2) Otherwise keep scanning
        if self._search_rotate_skill is None:
            self._search_rotate_skill = SearchRotate(
                kinds=[self.kind],
                step_deg=float(getattr(self.config, "recover_step_deg", 15.0)),
                max_deg=float(getattr(self.config, "recover_max_sweep_deg", 180.0)),
                timeout_s=3.0,
                max_age_s=self.config.vision_loss_timeout_s,
                label="GLOBAL_SEARCH][SCAN",
            )
            self._search_rotate_skill.start(motion_backend=motion_backend)

        sr = self._search_rotate_skill.update(motion_backend=motion_backend, perception=perception)
        if sr == PrimitiveStatus.FAILED:
            # Restart scan pattern
            self._search_rotate_skill.start(motion_backend=motion_backend)

        return self.status

    def stop(self):
        if self._select_skill is not None:
            self._select_skill.stop()
        if self._search_rotate_skill is not None:
            self._search_rotate_skill.stop()
