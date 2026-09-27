# behaviors/global_object_search.py

from __future__ import annotations

import time

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus

from skills.navigation.search_rotate import SearchRotate
from skills.perception.select_target_utils import get_closest_target


class GlobalObjectSearch(Behavior):
    """
    Search globally for at least one eligible game object.

    Responsibility:
      - Define what counts as an eligible object.
      - Run an appropriate search pattern.
      - Succeed as soon as at least one eligible object is visible.
      - Fail if the search pattern is exhausted.

    This behavior does NOT select which object to pursue.

    Autonomous flow:

        PrepareSearch
            -> GlobalObjectSearch
                -> SelectTarget

    Search policy:

      First acquisition:
        required_kind is None
        -> acidic OR basic is eligible

      Later acquisitions:
        required_kind is the delivered object kind
        -> only that kind is eligible

      In all cases:
        exclude_ids are ignored.

    Future versions may use localisation and arena-map knowledge
    to choose better search viewpoints. The current implementation
    simply delegates the physical search pattern to SearchRotate.
    """

    def __init__(self):
        super().__init__()

        self.config = None

        self.required_kind: str | None = None
        self.exclude_ids: set[int] = set()

        self._search: SearchRotate | None = None

        # Result / diagnostic information.
        self.eligible_target_seen = False
        self.found_object = None

    def start(
        self,
        *,
        config,
        required_kind=None,
        exclude_ids=None,
        motion_backend,
        localisation=None,
        **_,
    ):
        self.config = config

        self.required_kind = (
            str(required_kind)
            if required_kind is not None
            else None
        )

        self.exclude_ids = (
            set(int(x) for x in exclude_ids)
            if exclude_ids
            else set()
        )

        self.eligible_target_seen = False
        self.found_object = None

        # Current Stage 1 search strategy.
        #
        # GlobalObjectSearch owns the strategy decision;
        # SearchRotate only performs the requested motion pattern.
        self._search = SearchRotate(
            step_deg=float(
                getattr(
                    self.config,
                    "recover_step_deg",
                    15.0,
                )
            ),
            max_deg=float(
                getattr(
                    self.config,
                    "recover_max_sweep_deg",
                    180.0,
                )
            ),
            timeout_s=float(
                getattr(
                    self.config,
                    "global_object_search_timeout_s",
                    8.0,
                )
            ),
            settle_s=float(
                getattr(
                    self.config,
                    "recover_settle_time",
                    0.5,
                )
            ),
            label="GLOBAL_OBJECT_SEARCH",
        )

        self._search.start(
            motion_backend=motion_backend,
        )

        self.status = BehaviorStatus.RUNNING

        mode = (
            "any object kind"
            if self.required_kind is None
            else f"required kind={self.required_kind}"
        )

        print(
            "[GLOBAL_OBJECT_SEARCH] start "
            f"{mode} "
            f"exclude_ids={sorted(self.exclude_ids)}"
        )

        return self.status

    def update(
        self,
        *,
        perception,
        motion_backend,
        localisation=None,
        **_,
    ):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        # ---------------------------------
        # Determine whether an eligible
        # object is visible right now.
        # ---------------------------------
        found = self._find_eligible_object(
            perception=perception,
            now=time.time(),
        )

        # ---------------------------------
        # Let SearchRotate own only the
        # physical search pattern.
        # ---------------------------------
        st = self._search.update(
            motion_backend=motion_backend,
            found_item=found,
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.SUCCEEDED:
            self.eligible_target_seen = True

            # Retain for diagnostics/logging only.
            # SelectTarget will make the actual target choice.
            self.found_object = self._search.found_item

            if self.found_object is not None:
                print(
                    "[GLOBAL_OBJECT_SEARCH] "
                    "eligible object seen "
                    f"id={self.found_object.get('id', 'N/A')} "
                    f"kind={self.found_object.get('kind', 'N/A')}"
                )
            else:
                print(
                    "[GLOBAL_OBJECT_SEARCH] "
                    "eligible object seen"
                )

            self.status = BehaviorStatus.SUCCEEDED
            return self.status

        print(
            "[GLOBAL_OBJECT_SEARCH] "
            "search exhausted -> FAILED"
        )

        self.status = BehaviorStatus.FAILED
        return self.status

    def _find_eligible_object(
        self,
        *,
        perception,
        now: float,
    ):
        """
        Return one object proving that the GlobalObjectSearch
        success condition has been met.

        This is NOT target selection.

        SelectTarget subsequently examines perception and chooses
        the actual target according to strategy.
        """

        if self.required_kind is None:
            kinds = ("acidic", "basic")
        else:
            kinds = (self.required_kind,)

        found = None

        for kind in kinds:
            candidate = get_closest_target(
                perception,
                kind,
                now=now,
                max_age_s=float(
                    self.config.visible_max_age_s
                ),
                exclude_ids=self.exclude_ids,
            )

            if candidate is None:
                continue

            # We only need one eligible object to prove that
            # SelectTarget has something useful to work with.
            #
            # Choosing the closest here is NOT strategy; it is
            # merely deterministic if both kinds are visible.
            if (
                found is None
                or float(candidate["distance"])
                < float(found["distance"])
            ):
                found = candidate

        return found

    def stop(
        self,
        *,
        motion_backend=None,
        **_,
    ):
        if self._search is not None:
            self._search.stop(
                motion_backend=motion_backend,
            )

        self._search = None

        if self.status == BehaviorStatus.RUNNING:
            self.status = BehaviorStatus.FAILED

        return self.status

# Temporary compatibility for dormant legacy code in ApproachObject.
# Remove when old internal recovery/search code is cleaned out.
GlobalSearchStub = GlobalObjectSearch