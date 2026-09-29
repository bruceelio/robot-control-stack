# skills/navigation/search_for_return_guide.py

from __future__ import annotations

from typing import Optional, Sequence

from primitives.base import Primitive, PrimitiveStatus
from skills.navigation.search_rotate import SearchRotate


class SearchForReturnGuide(Primitive):
    """
    Recover Stage 2 return-route perception.

    Responsibility:
      - Define what counts as a valid return-route guide.
      - Run an appropriate search pattern.
      - Succeed as soon as a return-route guide is visible.
      - Fail if the search pattern is exhausted.

    Version 1:
      - Delegates the physical search pattern to SearchRotate.
      - Does not use localisation to steer the search.

    Future:
      - localisation may be used opportunistically to estimate where
        a useful return-route guide should be and choose a smarter
        initial search direction before falling back to SearchRotate.

    Localisation is therefore accepted by start() and update() now,
    even though Version 1 intentionally does not depend on it.
    """

    def __init__(
        self,
        *,
        config,
        guide_routes: Sequence[dict],
        required_guide_id: Optional[int] = None,
    ):
        super().__init__()

        if not guide_routes:
            raise ValueError("guide_routes must not be empty")

        self.config = config

        # Preserve route structure for future localisation-aware
        # recovery. Version 1 only needs the flattened guide ID set.
        self.guide_routes = []

        for route in guide_routes:
            guide_ids = tuple(
                int(tag_id)
                for tag_id in route["guide_ids"]
            )

            if not guide_ids:
                raise ValueError(
                    "guide route must contain guide_ids"
                )

            normalised_route = dict(route)
            normalised_route["guide_ids"] = guide_ids
            self.guide_routes.append(normalised_route)

        self.guide_ids = frozenset(
            tag_id
            for route in self.guide_routes
            for tag_id in route["guide_ids"]
        )

        self.required_guide_id = (
            None
            if required_guide_id is None
            else int(required_guide_id)
        )

        if (
            self.required_guide_id is not None
            and self.required_guide_id not in self.guide_ids
        ):
            raise ValueError(
                "required_guide_id must belong to a return route"
            )

        self._search: Optional[SearchRotate] = None

        # Result / diagnostic information.
        self.found_guide = None
        self.found_guide_id: Optional[int] = None

        self.status = PrimitiveStatus.RUNNING

    def start(
        self,
        *,
        motion_backend,
        localisation=None,
        **_,
    ):
        """
        Start return-guide recovery.

        localisation is intentionally unused in Version 1. It is part
        of the interface now so a future version can use an
        opportunistic arena pose without changing callers.
        """
        self.found_guide = None
        self.found_guide_id = None

        # Version 1 deliberately reuses the existing bounded-search
        # parameters rather than introducing return-guide-specific
        # configuration before testing shows that it is necessary.
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
            label="RETURN_GUIDE_SEARCH",
        )

        st = self._search.start(
            motion_backend=motion_backend,
        )

        if st == PrimitiveStatus.FAILED:
            self.status = PrimitiveStatus.FAILED
            return self.status

        self.status = PrimitiveStatus.RUNNING

        if self.required_guide_id is None:
            target = "any"
        else:
            target = str(self.required_guide_id)

        print(
            "[SEARCH_FOR_RETURN_GUIDE] start "
            f"required_guide={target} "
            f"guide_ids={sorted(self.guide_ids)}"
        )

        return self.status

    def update(
        self,
        *,
        motion_backend,
        arena_observations,
        localisation=None,
        **_,
    ) -> PrimitiveStatus:
        """
        Continue return-guide recovery.

        localisation is intentionally unused in Version 1. A future
        implementation may inspect it on every update so that a pose
        which becomes available during the search can immediately
        improve the search strategy.
        """
        if self.status != PrimitiveStatus.RUNNING:
            return self.status

        if self._search is None:
            self.status = PrimitiveStatus.FAILED
            return self.status

        found = self._find_visible_guide(
            arena_observations=arena_observations,
        )

        st = self._search.update(
            motion_backend=motion_backend,
            found_item=found,
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.SUCCEEDED:
            self.found_guide = self._search.found_item

            try:
                self.found_guide_id = int(
                    self.found_guide["id"]
                )
            except (TypeError, ValueError, KeyError):
                self.found_guide_id = None

            print(
                "[SEARCH_FOR_RETURN_GUIDE] "
                f"guide={self.found_guide_id} "
                "visible -> SUCCEEDED"
            )

            self.status = PrimitiveStatus.SUCCEEDED
            return self.status

        print(
            "[SEARCH_FOR_RETURN_GUIDE] "
            "search exhausted -> FAILED"
        )

        self.status = PrimitiveStatus.FAILED
        return self.status

    def _find_visible_guide(
        self,
        *,
        arena_observations,
    ):
        """
        Return one currently-visible arena observation belonging to
        either valid return route.

        This proves only that Stage 2 return navigation can see a
        usable guide again. Route selection remains the responsibility
        of ReturnToBaseServo.
        """
        for observation in arena_observations or ():
            try:
                tag_id = int(
                    observation.get("id", -1)
                )
            except (AttributeError, TypeError, ValueError):
                continue

            if self.required_guide_id is not None:
                if tag_id == self.required_guide_id:
                    return observation
                continue

            if tag_id in self.guide_ids:
                return observation

        return None

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

        if self.status == PrimitiveStatus.RUNNING:
            self.status = PrimitiveStatus.FAILED

        return self.status
