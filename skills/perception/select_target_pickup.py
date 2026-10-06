# skills/perception/select_target_pickup.py

"""
SelectTarget skill.

Chooses one target from the currently visible eligible objects.

Selection policy:

    1. required kind
       - hard filter
       - used after the first successful delivery

    2. preferred elevation
       - preference only
       - if no visible target satisfies it, ignore the preference

    3. preferred kind
       - preference only
       - if no remaining target satisfies it, ignore the preference

    4. distance
       - choose the closest remaining target

Typical first acquisition:

    required_kind=None
    preferred_elevation="low" / "high" / None
    preferred_kind="basic" / "acidic" / None

Typical later acquisition:

    required_kind=<delivered kind>
    preferred_elevation=None
    preferred_kind=None

The legacy ``kind=`` argument is retained temporarily for compatibility.
When supplied, it is treated as ``required_kind``.
"""

from __future__ import annotations

from __future__ import annotations

import math
import time
from enum import Enum

from typing import Iterable, Optional

from primitives.base import PrimitiveStatus

from perception import get_visible_targets
from skills.perception.marker_elevation import _marker_elevation

from skills.navigation.approach_feasibility import (
    ApproachFeasibilityEvaluator,
    ApproachFeasibilityMode,
)


class ApproachServoMethod(Enum):
    TARGET_BEARING = "target_bearing"
    POSE_BEARING = "pose_bearing"


TARGET_KINDS = (
    "acidic",
    "basic",
)

TARGET_ELEVATIONS = (
    "high",
    "low",
)


def _safe_int(value) -> Optional[int]:
    if value is None:
        return None

    try:
        return int(value)

    except (TypeError, ValueError):
        return None


def _normalise_kind(value: str | None) -> str | None:
    if value is None:
        return None

    value = str(value).lower()

    if value not in TARGET_KINDS:
        raise ValueError(
            f"invalid target kind: {value!r}"
        )

    return value


def _normalise_elevation(
    value: str | None,
) -> str | None:

    if value is None:
        return None

    value = str(value).lower()

    if value not in TARGET_ELEVATIONS:
        raise ValueError(
            f"invalid target elevation: {value!r}"
        )

    return value


class SelectTarget:
    """
    Select the best currently visible eligible target.

    Outputs after SUCCEEDED:

    selected_target
    selected_target_id
    selected_kind
    selected_elevation
    selected_servo_method

    FAILED is only returned when timeout_s is configured and no
    eligible target becomes available before that timeout.

    Otherwise, absence of a target returns RUNNING.
    """

    def __init__(
        self,
        *,
        max_age_s: float,
        config,
        pose_bearing_allowed: bool = False,

        preferred_kind: str | None = None,
        preferred_elevation: str | None = None,
        required_kind: str | None = None,

        marker_pitch_high_deg: float | None = None,
        marker_pitch_low_deg: float | None = None,

        timeout_s: float | None = None,

        log_every_s: float = 1.0,
        label: str = "SELECT_TARGET",

        exclude_ids: Iterable[int] | None = None,

        # Legacy compatibility.
        kind: str | None = None,
    ):

        # --------------------------------------------------
        # Legacy compatibility
        # --------------------------------------------------

        legacy_kind = _normalise_kind(kind)

        if required_kind is None:
            required_kind = legacy_kind

        elif (
            legacy_kind is not None
            and legacy_kind != required_kind
        ):
            raise ValueError(
                "SelectTarget received conflicting "
                f"kind={legacy_kind!r} and "
                f"required_kind={required_kind!r}"
            )

        # --------------------------------------------------
        # Selection policy
        # --------------------------------------------------

        self.required_kind = _normalise_kind(
            required_kind
        )

        self.preferred_kind = _normalise_kind(
            preferred_kind
        )

        self.preferred_elevation = (
            _normalise_elevation(
                preferred_elevation
            )
        )

        self.max_age_s = float(max_age_s)

        self.config = config

        self.pose_bearing_allowed = bool(
            pose_bearing_allowed
        )

        self._approach_feasibility = (
            ApproachFeasibilityEvaluator(
                config=self.config,
                pose_bearing_allowed=(
                    self.pose_bearing_allowed
                ),
            )
        )

        self.marker_pitch_high_deg = (
            None
            if marker_pitch_high_deg is None
            else float(marker_pitch_high_deg)
        )

        self.marker_pitch_low_deg = (
            None
            if marker_pitch_low_deg is None
            else float(marker_pitch_low_deg)
        )

        if (
            self.preferred_elevation is not None
            and (
                self.marker_pitch_high_deg is None
                or self.marker_pitch_low_deg is None
            )
        ):
            raise ValueError(
                "preferred_elevation requires "
                "marker_pitch_high_deg and "
                "marker_pitch_low_deg"
            )

        self.timeout_s = (
            None
            if timeout_s is None
            else float(timeout_s)
        )

        self.log_every_s = float(log_every_s)
        self.label = str(label)

        # Callers may deliberately pass a shared mutable set.
        self.exclude_ids = exclude_ids

        # Legacy attribute retained for callers which inspect it.
        self.kind = self.required_kind

        # --------------------------------------------------
        # Outputs
        # --------------------------------------------------

        self.selected_target = None
        self.selected_target_id: int | None = None
        self.selected_kind: str | None = None
        self.selected_elevation: str | None = None
        self.selected_servo_method: ApproachServoMethod | None = None

        self.selected_approach_feasibility = None
        self.selected_smooth_selection = None

        # --------------------------------------------------
        # Runtime state
        # --------------------------------------------------

        self._exclude_ids: set[int] = set()

        self._started_s: float | None = None
        self._last_no_target_log: float | None = None

        self._seed_target = None
        self._seed_used = False
        self._seed_rejected_logged = False

    # --------------------------------------------------
    # Lifecycle
    # --------------------------------------------------

    def start(
        self,
        *,
        seed_target=None,
        exclude_ids=None,
        now=None,
        **_,
    ):

        if now is None:
            now = time.time()

        self._started_s = float(now)

        self._exclude_ids = self._normalise_exclude_ids(
            exclude_ids
        )

        self.selected_target = None
        self.selected_target_id = None
        self.selected_kind = None
        self.selected_elevation = None
        self.selected_servo_method = None

        self.selected_approach_feasibility = None
        self.selected_smooth_selection = None

        self._last_no_target_log = None

        self._seed_target = seed_target
        self._seed_used = False
        self._seed_rejected_logged = False

        return PrimitiveStatus.RUNNING

    def update(
        self,
        *,
        perception=None,
        now=None,
        **_,
    ):

        if now is None:
            now = time.time()

        now = float(now)

        # Once selected, remain succeeded.
        if self.selected_target is not None:
            return PrimitiveStatus.SUCCEEDED

        exclude_ids = self._current_exclude_ids()

        # --------------------------------------------------
        # Gather currently visible eligible targets
        # --------------------------------------------------

        candidates = self._gather_candidates(
            perception=perception,
            now=now,
            exclude_ids=exclude_ids,
        )

        # Optional compatibility seed.
        #
        # It joins the candidate set rather than bypassing the
        # selection policy.
        self._add_seed_candidate(
            candidates=candidates,
            exclude_ids=exclude_ids,
        )

        # --------------------------------------------------
        # No eligible targets
        # --------------------------------------------------

        if not candidates:
            return self._no_target_status(now)

        # --------------------------------------------------
        # Rank candidates by policy, then test feasibility
        # --------------------------------------------------
        #
        # required_kind has already been applied as a HARD filter
        # by _gather_candidates().
        #
        # preferred elevation and preferred kind remain SOFT
        # preferences. Candidates are tried in preference order,
        # nearest first within each tier.
        #
        # Smooth is therefore evaluated only for the next candidate
        # we would otherwise select, not for every visible object.

        ranked_candidates = sorted(
            candidates,
            key=self._candidate_preference_key,
        )

        selected_kind = None
        selected_target = None
        selected_elevation = None
        selected_feasibility = None
        selected_servo_method = None

        for (
                candidate_kind,
                candidate_target,
        ) in ranked_candidates:

            candidate_elevation = (
                self._classify_elevation(
                    candidate_target
                )
            )

            feasibility = (
                self._approach_feasibility.evaluate(
                    kind=candidate_kind,
                    target=candidate_target,
                    elevation=candidate_elevation,
                    perception=perception,
                )
            )

            candidate_id = _safe_int(
                candidate_target.get("id")
            )

            if not feasibility.viable:
                print(
                    f"[{self.label}][APPROACH] "
                    f"reject id={candidate_id} "
                    f"kind={candidate_kind} "
                    f"elevation={candidate_elevation} "
                    f"reason={feasibility.reason}"
                )

                continue

            # First viable candidate wins.
            selected_kind = candidate_kind
            selected_target = candidate_target
            selected_elevation = candidate_elevation
            selected_feasibility = feasibility

            if (
                    feasibility.mode
                    == ApproachFeasibilityMode.SMOOTH
            ):
                selected_servo_method = (
                    ApproachServoMethod.POSE_BEARING
                )

            else:
                selected_servo_method = (
                    ApproachServoMethod.TARGET_BEARING
                )

            break

        # --------------------------------------------------
        # Visible candidates exist, but none are currently
        # approachable.
        #
        # This is different from "no target visible".
        # Waiting here would simply rerun the same expensive
        # feasibility checks every control cycle.
        # --------------------------------------------------

        if selected_target is None:
            print(
                f"[{self.label}] "
                "visible candidates exhausted "
                "by approach feasibility "
                "-> FAILED"
            )

            return PrimitiveStatus.FAILED

        # --------------------------------------------------
        # Store outputs
        # --------------------------------------------------

        self.selected_target = selected_target

        self.selected_target_id = _safe_int(
            selected_target.get("id")
        )

        self.selected_kind = selected_kind
        self.selected_elevation = selected_elevation
        self.selected_servo_method = selected_servo_method

        self.selected_approach_feasibility = (
            selected_feasibility
        )

        self.selected_smooth_selection = (
            selected_feasibility.smooth_selection
        )

        tid_display = (
            self.selected_target_id
            if self.selected_target_id is not None
            else "REL"
        )

        distance = float(
            selected_target.get(
                "distance",
                0.0,
            )
        )

        bearing = float(
            selected_target.get(
                "bearing",
                0.0,
            )
        )

        if self.selected_smooth_selection is not None:

            smooth = (
                self.selected_smooth_selection
            )

            approach_text = (
                f"{selected_feasibility.mode.value}"
                f"/{smooth.mode.value}"
                f"/{smooth.relaxation_fraction}"
            )


        else:

            approach_text = (

                f"{selected_feasibility.mode.value}"

                f"/{selected_feasibility.reason}"

            )

        print(
            f"[{self.label}] "
            f"selected id={tid_display} "
            f"kind={self.selected_kind} "
            f"elevation={self.selected_elevation} "
            f"servo={self.selected_servo_method.value} "
            f"approach={approach_text} "
            f"dist={distance:.0f} "
            f"bearing={bearing:.1f}"
        )

        return PrimitiveStatus.SUCCEEDED

    def stop(self, **_):
        pass

    # --------------------------------------------------
    # Candidate ranking
    # --------------------------------------------------

    def _candidate_preference_key(
            self,
            candidate,
    ):
        """
        Order candidates without turning soft preferences
        into hard filters.

        required_kind is handled earlier as a hard filter.

        Ordering:
            1. preferred elevation
            2. preferred kind
            3. nearest distance

        If every candidate in a preferred tier is rejected as
        unapproachable, selection naturally continues to the next
        preference tier.
        """

        candidate_kind, target = candidate

        elevation = self._classify_elevation(
            target
        )

        elevation_penalty = 0

        if (
                self.preferred_elevation is not None
                and elevation != self.preferred_elevation
        ):
            elevation_penalty = 1

        kind_penalty = 0

        if (
                self.preferred_kind is not None
                and candidate_kind != self.preferred_kind
        ):
            kind_penalty = 1

        distance = float(
            target.get(
                "distance",
                math.inf,
            )
        )

        return (
            elevation_penalty,
            kind_penalty,
            distance,
        )

    # --------------------------------------------------
    # Candidate gathering
    # --------------------------------------------------

    def _gather_candidates(
        self,
        *,
        perception,
        now: float,
        exclude_ids: set[int],
    ):
        """
        Gather currently visible candidates after hard filtering.

        required_kind is a HARD filter.

        Before the first successful delivery:

            required_kind = None

        so both acidic and basic targets may be considered.

        After the first successful delivery:

            required_kind = delivered_kind

        so targets of the other kind are excluded here before
        preference ranking or Smooth feasibility evaluation.
        """

        candidates = []

        if self.required_kind is not None:

            candidate_kinds = (
                self.required_kind,
            )

        else:

            candidate_kinds = TARGET_KINDS

        for candidate_kind in candidate_kinds:

            visible = get_visible_targets(
                perception,
                candidate_kind,
                now=now,
                max_age_s=self.max_age_s,
            )

            for target in visible:

                target_id = _safe_int(
                    target.get("id")
                )

                if (
                    target_id is not None
                    and target_id in exclude_ids
                ):
                    continue

                candidates.append(
                    (
                        candidate_kind,
                        target,
                    )
                )

        return candidates


    # --------------------------------------------------
    # Seed compatibility
    # --------------------------------------------------

    def _add_seed_candidate(
        self,
        *,
        candidates,
        exclude_ids: set[int],
    ):

        if (
            self._seed_target is None
            or self._seed_used
        ):
            return

        self._seed_used = True

        seed = self._seed_target

        seed_id = _safe_int(
            seed.get("id")
        )

        if (
            seed_id is not None
            and seed_id in exclude_ids
        ):

            if not self._seed_rejected_logged:

                print(
                    f"[{self.label}] "
                    f"seed id={seed_id} "
                    "is excluded — ignoring seed"
                )

                self._seed_rejected_logged = True

            return

        seed_kind = seed.get("kind")

        if seed_kind is not None:

            try:
                seed_kind = _normalise_kind(
                    seed_kind
                )

            except ValueError:
                seed_kind = None

        # Old callers may provide a seed which does not carry
        # its kind because the SelectTarget instance already
        # represented one particular kind.
        if (
            seed_kind is None
            and self.required_kind is not None
        ):
            seed_kind = self.required_kind

        if seed_kind is None:

            if not self._seed_rejected_logged:

                print(
                    f"[{self.label}] "
                    "seed has no usable kind "
                    "— ignoring seed"
                )

                self._seed_rejected_logged = True

            return

        # Required kind is a hard filter.
        if (
            self.required_kind is not None
            and seed_kind
            != self.required_kind
        ):
            return

        # Do not duplicate an already-visible target.
        for candidate_kind, target in candidates:

            candidate_id = _safe_int(
                target.get("id")
            )

            if (
                seed_id is not None
                and candidate_id == seed_id
                and candidate_kind == seed_kind
            ):
                return

        candidates.append(
            (
                seed_kind,
                seed,
            )
        )

    # --------------------------------------------------
    # Elevation
    # --------------------------------------------------

    def _matches_elevation(
        self,
        target,
        elevation: str,
    ) -> bool:

        pitch, source = _marker_elevation(
            target.get(
                "marker",
                target,
            )
        )

        if source == "none":
            return False

        if elevation == "high":

            return (
                pitch
                <= self.marker_pitch_high_deg
            )

        if elevation == "low":

            return (
                pitch
                >= self.marker_pitch_low_deg
            )

        return False

    def _classify_elevation(
        self,
        target,
    ) -> str:

        if (
            self.marker_pitch_high_deg is None
            or self.marker_pitch_low_deg is None
        ):
            return "unknown"

        pitch, source = _marker_elevation(
            target.get(
                "marker",
                target,
            )
        )

        if source == "none":
            return "unknown"

        if (
            pitch
            <= self.marker_pitch_high_deg
        ):
            return "high"

        if (
            pitch
            >= self.marker_pitch_low_deg
        ):
            return "low"

        return "unknown"

    # --------------------------------------------------
    # Exclusions
    # --------------------------------------------------

    @staticmethod
    def _normalise_exclude_ids(
        values,
    ) -> set[int]:

        if values is None:
            return set()

        result = set()

        for value in values:

            target_id = _safe_int(value)

            if target_id is not None:
                result.add(target_id)

        return result

    def _current_exclude_ids(
        self,
    ) -> set[int]:

        # Constructor-level exclude_ids can intentionally be
        # a shared mutable set.
        if self.exclude_ids is not None:

            return self._normalise_exclude_ids(
                self.exclude_ids
            )

        return set(
            self._exclude_ids
        )

    # --------------------------------------------------
    # No-target handling
    # --------------------------------------------------

    def _no_target_status(
        self,
        now: float,
    ):

        elapsed = 0.0

        if self._started_s is not None:
            elapsed = (
                now - self._started_s
            )

        if (
            self.timeout_s is not None
            and elapsed >= self.timeout_s
        ):

            print(
                f"[{self.label}] "
                "no eligible target "
                f"after {elapsed:.2f}s "
                "-> FAILED"
            )

            return PrimitiveStatus.FAILED

        if (
            self._last_no_target_log is None
            or (
                now
                - self._last_no_target_log
            )
            >= self.log_every_s
        ):

            print(
                f"[{self.label}] "
                "no eligible target visible "
                "— waiting"
            )

            self._last_no_target_log = now

        return PrimitiveStatus.RUNNING