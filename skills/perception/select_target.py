# skills/perception/select_target.py

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

import time
from enum import Enum
from typing import Iterable, Optional

from primitives.base import PrimitiveStatus

from perception import get_visible_targets
from skills.perception.marker_elevation import _marker_elevation


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
        # Preference 1: elevation
        # --------------------------------------------------

        preferred = candidates

        if self.preferred_elevation is not None:

            elevation_matches = [
                candidate
                for candidate in preferred
                if self._matches_elevation(
                    candidate[1],
                    self.preferred_elevation,
                )
            ]

            # Preference only.
            #
            # If nothing satisfies the requested elevation,
            # retain the current candidate set.
            if elevation_matches:
                preferred = elevation_matches

        # --------------------------------------------------
        # Preference 2: kind
        # --------------------------------------------------

        if self.preferred_kind is not None:

            kind_matches = [
                candidate
                for candidate in preferred
                if candidate[0]
                == self.preferred_kind
            ]

            # Preference only.
            if kind_matches:
                preferred = kind_matches

        # --------------------------------------------------
        # Preference 3: distance
        # --------------------------------------------------

        selected_kind, selected_target = min(
            preferred,
            key=lambda candidate: float(
                candidate[1]["distance"]
            ),
        )

        # --------------------------------------------------
        # Store outputs
        # --------------------------------------------------

        self.selected_target = selected_target

        self.selected_target_id = _safe_int(
            selected_target.get("id")
        )

        self.selected_kind = selected_kind

        self.selected_elevation = (
            self._classify_elevation(
                selected_target
            )
        )

        # Current default approach policy.
        #
        # Future target-selection logic may choose POSE_BEARING
        # when target geometry requires it and pose visual_servoing is
        # both available and usable.
        self.selected_servo_method = (
            ApproachServoMethod.TARGET_BEARING
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

        print(
            f"[{self.label}] "
            f"selected id={tid_display} "
            f"kind={self.selected_kind} "
            f"elevation={self.selected_elevation} "
            f"servo={self.selected_servo_method.value} "
            f"dist={distance:.0f} "
            f"bearing={bearing:.1f}"
        )

        return PrimitiveStatus.SUCCEEDED

    def stop(self, **_):
        pass

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