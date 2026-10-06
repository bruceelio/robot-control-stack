# skills/perception/home_base.py

from __future__ import annotations

from dataclasses import dataclass
import math

from config.arena import (
    BaseID,
    base_bounds,
)


@dataclass(frozen=True)
class HomeBaseState:
    """
    Semantic estimate of whether base_link is safely inside
    the robot's own home base.

    inside:
        True
            valid localisation places base_link inside the
            robot-specific safe home-base region.

        False
            valid localisation places base_link outside the
            safe home-base region.

        None
            no valid localisation position is currently
            available.

    Heading is deliberately not required.
    """

    inside: bool | None

    base_id: BaseID

    base_link_x_mm: float | None
    base_link_y_mm: float | None

    nominal_bounds_mm: tuple[
        float,
        float,
        float,
        float,
    ]

    safe_bounds_mm: tuple[
        float,
        float,
        float,
        float,
    ]

    entry_margin_mm: float

    position_valid: bool

    source: str | None
    timestamp_s: float | None


def safe_home_base_bounds(
    *,
    base_id: BaseID,
    arena_size_mm: float,
    entry_margin_mm: float,
) -> tuple[
    float,
    float,
    float,
    float,
]:
    """
    Return the safe base_link region for one home base.

    The margin is applied only to the two field-facing edges.

    The two arena-wall edges are not inset.

    This allows the robot profile to account for:
        - base_link -> gripper offset
        - carried-object protrusion
        - robot-specific geometry
        - desired placement tolerance

    For a 2000 x 1000 mm base and a 200 mm margin,
    the usable region becomes 1800 x 800 mm.
    """

    margin_mm = float(
        entry_margin_mm
    )

    if (
        not math.isfinite(margin_mm)
        or margin_mm < 0.0
    ):
        raise ValueError(
            "entry_margin_mm must be finite and >= 0"
        )

    (
        xmin,
        xmax,
        ymin,
        ymax,
    ) = base_bounds(
        base_id,
        int(arena_size_mm),
    )

    width_mm = xmax - xmin
    height_mm = ymax - ymin

    if margin_mm >= min(
        width_mm,
        height_mm,
    ):
        raise ValueError(
            "entry_margin_mm is too large for "
            "the home-base geometry"
        )

    # --------------------------------------------------
    # Field-facing boundaries
    # --------------------------------------------------
    #
    # BASE_0:
    #
    #     right  = field
    #     bottom = field
    #
    # BASE_1:
    #
    #     left   = field
    #     bottom = field
    #
    # BASE_2:
    #
    #     left   = field
    #     top    = field
    #
    # BASE_3:
    #
    #     right  = field
    #     top    = field
    # --------------------------------------------------

    if base_id == BaseID.BASE_0:
        xmax -= margin_mm
        ymin += margin_mm

    elif base_id == BaseID.BASE_1:
        xmin += margin_mm
        ymin += margin_mm

    elif base_id == BaseID.BASE_2:
        xmin += margin_mm
        ymax -= margin_mm

    elif base_id == BaseID.BASE_3:
        xmax -= margin_mm
        ymax -= margin_mm

    else:
        raise ValueError(
            base_id
        )

    return (
        xmin,
        xmax,
        ymin,
        ymax,
    )


class HomeBase:
    """
    Convert the resolved localisation position into the
    semantic statement:

        "base_link is safely inside our home base"

    This skill does NOT:
        - process AprilTags
        - perform localisation
        - fuse localisation providers
        - require robot heading
        - know about return-route guides
        - make autonomous decisions

    Localisation remains the single owner of robot arena pose.
    """

    def __init__(
        self,
        *,
        arena_size_mm: float,
        entry_margin_mm: float,
    ):
        self.arena_size_mm = float(
            arena_size_mm
        )

        self.entry_margin_mm = float(
            entry_margin_mm
        )

        if (
            not math.isfinite(
                self.arena_size_mm
            )
            or self.arena_size_mm <= 0.0
        ):
            raise ValueError(
                "arena_size_mm must be finite and > 0"
            )

        if (
            not math.isfinite(
                self.entry_margin_mm
            )
            or self.entry_margin_mm < 0.0
        ):
            raise ValueError(
                "entry_margin_mm must be finite and >= 0"
            )

    @staticmethod
    def _base_id(
        match_zone,
    ) -> BaseID:
        if isinstance(
            match_zone,
            BaseID,
        ):
            return match_zone

        return BaseID(
            int(match_zone)
        )

    def evaluate(
        self,
        *,
        localisation,
        match_zone,
    ) -> HomeBaseState:
        base_id = self._base_id(
            match_zone
        )

        nominal_bounds = base_bounds(
            base_id,
            int(self.arena_size_mm),
        )

        safe_bounds = (
            safe_home_base_bounds(
                base_id=base_id,
                arena_size_mm=(
                    self.arena_size_mm
                ),
                entry_margin_mm=(
                    self.entry_margin_mm
                ),
            )
        )

        pose = getattr(
            localisation,
            "pose",
            None,
        )

        if (
            pose is None
            or not bool(
                getattr(
                    pose,
                    "position_valid",
                    False,
                )
            )
        ):
            return HomeBaseState(
                inside=None,
                base_id=base_id,
                base_link_x_mm=None,
                base_link_y_mm=None,
                nominal_bounds_mm=(
                    nominal_bounds
                ),
                safe_bounds_mm=(
                    safe_bounds
                ),
                entry_margin_mm=(
                    self.entry_margin_mm
                ),
                position_valid=False,
                source=None,
                timestamp_s=None,
            )

        x_mm = float(
            pose.x
        )

        y_mm = float(
            pose.y
        )

        if (
            not math.isfinite(x_mm)
            or not math.isfinite(y_mm)
        ):
            return HomeBaseState(
                inside=None,
                base_id=base_id,
                base_link_x_mm=None,
                base_link_y_mm=None,
                nominal_bounds_mm=(
                    nominal_bounds
                ),
                safe_bounds_mm=(
                    safe_bounds
                ),
                entry_margin_mm=(
                    self.entry_margin_mm
                ),
                position_valid=False,
                source=getattr(
                    pose,
                    "source",
                    None,
                ),
                timestamp_s=getattr(
                    pose,
                    "timestamp",
                    None,
                ),
            )

        (
            xmin,
            xmax,
            ymin,
            ymax,
        ) = safe_bounds

        inside = (
            xmin <= x_mm <= xmax
            and
            ymin <= y_mm <= ymax
        )

        return HomeBaseState(
            inside=inside,
            base_id=base_id,
            base_link_x_mm=x_mm,
            base_link_y_mm=y_mm,
            nominal_bounds_mm=(
                nominal_bounds
            ),
            safe_bounds_mm=(
                safe_bounds
            ),
            entry_margin_mm=(
                self.entry_margin_mm
            ),
            position_valid=True,
            source=getattr(
                pose,
                "source",
                None,
            ),
            timestamp_s=getattr(
                pose,
                "timestamp",
                None,
            ),
        )