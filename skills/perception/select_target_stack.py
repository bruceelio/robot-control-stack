# perception/select_target_stack.py

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from perception import get_visible_targets
from perception.robot_geometry import relative_target_from_base_link
from skills.perception.marker_elevation import _marker_elevation


TARGET_KINDS = (
    "acidic",
    "basic",
)

DEFAULT_OBJECT_RADIUS_MM = 65.0
DEFAULT_LOCAL_PLANNING_SAFETY_MARGIN_MM = 75.0


@dataclass(frozen=True)
class StackTargetSelection:
    target: dict
    target_id: int
    kind: str

    distance_mm: float

    blocking_count: int
    total_intrusion_mm: float
    ignored_obstacle_ids: tuple[int, ...]


class SelectTargetStack:
    """
    Select a delivered LOW cube to use as the stacking base.

    Hard eligibility:
        - target must be freshly visible
        - target id must be in delivered_ids
        - target must classify LOW

    Ranking:
        1. fewest interfering cubes
        2. least total corridor intrusion
        3. nearest target

    A HIGH cube already sitting on the candidate LOW cube is deliberately
    ignored. The stacking policy allows that cube to be displaced while
    replacing it with the currently-carried cube.
    """

    def __init__(
        self,
        *,
        config,
        max_age_s: float,
        object_radius_mm: float = DEFAULT_OBJECT_RADIUS_MM,
    ):
        self.config = config
        self.max_age_s = float(max_age_s)

        self.object_radius_mm = float(
            object_radius_mm
        )

        self.robot_radius_mm = (
            float(config.drive_track_width_mm)
            / 2.0
        )

        self.safety_margin_mm = float(
            getattr(
                config,
                "local_planning_safety_margin_mm",
                DEFAULT_LOCAL_PLANNING_SAFETY_MARGIN_MM,
            )
        )

    # ==================================================
    # Public selection
    # ==================================================

    def select(
        self,
        *,
        perception,
        delivered_ids: Iterable[int],
        now_s: float,
        ignore_ids: Iterable[int] = (),
    ) -> StackTargetSelection | None:

        delivered = {
            int(target_id)
            for target_id in delivered_ids
        }

        ignored = {
            int(target_id)
            for target_id in ignore_ids
        }

        if not delivered:
            return None

        visible_objects = (
            self._visible_objects(
                perception=perception,
                now_s=now_s,
            )
        )

        candidates = []

        for kind, target in visible_objects:

            target_id = self._target_id(
                target
            )

            if target_id is None:
                continue

            if target_id not in delivered:
                continue

            if target_id in ignored:
                continue

            if (
                self._classify_elevation(target)
                != "low"
            ):
                continue

            geometry = (
                self._target_geometry(
                    target
                )
            )

            if geometry is None:
                continue

            (
                target_x_mm,
                target_y_mm,
                target_distance_mm,
            ) = geometry

            (
                blocking_count,
                total_intrusion_mm,
                ignored_obstacle_ids,
            ) = self._approach_interference(
                candidate_id=target_id,
                candidate_x_mm=target_x_mm,
                candidate_y_mm=target_y_mm,
                visible_objects=visible_objects,
                ignore_ids=ignored,
            )

            candidates.append(
                (
                    blocking_count,
                    total_intrusion_mm,
                    target_distance_mm,
                    kind,
                    target_id,
                    target,
                    ignored_obstacle_ids,
                )
            )

        if not candidates:
            return None

        (
            blocking_count,
            total_intrusion_mm,
            distance_mm,
            kind,
            target_id,
            target,
            ignored_obstacle_ids,
        ) = min(
            candidates,
            key=lambda item: (
                item[0],
                item[1],
                item[2],
            ),
        )

        print(
            "[SELECT_TARGET_STACK] "
            f"selected id={target_id} "
            f"kind={kind} "
            f"distance={distance_mm:.0f}mm "
            f"blockers={blocking_count} "
            f"intrusion={total_intrusion_mm:.0f}mm "
            f"ignored={ignored_obstacle_ids}"
        )

        return StackTargetSelection(
            target=target,
            target_id=target_id,
            kind=kind,
            distance_mm=distance_mm,
            blocking_count=blocking_count,
            total_intrusion_mm=(
                total_intrusion_mm
            ),
            ignored_obstacle_ids=(
                ignored_obstacle_ids
            ),
        )

    # ==================================================
    # Candidate gathering
    # ==================================================

    def _visible_objects(
        self,
        *,
        perception,
        now_s: float,
    ):

        objects = []

        for kind in TARGET_KINDS:

            visible = get_visible_targets(
                perception,
                kind,
                now=now_s,
                max_age_s=self.max_age_s,
            )

            for target in visible:
                objects.append(
                    (
                        kind,
                        target,
                    )
                )

        return objects

    # ==================================================
    # Elevation
    # ==================================================

    def _classify_elevation(
        self,
        target,
    ) -> str:

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
            <= float(
                self.config.marker_pitch_high_deg
            )
        ):
            return "high"

        if (
            pitch
            >= float(
                self.config.marker_pitch_low_deg
            )
        ):
            return "low"

        return "unknown"

    # ==================================================
    # Geometry
    # ==================================================

    @staticmethod
    def _target_id(
        target,
    ) -> int | None:

        try:
            return int(
                target.get("id")
            )

        except (
            TypeError,
            ValueError,
        ):
            return None

    def _target_geometry(
        self,
        target,
    ):

        try:
            geometry = (
                relative_target_from_base_link(
                    observation=target,
                    config=self.config,
                )
            )

            distance_mm = (
                float(
                    geometry.distance_m
                )
                * 1000.0
            )

            bearing_rad = float(
                geometry.bearing_rad
            )

        except (
            AttributeError,
            KeyError,
            TypeError,
            ValueError,
        ):
            return None

        if (
            not math.isfinite(distance_mm)
            or not math.isfinite(bearing_rad)
            or distance_mm <= 0.0
        ):
            return None

        return (
            distance_mm
            * math.cos(bearing_rad),
            distance_mm
            * math.sin(bearing_rad),
            distance_mm,
        )

    # ==================================================
    # Interference
    # ==================================================

    def _approach_interference(
        self,
        *,
        candidate_id: int,
        candidate_x_mm: float,
        candidate_y_mm: float,
        visible_objects,
        ignore_ids: set[int],
    ) -> tuple[int, float, tuple[int, ...]]:

        corridor_length_sq = (
            candidate_x_mm
            * candidate_x_mm
            + candidate_y_mm
            * candidate_y_mm
        )

        if corridor_length_sq <= 1e-9:
            return (
                0,
                0.0,
                (),
            )

        corridor_clearance_mm = (
            self.robot_radius_mm
            + self.safety_margin_mm
            + self.object_radius_mm
        )

        blocking_count = 0
        total_intrusion_mm = 0.0
        ignored_obstacle_ids: set[int] = set()

        for _, obstacle in visible_objects:

            obstacle_id = self._target_id(
                obstacle
            )

            if obstacle_id is None:
                continue

            if obstacle_id == candidate_id:
                continue

            if obstacle_id in ignore_ids:
                continue

            geometry = (
                self._target_geometry(
                    obstacle
                )
            )

            if geometry is None:
                continue

            (
                obstacle_x_mm,
                obstacle_y_mm,
                _,
            ) = geometry

            # A HIGH cube occupying effectively the same
            # ground position as the LOW target is the cube
            # currently stacked on top of it. It is explicitly
            # expendable for this stacking strategy.
            if (
                self._classify_elevation(
                    obstacle
                )
                == "high"
            ):
                target_separation_mm = math.hypot(
                    obstacle_x_mm
                    - candidate_x_mm,
                    obstacle_y_mm
                    - candidate_y_mm,
                )

                if (
                    target_separation_mm
                    <= (
                        2.0
                        * self.object_radius_mm
                    )
                ):
                    ignored_obstacle_ids.add(
                        obstacle_id
                    )
                    continue

            along = (
                obstacle_x_mm
                * candidate_x_mm
                + obstacle_y_mm
                * candidate_y_mm
            ) / corridor_length_sq

            # Only obstacles between the robot and
            # the selected stacking cube matter.
            if (
                along <= 0.0
                or along >= 1.0
            ):
                continue

            closest_x_mm = (
                along
                * candidate_x_mm
            )

            closest_y_mm = (
                along
                * candidate_y_mm
            )

            lateral_mm = math.hypot(
                obstacle_x_mm
                - closest_x_mm,
                obstacle_y_mm
                - closest_y_mm,
            )

            intrusion_mm = (
                corridor_clearance_mm
                - lateral_mm
            )

            if intrusion_mm <= 0.0:
                continue

            blocking_count += 1
            total_intrusion_mm += (
                intrusion_mm
            )

        return (
            blocking_count,
            total_intrusion_mm,
            tuple(
                sorted(
                    ignored_obstacle_ids
                )
            ),
        )