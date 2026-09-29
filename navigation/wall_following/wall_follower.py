# navigation/wall_following/wall_follower.py

from __future__ import annotations

import math
from enum import Enum

from navigation.wall_geometry import WallGeometry
from navigation.wall_following.distance_only import (
    DistanceOnlyWallFollower,
)
from navigation.wall_following.heading_distance import (
    HeadingDistanceWallFollower,
)
from navigation.wall_following.heading_only import (
    HeadingOnlyWallFollower,
)
from navigation.wall_following.models import (
    WallFollowResult,
    WallSide,
)


class WallFollowMode(Enum):
    HEADING_DISTANCE = "heading_distance"
    DISTANCE_ONLY = "distance_only"
    HEADING_ONLY = "heading_only"


class WallFollower:
    """
    Select the strongest available wall-following algorithm.

    Selection order:

        heading + distance
            -> HeadingDistanceWallFollower

        distance only
            -> DistanceOnlyWallFollower

        heading only
            -> HeadingOnlyWallFollower

        neither
            -> None

    This class knows nothing about sensors or robot hardware.
    """

    def __init__(self):
        self.heading_distance = (
            HeadingDistanceWallFollower()
        )

        self.distance_only = (
            DistanceOnlyWallFollower()
        )

        self.heading_only = (
            HeadingOnlyWallFollower()
        )

        self.active_mode: WallFollowMode | None = None

    def reset(self) -> None:
        self.distance_only.reset()
        self.active_mode = None

    @staticmethod
    def _parallel_heading(
        wall_side: WallSide,
    ) -> float:

        if wall_side == WallSide.LEFT:
            return math.pi / 2.0

        return -math.pi / 2.0

    def update(
        self,
        *,
        wall: WallGeometry,
        wall_side: WallSide,
        desired_distance_mm: float,
        linear_x_mps: float,
        dt_s: float,
    ) -> WallFollowResult | None:

        desired_heading_rad = (
            self._parallel_heading(
                wall_side
            )
        )

        # --------------------------------------------------
        # Heading + distance
        # --------------------------------------------------

        if (
            wall.has_heading
            and wall.has_distance
        ):
            self.active_mode = (
                WallFollowMode.HEADING_DISTANCE
            )

            return self.heading_distance.update(
                heading_rad=wall.heading_rad,
                distance_mm=wall.distance_mm,
                desired_distance_mm=desired_distance_mm,
                desired_heading_rad=desired_heading_rad,
                wall_side=wall_side,
                linear_x_mps=linear_x_mps,
            )

        # --------------------------------------------------
        # Distance only
        # --------------------------------------------------

        if wall.has_distance:

            if (
                self.active_mode
                != WallFollowMode.DISTANCE_ONLY
            ):
                # Do not calculate a derivative against an
                # old measurement from a previous mode.
                self.distance_only.reset()

            self.active_mode = (
                WallFollowMode.DISTANCE_ONLY
            )

            return self.distance_only.update(
                distance_mm=wall.distance_mm,
                desired_distance_mm=desired_distance_mm,
                wall_side=wall_side,
                linear_x_mps=linear_x_mps,
                dt_s=dt_s,
            )

        # --------------------------------------------------
        # Heading only
        # --------------------------------------------------

        if wall.has_heading:
            self.active_mode = (
                WallFollowMode.HEADING_ONLY
            )

            return self.heading_only.update(
                heading_rad=wall.heading_rad,
                desired_heading_rad=desired_heading_rad,
                linear_x_mps=linear_x_mps,
            )

        # --------------------------------------------------
        # No usable wall geometry
        # --------------------------------------------------

        self.active_mode = None
        return None