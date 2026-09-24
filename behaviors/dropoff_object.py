# behaviors/dropoff_object.py

from __future__ import annotations

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus
from primitives.manipulation import Release
from primitives.motion import Rotate


class DropoffObject(Behavior):
    """
Final drop-off behavior after ReturnToBase reaches its handoff position.

For now:
    RELEASE -> SUCCEEDED

This behavior will later own:
    - final drop target selection
    - fine alignment
    - final controlled drive
    - stack/lift positioning
    - release

Post-dropoff clearance remains owned by PostDropoffRealign.
"""
    def __init__(self):
        super().__init__()

        self.active_primitive = None
        self.step = None

        # Marker id to mark delivered after successful release.
        self.delivered_target_id = None

    @property
    def delivered_id(self):
        return self.delivered_target_id

    def start(
            self,
            *,
            config,
            match_zone,
            arrival_side=None,
            delivered_target_id=None,
            delivered_ids=None,
            **_,
    ):
        print("[DROPOFF_OBJECT] start")

        self.config = config
        self.match_zone = int(match_zone)
        self.delivered_target_id = delivered_target_id
        self.delivered_ids = set(delivered_ids) if delivered_ids else set()
        self.arrival_side = arrival_side

        self.active_primitive = None
        self.status = BehaviorStatus.RUNNING

        if self.arrival_side in ("left", "right"):
            print(
                "[DROPOFF_OBJECT] "
                f"Stage 2 placement side={self.arrival_side}"
            )
            self.step = "ROTATE"
        else:
            print(
                "[DROPOFF_OBJECT] "
                "Stage 1 / no arrival side -> RELEASE"
            )
            self.step = "RELEASE"

        print(
            "[DROPOFF_OBJECT] "
            f"arrival_side={self.arrival_side}"
        )

        if self.delivered_target_id is not None:
            print(
                "[DROPOFF_OBJECT] "
                f"will mark delivered id="
                f"{self.delivered_target_id}"
            )

        return self.status

    def update(
            self,
            *,
            lvl2,
            perception=None,
            motion_backend=None,
            arena_observations=None,
            observation_timestamp=None,
            **_,
    ):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        if self.active_primitive is None:

            if self.step == "ROTATE":

                # guide_side="left" is the 18 -> 19 route.
                # Turn clockwise/right, into the arena.
                if self.arrival_side == "left":
                    angle_deg = -70.0

                # guide_side="right" is the opposite return route.
                # Turn counter-clockwise/left, into the arena.
                else:
                    angle_deg = 70.0

                print(
                    "[DROPOFF_OBJECT] "
                    f"rotate inward {angle_deg:+.0f}deg"
                )

                self.active_primitive = Rotate(
                    angle_deg=angle_deg,
                )

                try:
                    self.active_primitive.start(
                        motion_backend=motion_backend,
                    )
                except RuntimeError as exc:
                    print(
                        "[DROPOFF_OBJECT] "
                        f"rotate ended early: {exc} "
                        "-> RELEASE"
                    )
                    self.active_primitive = None
                    self.step = "RELEASE"
                    return self.status

            elif self.step == "RELEASE":
                print("[DROPOFF_OBJECT] release")

                self.active_primitive = Release()
                self.active_primitive.start(
                    lvl2=lvl2,
                )

            else:
                self.status = BehaviorStatus.SUCCEEDED

                if self.delivered_target_id is not None:
                    print(
                        "[DROPOFF_OBJECT] "
                        f"delivered id="
                        f"{self.delivered_target_id}"
                    )

                return self.status

        if self.step == "ROTATE":
            st = self.active_primitive.update(
                motion_backend=motion_backend,
            )
        else:
            st = self.active_primitive.update()

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:

            if self.step == "ROTATE":
                print(
                    "[DROPOFF_OBJECT] "
                    "rotate FAILED -> RELEASE"
                )
                self.active_primitive = None
                self.step = "RELEASE"
                return self.status

            self.active_primitive = None
            self.status = BehaviorStatus.FAILED
            return self.status

        if self.step == "ROTATE":
            print("[DROPOFF_OBJECT] rotate complete")

            self.active_primitive = None
            self.step = "RELEASE"
            return self.status

        print("[DROPOFF_OBJECT] release complete")

        self.active_primitive = None
        self.step = "DONE"

        return self.status