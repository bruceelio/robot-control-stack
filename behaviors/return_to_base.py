# behaviors/return_to_base.py

from behaviors.base import Behavior, BehaviorStatus
from primitives.base import PrimitiveStatus
from primitives.motion import Drive
from skills.navigation.return_to_base_servo import ReturnToBaseServo
from config.arena import return_guide_routes


STAGE1_RETURN_DISTANCE_MM = 750.0


class ReturnToBase(Behavior):
    """
    Navigate from the collection area back to the delivery position.

    Navigation stages:

        Stage 2 = vision servoing via ReturnToBaseServo
        Stage 1 = existing scripted/dead-reckoning return

    This behavior does NOT release the object.
    DeliverObject owns the drop-off action.
    """

    def __init__(self):
        super().__init__()

        self.config = None
        self.active = None

        self.match_zone = None
        self.guide_routes = None
        self.arrival_side = None

        # 2 = perception/servo navigation
        # 1 = dead reckoning
        self._navigation_stage = None

    def start(
            self,
            *,
            config,
            match_zone,
            **_,
    ):
        print("[RETURN_TO_BASE] start")

        self.config = config
        self.active = None
        self.arrival_side = None
        self.status = BehaviorStatus.RUNNING

        self.match_zone = int(match_zone)
        self.guide_routes = return_guide_routes(
            self.match_zone
        )

        print(
            "[RETURN_TO_BASE] "
            f"zone={self.match_zone} "
            f"routes={self.guide_routes}"
        )

        if self.config.servoing_enabled:
            print(
                "[RETURN_TO_BASE][NAV] "
                "Stage 2 available -> RETURN TO BASE SERVO"
            )
            self._navigation_stage = 2

        else:
            print(
                "[RETURN_TO_BASE][NAV] "
                "Stage 2 unavailable -> Stage 1 DEAD_RECKONING"
            )
            self._navigation_stage = 1

        return self.status

    def update(
        self,
        *,
        lvl2,
        motion_backend,
        arena_observations=None,
        observation_timestamp=None,
        **_,
    ):
        if self.status != BehaviorStatus.RUNNING:
            return self.status

        # --------------------------------------------------
        # Start selected navigation stage
        # --------------------------------------------------

        if self.active is None:

            if self._navigation_stage == 2:
                print(
                    "[RETURN_TO_BASE][NAV] "
                    "starting Stage 2 ReturnToBaseServo"
                )

                self.active = ReturnToBaseServo(
                    config=self.config,
                    guide_routes=self.guide_routes,
                )

                self.active.start(
                    lvl2=lvl2,
                )

            else:
                print(
                    "[RETURN_TO_BASE][NAV] "
                    f"starting Stage 1 Drive "
                    f"{STAGE1_RETURN_DISTANCE_MM:.0f}mm"
                )

                self.active = Drive(
                    distance_mm=STAGE1_RETURN_DISTANCE_MM,
                )

                self.active.start(
                    motion_backend=motion_backend,
                )

        # --------------------------------------------------
        # Update selected navigation stage
        # --------------------------------------------------

        if self._navigation_stage == 2:

            if (
                arena_observations is None
                or observation_timestamp is None
            ):
                print(
                    "[RETURN_TO_BASE][NAV] "
                    "Stage 2 missing vision input"
                )

                self.status = BehaviorStatus.FAILED
                return self.status

            st = self.active.update(
                arena_observations=arena_observations,
                observation_timestamp=observation_timestamp,
            )

        else:
            st = self.active.update(
                motion_backend=motion_backend,
            )

        # --------------------------------------------------
        # Result
        # --------------------------------------------------

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            print(
                "[RETURN_TO_BASE][NAV] "
                f"Stage {self._navigation_stage} FAILED"
            )

            self.status = BehaviorStatus.FAILED
            return self.status

        print(
            "[RETURN_TO_BASE][NAV] "
            f"Stage {self._navigation_stage} complete"
        )

        if self._navigation_stage == 2:
            self.arrival_side = getattr(
                self.active,
                "selected_guide_side",
                None,
            )

        print(
            "[RETURN_TO_BASE] "
            f"arrival_side={self.arrival_side}"
        )

        self.status = BehaviorStatus.SUCCEEDED
        return self.status