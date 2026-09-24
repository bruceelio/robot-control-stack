# autonomous/auto_sr2026_stage1.py

from state_machine import RobotState

from behaviors.init_escape import InitEscape
from behaviors.approach_object import ApproachObject
from behaviors.pickup_object import PickupObject
from behaviors.post_pickup_realign import PostPickupRealign
from behaviors.recover_localisation import RecoverLocalisation
from behaviors.dropoff_object import DropoffObject
from behaviors.post_dropoff_realign import PostDropoffRealign
from behaviors.scripted_start import ScriptedStart
from behaviors.return_to_base import ReturnToBase

from config import CONFIG
from config.strategy import STARTUP_SCRIPT, StartupScript


class AutoSR2026Stage1:
    """
    Existing SR2026 Stage 1 autonomous program.

    This is the existing autonomous strategy moved out of
    robot_controller.py.

    The shared robot runtime remains owned by Controller.
    """

    def __init__(self):

        # -------------------------
        # Strategy memory
        # -------------------------
        self.delivered_ids: set[int] = set()
        self.last_collected_id: int | None = None

        self.pending_pickup_id: int | None = None
        self.pickup_distance_mm: float | None = None
        self.pickup_bearing_deg: float | None = None
        self.pickup_target_is_high: bool | None = None

        self.return_arrival_side = None

        # -------------------------
        # State & behavior
        # -------------------------
        if STARTUP_SCRIPT == StartupScript.NONE:
            self.state = RobotState.INIT_ESCAPE
        else:
            self.state = RobotState.SCRIPTED_START

        self.behavior = None

    def update(self, controller):

        # -------------------------
        # SCRIPTED START
        # -------------------------
        if self.state == RobotState.SCRIPTED_START:
            if self.behavior is None:
                self.behavior = ScriptedStart()
                self.behavior.start(config=CONFIG)

            status = self.behavior.update(
                motion_backend=controller.motion_backend,
                lvl2=controller.lvl2,
            )

            if status.name in ("SUCCEEDED", "FAILED"):
                print(f"ScriptedStart {status.name} -> autonomous")
                self.behavior = None
                self.state = RobotState.SEEK_AND_COLLECT  # or SEEK_AND_COLLECT if you want to skip escape

            return

        # -------------------------
        # INIT ESCAPE
        # -------------------------
        if self.state == RobotState.INIT_ESCAPE:
            if self.behavior is None:
                self.behavior = InitEscape()
                self.behavior.start(
                    config=CONFIG,
                    motion_backend=controller.motion_backend
                )

            status = self.behavior.update(
                lvl2=controller.lvl2,
                localisation=controller.localisation,
                motion_backend=controller.motion_backend
            )

            if status.name == "SUCCEEDED":
                self.behavior = None
                self.state = RobotState.SEEK_AND_COLLECT

            return

        # -------------------------
        # ACQUIRE OBJECT
        # -------------------------
        if self.state == RobotState.SEEK_AND_COLLECT:
            if self.behavior is None:
                self.behavior = ApproachObject()
                self.behavior.start(
                    config=CONFIG,
                    kind=CONFIG.default_target_kind,
                    exclude_ids=self.delivered_ids,  # NEW: don’t re-select delivered markers
                )

            status = self.behavior.update(
                lvl2=controller.lvl2,
                perception=controller.perception,
                localisation=controller.localisation,
                motion_backend=controller.motion_backend
            )

            if status.name == "SUCCEEDED":
                self.pending_pickup_id = getattr(
                    self.behavior,
                    "acquired_id",
                    None,
                )

                self.pickup_distance_mm = getattr(
                    self.behavior,
                    "final_distance_mm",
                    None,
                )

                self.pickup_bearing_deg = getattr(
                    self.behavior,
                    "final_bearing_deg",
                    None,
                )

                self.pickup_target_is_high = getattr(
                    self.behavior,
                    "target_is_high",
                    None,
                )

                print(
                    "ApproachObject complete "
                    f"id={self.pending_pickup_id} "
                    f"distance={self.pickup_distance_mm} "
                    f"bearing={self.pickup_bearing_deg} "
                    f"high={self.pickup_target_is_high}"
                )

                self.behavior = None
                self.state = RobotState.PICKUP_OBJECT

            elif status.name == "FAILED":
                print("AcquireObject failed — retrying")
                self.behavior = None
                self.state = RobotState.SEEK_AND_COLLECT

            return

        # -------------------------
        # PICKUP OBJECT
        # -------------------------
        if self.state == RobotState.PICKUP_OBJECT:
            if self.behavior is None:
                self.behavior = PickupObject()

                self.behavior.start(
                    config=CONFIG,
                    lvl2=controller.lvl2,
                    motion_backend=controller.motion_backend,
                    distance_mm=self.pickup_distance_mm,
                    bearing_deg=self.pickup_bearing_deg,
                    target_is_high=self.pickup_target_is_high,
                )

            status = self.behavior.update(
                lvl2=controller.lvl2,
                motion_backend=controller.motion_backend,
            )

            if status.name == "SUCCEEDED":
                self.last_collected_id = self.pending_pickup_id

                print(
                    f"PickupObject complete "
                    f"(id={self.last_collected_id})"
                )

                self.pending_pickup_id = None
                self.behavior = None
                self.state = RobotState.POST_PICKUP_REALIGN

            elif status.name == "FAILED":
                print("PickupObject failed — returning to search")

                self.pending_pickup_id = None
                self.behavior = None
                self.state = RobotState.SEEK_AND_COLLECT

            return

        # -------------------------
        # POST-PICKUP REALIGN
        # -------------------------
        if self.state == RobotState.POST_PICKUP_REALIGN:
            if self.behavior is None:
                self.behavior = PostPickupRealign()
                self.behavior.start(
                    config=CONFIG,
                    motion_backend=controller.motion_backend,
                    localisation=controller.localisation,
                )

            status = self.behavior.update(
                lvl2=controller.lvl2,
                perception=controller.perception,
                localisation=controller.localisation,
                motion_backend=controller.motion_backend,
            )

            if status.name == "SUCCEEDED":
                print("PostPickupRealign complete")
                self.behavior = None
                self.state = RobotState.RECOVER_LOCALISATION

            elif status.name == "FAILED":
                print("PostPickupRealign failed — attempting recovery")
                self.behavior = None
                self.state = RobotState.RECOVER_LOCALISATION

            return

        # -------------------------
        # RECOVER LOCALISATION
        # -------------------------
        if self.state == RobotState.RECOVER_LOCALISATION:
            if self.behavior is None:
                self.behavior = RecoverLocalisation()
                self.behavior.start(
                    config=CONFIG,
                    motion_backend=controller.motion_backend
                )

            status = self.behavior.update(
                perception=controller.perception,
                localisation=controller.localisation,
                motion_backend=controller.motion_backend,
            )

            if status.name == "SUCCEEDED":
                print("Localisation recovered")
                self.behavior = None
                self.state = RobotState.RETURN_TO_BASE


            elif status.name == "FAILED":

                print("ReturnToBase failed — recovering localisation")

                self.behavior = None

                self.state = RobotState.RECOVER_LOCALISATION

            return

        # -------------------------
        # RETURN TO BASE
        # -------------------------
        if self.state == RobotState.RETURN_TO_BASE:
            if self.behavior is None:
                self.behavior = ReturnToBase()
                self.behavior.start(
                    config=CONFIG,
                    match_zone=controller.match_zone,
                )

            status = self.behavior.update(
                lvl2=controller.lvl2,
                motion_backend=controller.motion_backend,
                arena_observations=controller.latest_arena_observations,
                observation_timestamp=(
                    controller.latest_arena_observation_timestamp
                ),
            )

            if status.name == "SUCCEEDED":
                self.return_arrival_side = getattr(
                    self.behavior,
                    "arrival_side",
                    None,
                )

                print(
                    "ReturnToBase complete "
                    f"arrival_side={self.return_arrival_side} "
                    "— dropping off object"
                )

                self.behavior = None
                self.state = RobotState.DROPOFF_OBJECT


            elif status.name == "FAILED":

                print("ReturnToBase failed — recovering localisation")

                self.behavior = None

                self.state = RobotState.RECOVER_LOCALISATION

            return

        # -------------------------
        # DROP OFF OBJECT
        # -------------------------
        if self.state == RobotState.DROPOFF_OBJECT:
            if self.behavior is None:
                self.behavior = DropoffObject()
                self.behavior.start(
                    config=CONFIG,
                    match_zone=controller.match_zone,
                    arrival_side=self.return_arrival_side,
                    delivered_target_id=self.last_collected_id,
                    delivered_ids=self.delivered_ids,
                )

            status = self.behavior.update(
                lvl2=controller.lvl2,
                perception=controller.perception,
                motion_backend=controller.motion_backend,
                arena_observations=controller.latest_arena_observations,
                observation_timestamp=(
                    controller.latest_arena_observation_timestamp
                ),
            )

            if status.name == "SUCCEEDED":
                delivered_id = getattr(
                    self.behavior,
                    "delivered_target_id",
                    None,
                )

                if delivered_id is None:
                    delivered_id = self.last_collected_id

                if delivered_id is not None:
                    self.delivered_ids.add(delivered_id)

                    print(
                        f"Delivered id={delivered_id} "
                        f"(delivered_ids="
                        f"{sorted(self.delivered_ids)})"
                    )

                print(
                    "DropffObject complete — "
                    "post-dropoff realign"
                )

                self.behavior = None
                self.state = RobotState.POST_DROPOFF_REALIGN

            elif status.name == "FAILED":
                print("DropoffObject failed — resuming seek")

                self.behavior = None
                self.state = RobotState.SEEK_AND_COLLECT

            return

        # -------------------------
        # POST-DROPOFF REALIGN
        # -------------------------
        if self.state == RobotState.POST_DROPOFF_REALIGN:
            if self.behavior is None:
                self.behavior = PostDropoffRealign()
                self.behavior.start(
                    config=CONFIG,
                    motion_backend=controller.motion_backend,
                )

            status = self.behavior.update(
                lvl2=controller.lvl2,
                motion_backend=controller.motion_backend,
            )

            if status.name == "SUCCEEDED":
                print("PostDropoffRealign complete — resuming seek")
                self.behavior = None
                self.state = RobotState.SEEK_AND_COLLECT

            elif status.name == "FAILED":
                print("PostDropoffRealign failed — resuming seek anyway")
                self.behavior = None
                self.state = RobotState.SEEK_AND_COLLECT

            return


def run(controller):
    autonomous = AutoSR2026Stage1()

    while True:
        controller.tick()
        autonomous.update(controller)