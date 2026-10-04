# autonomous/SR2026/auto_sr2026_stage1.py

from state_machine import RobotState

from autonomous.SR2026.init_escape import InitEscape
from autonomous.SR2026.approach_object import ApproachObject
from autonomous.SR2026.pickup_object import PickupObject
from autonomous.SR2026.post_pickup_realign import PostPickupRealign
from autonomous.SR2026.recover_localisation import RecoverLocalisation
from autonomous.SR2026.dropoff_object import DropoffObject
from autonomous.SR2026.post_dropoff_realign import PostDropoffRealign
from autonomous.SR2026.scripted_start import ScriptedStart
from autonomous.SR2026.return_to_base import ReturnToBase
from autonomous.SR2026.global_object_search import GlobalObjectSearch

from skills.manipulation.prepare_search import PrepareSearch
from skills.perception.select_target import SelectTarget

from config import CONFIG
from config.strategy import STARTUP_SCRIPT, StartupScript

from perception.providers.acquisition import (
    acquire_wall_geometry,
)


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
        self.delivered_kind: str | None = None
        self.pending_pickup_kind: str | None = None

        self.pending_pickup_id: int | None = None
        self.pickup_distance_mm: float | None = None
        self.pickup_bearing_deg: float | None = None
        self.pickup_target_is_high: bool | None = None
        self.selected_target = None
        self.selected_kind: str | None = None
        self.selected_elevation: str | None = None
        self.selected_servo_method = None
        self.selected_approach_feasibility = None


        self.return_arrival_side = None
        self.recover_localisation_success_state = None

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
                self.state = RobotState.PREPARE_SEARCH

            return


        # -------------------------
        # PREPARE SEARCH
        # -------------------------
        if self.state == RobotState.PREPARE_SEARCH:
            if self.behavior is None:
                self.behavior = PrepareSearch()

                self.behavior.start(
                    lvl2=controller.lvl2,
                )

            status = self.behavior.update()

            if status.name == "SUCCEEDED":
                print("PrepareSearch complete")
                self.behavior = None
                self.state = RobotState.GLOBAL_OBJECT_SEARCH

            elif status.name == "FAILED":
                print("PrepareSearch failed")
                # Failure routing still TBD.
                # Do not silently retry it here.

            return

        # -------------------------
        # GLOBAL OBJECT SEARCH
        # -------------------------
        if self.state == RobotState.GLOBAL_OBJECT_SEARCH:

            if self.behavior is None:
                # Before the first successful delivery, either object
                # kind is sufficient to move on to SelectTarget.
                #
                # After the first delivery, the delivered kind becomes
                # mandatory.
                required_kind = (
                    None
                    if self.delivered_kind is None
                    else self.delivered_kind
                )

                self.behavior = GlobalObjectSearch()

                self.behavior.start(
                    config=CONFIG,
                    required_kind=required_kind,
                    exclude_ids=self.delivered_ids,
                    motion_backend=controller.motion_backend,
                    localisation=controller.localisation,
                )

            status = self.behavior.update(
                perception=controller.perception,
                motion_backend=controller.motion_backend,
                localisation=controller.localisation,
            )

            if status.name == "SUCCEEDED":

                print(
                    "[GLOBAL_OBJECT_SEARCH] complete "
                    "-> SELECT_TARGET"
                )

                self.behavior = None
                self.state = RobotState.SELECT_TARGET

            elif status.name == "FAILED":

                print(
                    "[GLOBAL_OBJECT_SEARCH] failed "
                    "-> RECOVER_LOCALISATION"
                )

                self.behavior = None

                self.recover_localisation_success_state = (
                    RobotState.PREPARE_SEARCH
                )

                self.state = RobotState.RECOVER_LOCALISATION

            return

        # -------------------------
        # SELECT TARGET
        # -------------------------
        if self.state == RobotState.SELECT_TARGET:

            if self.behavior is None:

                if self.delivered_kind is None:
                    required_kind = None
                    preferred_kind = CONFIG.default_target_kind
                    preferred_elevation = CONFIG.default_target_elevation

                else:
                    required_kind = self.delivered_kind
                    preferred_kind = None
                    preferred_elevation = None

                self.behavior = SelectTarget(
                    config=CONFIG,

                    pose_bearing_allowed=(
                            CONFIG.environment == "simulation"
                    ),

                    max_age_s=CONFIG.visible_max_age_s,

                    required_kind=required_kind,
                    preferred_kind=preferred_kind,
                    preferred_elevation=preferred_elevation,

                    marker_pitch_high_deg=CONFIG.marker_pitch_high_deg,
                    marker_pitch_low_deg=CONFIG.marker_pitch_low_deg,

                    timeout_s=float(
                        getattr(
                            CONFIG,
                            "select_timeout_s",
                            0.8,
                        )
                    ),

                    label="SELECT_TARGET",

                    exclude_ids=self.delivered_ids,
                )

                self.behavior.start(
                    exclude_ids=self.delivered_ids,
                )

            status = self.behavior.update(
                perception=controller.perception,
            )

            if status.name == "SUCCEEDED":

                self.selected_target = (
                    self.behavior.selected_target
                )

                self.selected_kind = (
                    self.behavior.selected_kind
                )

                self.selected_elevation = (
                    self.behavior.selected_elevation
                )

                self.selected_servo_method = (
                    self.behavior.selected_servo_method
                )

                self.selected_approach_feasibility = (
                    self.behavior
                    .selected_approach_feasibility
                )

                print(
                    "[SELECT_TARGET] complete "
                    f"id={self.behavior.selected_target_id} "
                    f"kind={self.selected_kind} "
                    f"elevation={self.selected_elevation} "
                    f"servo={self.selected_servo_method.value}"
                )

                self.behavior = None
                self.state = RobotState.SEEK_AND_COLLECT


            elif status.name == "FAILED":

                print(

                    "SelectTarget failed "

                    "-> GLOBAL_OBJECT_SEARCH"

                )

                self.behavior = None

                self.state = RobotState.GLOBAL_OBJECT_SEARCH

            return

        # -------------------------
        # APPROACH OBJECT
        # -------------------------

        if self.state == RobotState.SEEK_AND_COLLECT:
            if self.behavior is None:
                self.behavior = ApproachObject(
                    pose_bearing_allowed=(
                            CONFIG.environment == "simulation"
                    )
                )

                self.behavior.start(
                    config=CONFIG,
                    selected_target=self.selected_target,
                    selected_kind=self.selected_kind,
                    selected_elevation=self.selected_elevation,
                    selected_servo_method=self.selected_servo_method,
                    selected_approach_feasibility=(
                        self.selected_approach_feasibility
                    ),
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

                self.pending_pickup_kind = getattr(
                    self.behavior,
                    "kind",
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

                self.selected_target = None
                self.selected_kind = None
                self.selected_elevation = None
                self.selected_servo_method = None
                self.selected_approach_feasibility = None

                self.behavior = None
                self.state = RobotState.PICKUP_OBJECT


            elif status.name == "FAILED":

                print(

                    "ApproachObject failed — "

                    "returning to target selection"

                )

                self.selected_target = None
                self.selected_kind = None
                self.selected_elevation = None
                self.selected_servo_method = None
                self.selected_approach_feasibility = None

                self.behavior = None

                # Temporary until ReacquireTarget is the proper route.

                self.state = RobotState.SELECT_TARGET

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
                    target_id=self.pending_pickup_id,
                )

            status = self.behavior.update(
                lvl2=controller.lvl2,
                motion_backend=controller.motion_backend,
                io=controller.io,
                perception=controller.perception,
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
                self.pending_pickup_kind = None
                self.behavior = None
                self.state = RobotState.PREPARE_SEARCH

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

                self.recover_localisation_success_state = (
                    RobotState.RETURN_TO_BASE
                )

                self.state = RobotState.RECOVER_LOCALISATION


            elif status.name == "FAILED":

                print("PostPickupRealign failed — attempting recovery")

                self.behavior = None

                self.recover_localisation_success_state = (

                    RobotState.RETURN_TO_BASE

                )

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

                next_state = (
                    self.recover_localisation_success_state
                    if self.recover_localisation_success_state is not None
                    else RobotState.RETURN_TO_BASE
                )

                self.recover_localisation_success_state = None
                self.state = next_state


            elif status.name == "FAILED":

                print("RecoverLocalisation failed")

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
                    match_zone=controller.io.usb["match_zone"].value,
                )

            wall_geometry = None

            wall_side = getattr(
                self.behavior,
                "arrival_side",
                None,
            )

            if wall_side is not None:
                wall_geometry = acquire_wall_geometry(
                    config=CONFIG,
                    io=controller.io,
                    wall_side=wall_side,
                )
            status = self.behavior.update(
                lvl2=controller.lvl2,
                motion_backend=controller.motion_backend,
                io=controller.io,
                localisation=controller.localisation,
                wall_geometry=wall_geometry,
                perception=controller.perception,
                delivered_ids=self.delivered_ids,
                arena_observations=controller.latest_arena_observations,
                observation_timestamp=(
                    controller.latest_arena_observation_timestamp
                ),
            )

            wall_geometry = None

            wall_side = getattr(
                self.behavior,
                "arrival_side",
                None,
            )

            if wall_side is not None:
                wall_geometry = acquire_wall_geometry(
                    config=CONFIG,
                    io=controller.io,
                    wall_side=wall_side,
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
                    match_zone=controller.io.usb["match_zone"].value,
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

                    if (
                            self.delivered_kind is None
                            and self.pending_pickup_kind is not None
                    ):
                        self.delivered_kind = self.pending_pickup_kind

                        print(
                            f"[STRATEGY] first delivered kind="
                            f"{self.delivered_kind}"
                        )

                    self.pending_pickup_kind = None

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

                print("DropoffObject failed — returning to search")

                self.behavior = None

                self.state = RobotState.PREPARE_SEARCH

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
                print("PostDropoffRealign complete — preparing next search")
                self.behavior = None
                self.state = RobotState.PREPARE_SEARCH

            elif status.name == "FAILED":
                print("PostDropoffRealign failed — preparing next search anyway")
                self.behavior = None
                self.state = RobotState.PREPARE_SEARCH

            return


def run(controller):
    autonomous = AutoSR2026Stage1()

    while True:
        controller.tick()
        autonomous.update(controller)