# robot_controller.py

import time
from level2.level2_canonical import Level2
from perception import Perception, sense
from localisation import Localisation
from state_machine import RobotState

from behaviors.init_escape import InitEscape
from behaviors.acquire_object import AcquireObject
from behaviors.post_pickup_realign import PostPickupRealign
from behaviors.recover_localisation import RecoverLocalisation
from behaviors.deliver_object import DeliverObject
from behaviors.post_dropoff_realign import PostDropoffRealign
from behaviors.scripted_start import ScriptedStart

from motion_backends import create_motion_backend

from config import CONFIG
from config.strategy import RUN_MODE, RunMode
from config.strategy import STARTUP_SCRIPT, StartupScript

from calibration import CALIBRATION
from calibration.resolve import resolve

from hw_io.base import IOMap
from hw_io.resolve import resolve_io

from log_trace import next_tick
from hw_io.buzzer_patterns import BuzzerCue

print("\n=== CALIBRATION CAMERA CHECK ===")
print("Calibration cameras:", CALIBRATION.cameras.keys())
print("=== END CALIBRATION CHECK ===\n")

try:
    from tests.runner import run_tests
except ImportError:
    run_tests = None

def safe_cue(lvl2, cue: BuzzerCue) -> None:
    print(f"[CUE] {cue.value}")  # always visible in sim/logs
    try:
        lvl2.patterns.cue(cue)
    except Exception:
        pass

class Controller:
    """
    Central coordinator.

    Decides WHICH behavior runs.
    Does NOT perform robot actions directly.
    """

    def __init__(self, robot):
        self.robot = robot

        # -------------------------
        # Core subsystems
        # -------------------------

        # IO layer (single source of hardware truth) — create this FIRST
        self.io: IOMap = resolve_io(
            robot=robot,
            hardware_profile=CONFIG.hardware_profile,
        )

        # --------------------------------------------------
        # TEMP DEBUG — CAMERA SANITY CHECK (IOMap-based)
        # --------------------------------------------------
        print("\n=== CAMERA SANITY CHECK (IOMap) ===")
        try:
            cams = self.io.cameras()
            print("io.cameras():", list(cams.keys()))
            front = cams.get("front")
            if front is None:
                print("No 'front' camera found in io.cameras()")
            else:
                seen = front.see()
                print(f"io.cameras()['front'].see() OK — saw {len(seen)} markers")
        except Exception as e:
            print("IOMap camera check FAILED:", e)
        print("=== END CAMERA CHECK ===\n")

        # Level2 now consumes IO, not robot
        self.lvl2 = Level2(
            self.io,
            max_power=CONFIG.max_motor_power,
        )

        # --- Vacuum / solenoid startup test ---
        outs = getattr(self.io, "outputs", None)
        if outs is not None:
            print("[BOOT] Forcing VACUUM OFF at startup")
            outs.set("VACUUM", False)
            self.io.sleep(0.25)

            print("[BOOT] VACUUM ON test pulse")
            outs.set("VACUUM", True)
            self.io.sleep(0.25)

            print("[BOOT] VACUUM OFF again")
            outs.set("VACUUM", False)
            self.io.sleep(0.25)
        else:
            print("[BOOT] No io.outputs available")

        # Perception now consumes IO, not robot
        self.perception = Perception(self.io)

        self.localisation = Localisation()

        # -------------------------
        # Configuration & Calibration
        # -------------------------
        self.config = CONFIG
        self.calibration = resolve(config=CONFIG)

        # -------------------------
        # Motion backend
        # -------------------------
        self.motion_backend = create_motion_backend(
            CONFIG.motion_backend,
            self.lvl2,
            self.config,
            self.calibration,
        )

        # -------------------------
        # Strategy memory
        # -------------------------
        self.delivered_ids: set[int] = set()
        self.last_collected_id: int | None = None


        # -------------------------
        # State & behavior
        # -------------------------
        if STARTUP_SCRIPT == StartupScript.NONE:
            self.state = RobotState.INIT_ESCAPE
        else:
            self.state = RobotState.SCRIPTED_START

        self.behavior = None

    # --------------------------------------------------
    # Main loop
    # --------------------------------------------------

    def run(self):
        # ----------------------------------
        # PRINT RESOLVED CONFIG (ONCE)
        # ----------------------------------
        CONFIG.dump()
        safe_cue(self.lvl2, BuzzerCue.START)

        # ----------------------------------
        # EXECUTION MODE DISPATCH
        # ----------------------------------
        if RUN_MODE == RunMode.TESTS:
            if run_tests is None:
                safe_cue(self.lvl2, BuzzerCue.ERROR)
                raise RuntimeError("Test runner not available")

            print("\n=== RUNNING TESTS MODE ===")
            try:
                run_tests(robot=self.robot)
                safe_cue(self.lvl2, BuzzerCue.SUCCESS)
            except Exception:
                safe_cue(self.lvl2, BuzzerCue.ERROR)
                raise
            finally:
                safe_cue(self.lvl2, BuzzerCue.END)

            print("=== TESTS COMPLETE ===\n")
            return

        if RUN_MODE == RunMode.DIAGNOSTICS:
            print("\n=== RUNNING DIAGNOSTICS MODE ===")
            from diagnostics.runner import run_diagnostics
            try:
                run_diagnostics(self.robot)
                safe_cue(self.lvl2, BuzzerCue.SUCCESS)
            except Exception:
                safe_cue(self.lvl2, BuzzerCue.ERROR)
                raise
            finally:
                safe_cue(self.lvl2, BuzzerCue.END)

            print("=== DIAGNOSTICS COMPLETE ===\n")
            return

        # ----------------------------------
        # NORMAL ROBOT OPERATION
        # ----------------------------------
        try:
            while True:
                self.update()
        except Exception:
            safe_cue(self.lvl2, BuzzerCue.ERROR)
            raise

    # --------------------------------------------------
    # Per-tick update
    # --------------------------------------------------

    def update(self):
        next_tick()
        # Always sense first
        arena_obs, objects = sense(self.io, self.perception)

        pose_obs = self.localisation.estimate(
            arena_observations=arena_obs,
            now_s=time.time(),
        )

        print(f"[LOC] arena={len(arena_obs)} pose_obs={'YES' if pose_obs else 'NO'}")

        if pose_obs is not None:
            self.localisation.accept(pose_obs)
        else:
            self.localisation.invalidate()

        # -------------------------
        # SCRIPTED START
        # -------------------------
        if self.state == RobotState.SCRIPTED_START:
            if self.behavior is None:
                self.behavior = ScriptedStart()
                self.behavior.start(config=CONFIG)

            status = self.behavior.update(
                motion_backend=self.motion_backend,
                lvl2=self.lvl2,
            )

            if status.name in ("SUCCEEDED", "FAILED"):
                print(f"ScriptedStart {status.name} -> autonomous")
                self.behavior = None
                self.state = RobotState.INIT_ESCAPE  # or SEEK_AND_COLLECT if you want to skip escape

            return

        # -------------------------
        # INIT ESCAPE
        # -------------------------
        if self.state == RobotState.INIT_ESCAPE:
            if self.behavior is None:
                self.behavior = InitEscape()
                self.behavior.start(
                    config=CONFIG,
                    motion_backend=self.motion_backend
                )

            status = self.behavior.update(
                lvl2=self.lvl2,
                localisation=self.localisation,
                motion_backend=self.motion_backend
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
                self.behavior = AcquireObject()
                self.behavior.start(
                    config=CONFIG,
                    kind=CONFIG.default_target_kind,
                    exclude_ids=self.delivered_ids,  # NEW: don’t re-select delivered markers
                )

            status = self.behavior.update(
                lvl2=self.lvl2,
                perception=self.perception,
                localisation=self.localisation,
                motion_backend=self.motion_backend
            )

            if status.name == "SUCCEEDED":
                collected_id = getattr(self.behavior, "acquired_id", None)
                self.last_collected_id = collected_id
                print(f"Marker collected (id={collected_id})")

                self.behavior = None
                self.state = RobotState.POST_PICKUP_REALIGN

            elif status.name == "FAILED":
                print("AcquireObject failed — retrying")
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
                    motion_backend=self.motion_backend,
                    localisation=self.localisation,
                )

            status = self.behavior.update(
                perception=self.perception,
                localisation=self.localisation,
                motion_backend=self.motion_backend,
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
                    motion_backend=self.motion_backend
                )

            status = self.behavior.update(
                perception=self.perception,
                localisation=self.localisation,
                motion_backend=self.motion_backend,
            )

            if status.name == "SUCCEEDED":
                print("Localisation recovered")
                self.behavior = None
                self.state = RobotState.RETURN_TO_BASE

            elif status.name == "FAILED":
                print("Localisation recovery failed — resuming search")
                self.behavior = None
                self.state = RobotState.SEEK_AND_COLLECT

            return

        # -------------------------
        # DELIVER OBJECT
        # -------------------------
        if self.state == RobotState.RETURN_TO_BASE:
            if self.behavior is None:
                self.behavior = DeliverObject()
                self.behavior.start(
                    config=CONFIG,
                    delivered_target_id=self.last_collected_id,  # NEW
                )

            status = self.behavior.update(
                lvl2=self.lvl2,
                motion_backend=self.motion_backend,
            )

            if status.name == "SUCCEEDED":
                delivered_id = getattr(self.behavior, "delivered_target_id", None)
                if delivered_id is None:
                    delivered_id = self.last_collected_id

                if delivered_id is not None:
                    self.delivered_ids.add(delivered_id)
                    print(f"Delivered id={delivered_id} (delivered_ids={sorted(self.delivered_ids)})")

                print("DeliverObject complete — post-dropoff realign")
                self.behavior = None
                self.state = RobotState.POST_DROPOFF_REALIGN


            elif status.name == "FAILED":
                print("DeliverObject failed — resuming seek")
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
                    motion_backend=self.motion_backend,
                )

            status = self.behavior.update(
                motion_backend=self.motion_backend,
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
