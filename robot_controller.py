# robot_controller.py

from level2_canonical import Level2
from perception import Perception, sense
from navigation.localisation import Localisation
from state_machine import RobotState

from behaviors.init_escape import InitEscape
from behaviors.seek_and_collect import SeekAndCollect
from behaviors.post_pickup_realign import PostPickupRealign
from behaviors.recover_localisation import RecoverLocalisation
from behaviors.return_to_base import ReturnToBase

from motion_backends import create_motion_backend
from config import CONFIG
from config.strategy import RUN_MODE, RunMode
from calibration.resolve import resolve
from hw_io.base import IOMap
from hw_io.resolve import resolve_io


from calibration import CALIBRATION
print("\n=== CALIBRATION CAMERA CHECK ===")
print("Calibration cameras:", CALIBRATION.cameras.keys())
print("=== END CALIBRATION CHECK ===\n")

try:
    from tests.runner import run_tests
except ImportError:
    run_tests = None


class Controller:
    """
    Central coordinator.

    Decides WHICH behavior runs.
    Does NOT perform robot actions directly.
    """

    def __init__(self, robot):
        self.robot = robot

        # --------------------------------------------------
        # TEMP DEBUG — CAMERA SANITY CHECK
        # --------------------------------------------------
        print("\n=== CAMERA SANITY CHECK ===")
        print("robot.camera:", getattr(robot, "camera", None))

        if hasattr(robot, "camera") and robot.camera is not None:
            print("robot.camera type:", type(robot.camera))
            try:
                seen = robot.camera.see()
                print(f"robot.camera.see() OK — saw {len(seen)} markers")
            except Exception as e:
                print("robot.camera.see() FAILED:", e)
        else:
            print("NO camera attribute on robot")

        print("=== END CAMERA CHECK ===\n")

        # -------------------------
        # Core subsystems
        # -------------------------

        # IO layer (single source of hardware truth) — create this FIRST
        self.io: IOMap = resolve_io(
            robot=robot,
            hardware_profile=CONFIG.hardware_profile,
        )

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
        # State & behavior
        # -------------------------
        self.state = RobotState.INIT_ESCAPE
        self.behavior = None

    # --------------------------------------------------
    # Main loop
    # --------------------------------------------------

    def run(self):
        # ----------------------------------
        # PRINT RESOLVED CONFIG (ONCE)
        # ----------------------------------
        CONFIG.dump()

        # ----------------------------------
        # EXECUTION MODE DISPATCH
        # ----------------------------------
        if RUN_MODE == RunMode.TESTS:
            if run_tests is None:
                raise RuntimeError("Test runner not available")

            print("\n=== RUNNING TESTS MODE ===")
            run_tests(robot=self.robot)
            print("=== TESTS COMPLETE ===\n")
            return

        if RUN_MODE == RunMode.DIAGNOSTICS:
            print("\n=== RUNNING DIAGNOSTICS MODE ===")
            from diagnostics.runner import run_diagnostics
            run_diagnostics(self.robot)
            print("=== DIAGNOSTICS COMPLETE ===\n")
            return

        # ----------------------------------
        # NORMAL ROBOT OPERATION
        # ----------------------------------
        while True:
            self.update()

    # --------------------------------------------------
    # Per-tick update
    # --------------------------------------------------

    def update(self):
        # Always sense first
        pose, objects = sense(self.io, self.perception)

        if pose is not None:
            x, y, heading = pose
            self.localisation.set_pose((x, y), heading)
        else:
            self.localisation.invalidate()

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
        # SEEK & COLLECT
        # -------------------------
        if self.state == RobotState.SEEK_AND_COLLECT:
            if self.behavior is None:
                self.behavior = SeekAndCollect()
                self.behavior.start(
                    config=CONFIG,
                    kind=CONFIG.default_target_kind,
                )

            status = self.behavior.update(
                lvl2=self.lvl2,
                perception=self.perception,
                localisation=self.localisation,
                motion_backend=self.motion_backend
            )

            if status.name == "SUCCEEDED":
                print("Marker collected")
                self.behavior = None
                self.state = RobotState.POST_PICKUP_REALIGN

            elif status.name == "FAILED":
                print("SeekAndCollect failed — retrying")
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
        # RETURN TO BASE
        # -------------------------
        if self.state == RobotState.RETURN_TO_BASE:
            if self.behavior is None:
                self.behavior = ReturnToBase()
                self.behavior.start(
                    localisation=self.localisation,
                    motion_backend=self.motion_backend,
                )

            status = self.behavior.update(
                lvl2=self.lvl2,
                localisation=self.localisation,
                motion_backend=self.motion_backend,
            )

            if status.name == "SUCCEEDED":
                print("ReturnToBase complete — resuming seek")
                self.behavior = None
                self.state = RobotState.SEEK_AND_COLLECT

            return
