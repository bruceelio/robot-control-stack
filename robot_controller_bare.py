# robot_controller_bare.py

import math
import time

from level2.level2_canonical import Level2
from perception import Perception, sense
from localisation import Localisation

from motion_backends import create_motion_backend

from config import CONFIG
from config.strategy import RUN_MODE, RunMode
from config.strategy import CHALLENGE
from config.strategy import START_SLOT
from config.arena import get_start_pose

from calibration import CALIBRATION
from calibration.resolve import resolve
from perception.vision.detection_pipeline import build_vision_message

from hw_io.base import IOMap
from hw_io.resolve import resolve_io
from hw_io.encoder_manager import EncoderManager, make_signals

from log_trace import next_tick
from hw_io.buzzer_patterns import BuzzerCue
from hw_io.cameras.camera_process import CameraProcessManager
from tools.perf_monitor import PerformanceMonitor


try:
    from tools.tests.runner import run_tests
except ImportError:
    run_tests = None

try:
    from challenges.runner import run_challenge
except ImportError:
    run_challenge = None


def safe_cue(lvl2, cue: BuzzerCue) -> None:
    """Play a status cue when available and always print it."""
    print(f"[CUE] {cue.value}")

    try:
        lvl2.patterns.cue(cue)
    except Exception:
        pass


class Controller:
    """
    Bare-bones robot controller.

    Common robot infrastructure is initialised here, but no supplied
    autonomous competition behaviour is included.

    NORMAL mode keeps the core robot services updated and then calls
    autonomous_update(), which is intentionally empty for student code.

    CHALLENGES, TESTS and DIAGNOSTICS retain their normal runners.
    """

    def __init__(self, robot):
        self.robot = robot

        # --------------------------------------------------
        # Encoder signals
        # --------------------------------------------------

        self.signals = make_signals()
        self.encoder_manager = EncoderManager(CONFIG.encoders)

        # --------------------------------------------------
        # Performance monitoring
        # --------------------------------------------------

        self.perf = PerformanceMonitor(
            "main",
            report_every_s=5.0,
        )

        # --------------------------------------------------
        # Cameras / IO
        # --------------------------------------------------

        if CONFIG.async_vision_enabled:
            self.camera_manager = CameraProcessManager(
                camera_names=list(CONFIG.cameras.keys()),
                robot=self.robot,
            )
        else:
            self.camera_manager = None

        self.io: IOMap = resolve_io(
            robot=robot,
            camera_manager=self.camera_manager,
        )

        if self.camera_manager is not None:
            self.camera_manager.start()
            print("[CAMERA_PROCESS] manager started")
        else:
            print("[CAMERA_PROCESS] async vision disabled")

        # --------------------------------------------------
        # Level2
        # --------------------------------------------------

        self.lvl2 = Level2(
            self.io,
            max_power=CONFIG.max_motor_power,
        )

        # --------------------------------------------------
        # Simulator startup checkout
        # --------------------------------------------------

        if CONFIG.environment == "simulation":
            outs = getattr(self.io, "outputs", None)

            if outs is not None:
                print("[BOOT][SIM] Forcing VACUUM OFF at startup")
                outs.set("VACUUM", False)
                self.io.sleep(0.25)

                print("[BOOT][SIM] VACUUM ON test pulse")
                outs.set("VACUUM", True)
                self.io.sleep(0.50)

                print("[BOOT][SIM] Forcing VACUUM OFF at startup")
                outs.set("VACUUM", False)
                self.io.sleep(0.25)
            else:
                print("[BOOT][SIM] No io.outputs available")
        else:
            print("[BOOT] Skipping vacuum startup test outside simulation")

        # --------------------------------------------------
        # Perception / Localisation
        # --------------------------------------------------

        self.perception = Perception(self.io)
        self.localisation = Localisation()

        # Current runtime data exposed to student autonomous code.
        self.objects = []
        self.pose = None
        self.vision_message = None

        # --------------------------------------------------
        # Starting pose
        # --------------------------------------------------

        if CONFIG.io.get("usb.match_zone") is not None:
            match_zone = self.io.usb["match_zone"]
        else:
            match_zone = 0

        start_x, start_y, start_heading = get_start_pose(
            match_zone,
            START_SLOT,
        )

        self.localisation.set_pose(
            (start_x, start_y),
            heading=start_heading,
            source="startup_config",
            timestamp=time.time(),
        )

        print(
            f"[LOC][START] zone={match_zone} slot={START_SLOT} "
            f"pose=({start_x:.1f}, {start_y:.1f}, "
            f"{math.degrees(start_heading):.1f}deg)"
        )

        self._last_loc_method = "startup_config"
        print("[LOC][METHOD] None -> startup_config")

        # --------------------------------------------------
        # Configuration / Calibration
        # --------------------------------------------------

        self.config = CONFIG
        self.calibration = resolve(config=CONFIG)

        # --------------------------------------------------
        # Motion backend
        # --------------------------------------------------

        self.motion_backend = create_motion_backend(
            CONFIG.motion_backend,
            self.lvl2,
            self.config,
            self.calibration,
        )

    # --------------------------------------------------
    # Vision
    # --------------------------------------------------

    def _get_vision_message(
        self,
        *,
        camera_name: str,
        now_s: float,
    ) -> dict | None:

        # Asynchronous physical-camera path.
        if self.camera_manager is not None:
            return self.camera_manager.get_latest(camera_name)

        # Synchronous path, e.g. SR/Webots.
        camera = self.io.cameras().get(camera_name)

        if camera is None:
            return None

        markers = list(camera.see())

        cam_cal = CALIBRATION.cameras[camera_name]

        vision_message = build_vision_message(
            camera_name=camera_name,
            timestamp=now_s,
            markers=markers,
            cam_cal=cam_cal,
            camera_yaw_deg=float(
                CONFIG.camera_mounts[camera_name]["yaw_deg"]
            ),
        )

        # Retain raw markers for localisation providers which require
        # AprilTag information beyond the simplified detections.
        vision_message["markers"] = markers

        return vision_message

    # --------------------------------------------------
    # Run-mode dispatch
    # --------------------------------------------------

    def run(self):
        CONFIG.dump()
        safe_cue(self.lvl2, BuzzerCue.START)

        # --------------------------------------------------
        # TESTS
        # --------------------------------------------------

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

        # --------------------------------------------------
        # DIAGNOSTICS
        # --------------------------------------------------

        if RUN_MODE == RunMode.DIAGNOSTICS:
            print("\n=== RUNNING DIAGNOSTICS MODE ===")

            from tools.diagnostics.runner import run_diagnostics

            try:
                run_diagnostics(
                    robot=self.robot,
                    io=self.io,
                )
                safe_cue(self.lvl2, BuzzerCue.SUCCESS)
            except Exception:
                safe_cue(self.lvl2, BuzzerCue.ERROR)
                raise
            finally:
                safe_cue(self.lvl2, BuzzerCue.END)

            print("=== DIAGNOSTICS COMPLETE ===\n")
            return

        # --------------------------------------------------
        # CHALLENGES
        # --------------------------------------------------

        if RUN_MODE == RunMode.CHALLENGES:
            if run_challenge is None:
                safe_cue(self.lvl2, BuzzerCue.ERROR)
                raise RuntimeError("Challenge runner not available")

            print(f"\n=== RUNNING CHALLENGE: {CHALLENGE.name} ===")

            try:
                run_challenge(
                    challenge=CHALLENGE,
                    controller=self,
                )
                safe_cue(self.lvl2, BuzzerCue.SUCCESS)
            except Exception:
                safe_cue(self.lvl2, BuzzerCue.ERROR)
                raise
            finally:
                safe_cue(self.lvl2, BuzzerCue.END)

            print("=== CHALLENGE COMPLETE ===\n")
            return

        # --------------------------------------------------
        # NORMAL
        # --------------------------------------------------
        #
        # NORMAL deliberately contains no supplied autonomous behaviour.
        # The controller only keeps common robot services updated and
        # calls autonomous_update().
        # --------------------------------------------------

        print("\n=== RUNNING NORMAL MODE ===")

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
        tick_start = time.perf_counter()

        try:
            self._update_core()
            self.autonomous_update()
        finally:
            self.perf.record_tick(
                time.perf_counter() - tick_start
            )

    def _update_core(self):
        """
        Update common robot services.

        This is infrastructure only. It does not choose or execute
        autonomous competition behaviours.
        """

        next_tick()

        now_s = time.time()

        # --------------------------------------------------
        # Encoders
        # --------------------------------------------------

        self.encoder_manager.update(
            io=self.io,
            signals=self.signals,
        )

        # --------------------------------------------------
        # Motion backend runtime context
        # --------------------------------------------------

        self.motion_backend.localisation = self.localisation
        self.motion_backend.now_s = now_s

        # --------------------------------------------------
        # Vision
        # --------------------------------------------------

        self.vision_message = self._get_vision_message(
            camera_name="front",
            now_s=now_s,
        )

        # --------------------------------------------------
        # Perception consumes Vision
        # --------------------------------------------------

        _, self.objects = sense(
            self.io,
            self.perception,
            latest_vision_message=self.vision_message,
        )

        # --------------------------------------------------
        # Localisation independently consumes Vision
        # --------------------------------------------------

        localisation_now_s = time.time()

        pose_obs = self.localisation.estimate(
            vision_message=self.vision_message,
            now_s=localisation_now_s,
        )

        if pose_obs is not None:
            self.localisation.accept(pose_obs)

        self.pose = self.localisation.pose

        if self.pose is None:
            current_method = "none"
        else:
            current_method = self.pose.source

        if current_method != self._last_loc_method:
            print(
                f"[LOC][METHOD] "
                f"{self._last_loc_method} -> {current_method}"
            )
            self._last_loc_method = current_method

    # --------------------------------------------------
    # Student autonomous code
    # --------------------------------------------------

    def autonomous_update(self):
        """
        NORMAL-mode autonomous hook.

        The common robot stack has already been initialised and updated
        before this method is called.

        Available interfaces include:

            self.io
            self.lvl2
            self.motion_backend

            self.vision_message
            self.perception
            self.objects

            self.localisation
            self.pose

            self.config
            self.calibration

            self.signals

        Students can implement their autonomous logic here, or replace
        this method with their own preferred architecture.
        """

        # TODO: student autonomous code
        pass
