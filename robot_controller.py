# robot_controller.py

import math
import time

from level2.level2_canonical import Level2
from perception import Perception, sense
from localisation import Localisation


from motion_backends import create_motion_backend

from config import CONFIG
from config.strategy import RUN_MODE, RunMode

from config.strategy import CHALLENGE
from config.arena import get_start_pose
from config.strategy import START_SLOT
from config.strategy import AUTONOMOUS_PROGRAM

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

from tools.challenges.runner import run_challenge
from autonomous.runner import run_autonomous

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
        self.signals = make_signals()
        self.encoder_manager = EncoderManager(
            CONFIG.encoders,
            CONFIG.encoder_sign,
        )

        self.perf = PerformanceMonitor("main", report_every_s=5.0)

        # -------------------------
        # Core subsystems
        # -------------------------

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

        # Level2 now consumes IO, not robot
        self.lvl2 = Level2(
            self.io,
            max_power=CONFIG.motor_power_max,
            config=CONFIG,
        )

        # --- Simulator-only vacuum / solenoid startup test ---
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

        # Perception now consumes IO, not robot
        self.perception = Perception(self.io)

        self.localisation = Localisation()

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
            f"pose=({start_x:.1f}, {start_y:.1f}, {math.degrees(start_heading):.1f}deg)"
        )

        self._last_loc_method = "startup_config"

        print("[LOC][METHOD] None -> startup_config")
        print(
            f"[LOC] arena=0 pose_obs=NO method=startup_config "
            f"pos_valid=YES hdg_valid=YES "
            f"x={start_x:.1f} y={start_y:.1f} hdg={math.degrees(start_heading):.1f}"
        )

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

        vision_message["markers"] = markers

        return vision_message

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
            from tools.diagnostics.runner import run_diagnostics
            try:
                run_diagnostics(robot=self.robot, io=self.io)
                safe_cue(self.lvl2, BuzzerCue.SUCCESS)
            except Exception:
                safe_cue(self.lvl2, BuzzerCue.ERROR)
                raise
            finally:
                safe_cue(self.lvl2, BuzzerCue.END)

            print("=== DIAGNOSTICS COMPLETE ===\n")
            return

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

        # ----------------------------------
        # NORMAL ROBOT OPERATION
        # ----------------------------------
        try:
            run_autonomous(
                autonomous_program=AUTONOMOUS_PROGRAM,
                controller=self,
            )
        except Exception:
            safe_cue(self.lvl2, BuzzerCue.ERROR)
            raise


    # --------------------------------------------------
    # Per-tick update
    # --------------------------------------------------

    def tick(self):
        tick_start = time.perf_counter()

        try:
            self._tick_impl()
        finally:
            self.perf.record_tick(time.perf_counter() - tick_start)

    # Compatibility wrapper for any existing code which
    # still calls controller.update().
    def update(self):
        self.tick()

    def _tick_impl(self):
        next_tick()

        now_s = time.time()
        self.encoder_manager.update(io=self.io, signals=self.signals)

        # Bind runtime context to backend
        self.motion_backend.localisation = self.localisation
        self.motion_backend.now_s = now_s

        # ----------------------------------
        # Vision
        # ----------------------------------

        vision_message = self._get_vision_message(
            camera_name="front",
            now_s=now_s,
        )

        # ----------------------------------
        # Perception consumes Vision
        # ----------------------------------

        _, objects = sense(
            self.io,
            self.perception,
            latest_vision_message=vision_message,
        )


        # ----------------------------------
        # Localisation independently consumes Vision
        # ----------------------------------

        localisation_now_s = time.time()

        pose_obs = self.localisation.estimate(
            io=self.io,
            vision_message=vision_message,
            now_s=localisation_now_s,
        )

        if pose_obs is not None:
            '''
            print(
                f"[LOC][ACCEPT] src={pose_obs.source} "
                f"x={pose_obs.x:.1f} y={pose_obs.y:.1f} "
                f"hdg={'None' if pose_obs.heading is None else f'{math.degrees(pose_obs.heading):.1f}'} "
                f"conf={pose_obs.confidence:.2f}"
            )
            '''
            self.localisation.accept(pose_obs)

        pose = self.localisation.pose

        if pose is None:
            current_method = "none"
            pos_valid = "NO"
            hdg_valid = "NO"
            x_str = "None"
            y_str = "None"
            hdg_str = "None"
        else:
            current_method = pose.source
            pos_valid = "YES" if pose.position_valid else "NO"
            hdg_valid = "YES" if pose.heading_valid else "NO"
            x_str = f"{pose.x:.1f}"
            y_str = f"{pose.y:.1f}"
            hdg_str = "None" if pose.heading is None else f"{math.degrees(pose.heading):.1f}"

        if current_method != self._last_loc_method:
            print(f"[LOC][METHOD] {self._last_loc_method} -> {current_method}")
            self._last_loc_method = current_method

        '''
        print(
            f"[LOC] arena={len(arena_obs)} "
            f"pose_obs={'YES' if pose_obs else 'NO'} "
            f"method={current_method} "
            f"pos_valid={pos_valid} hdg_valid={hdg_valid} "
            f"x={x_str} y={y_str} hdg={hdg_str}"
        )
        '''


