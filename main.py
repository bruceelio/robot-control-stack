from __future__ import annotations

import os
import sys
import time


# ============================================================
# RUNTIME STARTUP POLICY
# ============================================================
#
# These are here on purpose so you do not need to hunt around
# the codebase later to remember how startup currently works.
#
# CURRENT BEHAVIOUR:
# - If running interactively in a terminal, wait for Enter.
# - If running non-interactively (for example via systemd),
#   optionally autostart after a short arming delay.
#
# FUTURE COMPETITION BEHAVIOUR:
# - main.py should be started automatically on boot by systemd
# - robot should initialize into a SAFE/READY state
# - robot should wait for a PHYSICAL START BUTTON
# - optional mode selection (AUTO / TEST / DIAGNOSTICS) can also
#   be done by hardware inputs before the run begins
#
# Until the physical start button is wired and implemented,
# AUTOSTART_WHEN_HEADLESS=True is the simplest practical option.
# Set it to False once a real start button exists.
#
AUTOSTART_WHEN_HEADLESS = True
HEADLESS_AUTOSTART_DELAY_S = 3.0
READY_POLL_INTERVAL_S = 1.0


def config_uses_sr(config) -> bool:
    """
    Decide whether this run needs sr.robot3.Robot().

    CURRENT RULE:
    - existing codebase uses CONFIG.hardware_profile
    - if it is 'sr1', assume we need the SR API object

    FUTURE RULE:
    - replace/extend this with per-subsystem backend inspection,
      for example:
          DRIVE_BACKEND == "sr"
          OUTPUT_BACKEND == "sr"
          any camera backend == "sr"
    """
    hardware_profile = getattr(config, "hardware_profile", None)
    return hardware_profile == "sr1"


def build_robot_if_needed(config):
    """
    Construct the SR Robot object only if the selected configuration
    actually needs it.
    """
    if not config_uses_sr(config):
        print("[MAIN] Native mode selected: no SR Robot() required")
        return None

    print("[MAIN] SR-backed mode selected: creating sr.robot3.Robot()")
    from sr.robot3 import Robot
    return Robot()


def wait_for_start_signal() -> None:
    """
    Wait until the robot should actually begin running.

    CURRENT IMPLEMENTATION:
    - if a terminal is attached, wait for Enter
    - if no terminal is attached (for example systemd), optionally
      autostart after a short delay

    FUTURE IMPLEMENTATION:
    - replace the headless/autostart path with a real physical
      start button, for example:
          while not start_button_pressed():
              sleep(...)
      and optionally add mode selection before starting.
    """
    print("[MAIN] Robot is in READY state")

    # --------------------------------------------------------
    # INTERACTIVE DEVELOPMENT MODE
    # --------------------------------------------------------
    if sys.stdin.isatty():
        print("[MAIN] Interactive terminal detected")
        print("[MAIN] Press Enter to start the robot")
        input()
        print("[MAIN] Start confirmed from terminal")
        return

    # --------------------------------------------------------
    # HEADLESS MODE (for example systemd on boot)
    # --------------------------------------------------------
    print("[MAIN] No interactive terminal detected")

    if AUTOSTART_WHEN_HEADLESS:
        print(
            f"[MAIN] Headless autostart enabled "
            f"(starting in {HEADLESS_AUTOSTART_DELAY_S:.1f}s)"
        )
        time.sleep(HEADLESS_AUTOSTART_DELAY_S)
        print("[MAIN] Headless autostart triggered")
        return

    print("[MAIN] Headless autostart disabled; waiting for future start input")

    while True:
        # ====================================================
        # FUTURE PHYSICAL START BUTTON HOOK
        # ====================================================
        #
        # Example shape later:
        #
        # if start_button_pressed():
        #     print("[MAIN] Physical start button pressed")
        #     return
        #
        # You may also want mode selection here, e.g.:
        # - AUTO
        # - TESTS
        # - DIAGNOSTICS
        #
        # Example future flow:
        # selected_mode = read_mode_selector()
        # apply_selected_mode(selected_mode)
        # if start_button_pressed():
        #     return
        #
        # Until then, this loop just idles safely.
        # ====================================================

        time.sleep(READY_POLL_INTERVAL_S)


def main() -> int:
    """
    Top-level supervisor for the physical robot.

    RESPONSIBILITIES:
    - import config
    - create SR Robot() if needed
    - build Controller
    - enter READY state
    - wait for start signal
    - hand off to controller.run()

    NOT RESPONSIBLE FOR:
    - detailed robot behaviour
    - state machine logic
    - autonomous action sequencing

    Those remain inside robot_controller.py.
    """
    try:
        from config import CONFIG
        from robot_controller import Controller

        print("[MAIN] Starting robot application")
        print(f"[MAIN] PID={os.getpid()}")

        # ----------------------------------------------------
        # Build runtime objects
        # ----------------------------------------------------
        robot = build_robot_if_needed(CONFIG)
        controller = Controller(robot)

        # ----------------------------------------------------
        # READY / ARMING STAGE
        # ----------------------------------------------------
        #
        # At this point the Controller has already initialized
        # hardware and your existing startup safety logic in
        # robot_controller.py has run.
        #
        # FUTURE NOTE:
        # If you later want a dedicated READY buzzer/light cue
        # before the run starts, this is the correct place.
        #
        # Example future additions here:
        # - play "ready" buzzer pattern
        # - flash LED while waiting
        # - show selected profile/mode
        # ----------------------------------------------------
        wait_for_start_signal()

        # ----------------------------------------------------
        # EXECUTION
        # ----------------------------------------------------
        print("[MAIN] Launching controller.run()")
        controller.run()

        print("[MAIN] controller.run() returned normally")
        return 0

    except KeyboardInterrupt:
        print("\n[MAIN] Interrupted by user")
        return 130

    except Exception as exc:
        print(f"[MAIN] Fatal error: {exc}")
        raise


if __name__ == "__main__":
    sys.exit(main())