# 3rdparty/Cameras/Pi3/run_camera_config_tests.py
#!/usr/bin/env python3
"""Run a sequence of Pi Camera 3 test configurations.

Typical use:
    python3 run_camera_config_tests.py --preview drm

This script:
- loads CONFIGS from camera_test_configs.py
- applies each config in turn
- prints the requested and observed camera state
- optionally shows live camera preview on the Pi monitor
- can save one frame per config for later inspection
- pauses on each config so you can run your own AprilTag tests alongside it

It is intended for systematic pre-calibration tuning of exposure/gain/WB, after
resolution/fps have been fixed and after focus selection has been completed.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import cv2
from picamera2 import Picamera2, Preview

from camera_test_configs import CONFIGS


MANUAL = 0


def safe_apply(picam2: Picamera2, controls: dict[str, Any], label: str) -> None:
    try:
        picam2.set_controls(controls)
    except Exception as exc:
        print(f"[WARN] Could not apply {label}: {controls} ({exc})")


def grab_fresh_metadata(picam2: Picamera2, frames: int = 5, delay: float = 0.08) -> dict[str, Any]:
    md: dict[str, Any] = {}
    for _ in range(frames):
        md = picam2.capture_metadata()
        time.sleep(delay)
    return md


def frame_duration_us(fps: float) -> int:
    return int(round(1_000_000 / fps))


def make_camera(width: int, height: int, fps: int, preview_mode: str) -> Picamera2:
    picam2 = Picamera2()
    fd = frame_duration_us(fps)
    config = picam2.create_preview_configuration(
        main={"size": (width, height), "format": "RGB888"},
        controls={"FrameDurationLimits": (fd, fd)},
    )
    picam2.configure(config)

    if preview_mode == "drm":
        picam2.start_preview(Preview.DRM)

    picam2.start()
    time.sleep(1.0)
    return picam2


def lock_awb_after_settle(picam2: Picamera2, settle_s: float) -> tuple[Any, Any]:
    safe_apply(picam2, {"AwbEnable": True}, "enable AWB")
    time.sleep(settle_s)
    md = grab_fresh_metadata(picam2)
    colour_gains = md.get("ColourGains")
    colour_temp = md.get("ColourTemperature")
    if colour_gains is not None:
        safe_apply(picam2, {"AwbEnable": False, "ColourGains": colour_gains}, "lock AWB")
        time.sleep(0.4)
    else:
        print("[WARN] AWB did not report ColourGains; WB may remain automatic.")
    return colour_gains, colour_temp


def apply_manual_colour_gains(picam2: Picamera2, colour_gains: tuple[float, float]) -> None:
    safe_apply(picam2, {"AwbEnable": False, "ColourGains": colour_gains}, "manual colour gains")
    time.sleep(0.4)


def apply_manual_exposure_gain(picam2: Picamera2, shutter_us: int, analogue_gain: float) -> None:
    safe_apply(picam2, {"AeEnable": False}, "disable AE")
    safe_apply(
        picam2,
        {"ExposureTime": int(shutter_us), "AnalogueGain": float(analogue_gain)},
        "manual exposure/gain",
    )
    time.sleep(0.5)


def apply_fixed_focus(picam2: Picamera2, lens_position: float) -> None:
    safe_apply(picam2, {"AfMode": MANUAL, "LensPosition": float(lens_position)}, "fixed focus")
    time.sleep(0.5)


def summarise(md: dict[str, Any]) -> dict[str, Any]:
    keys = [
        "ExposureTime",
        "AnalogueGain",
        "DigitalGain",
        "ColourGains",
        "ColourTemperature",
        "FrameDuration",
        "LensPosition",
        "AfMode",
        "AfState",
        "Lux",
    ]
    return {k: md.get(k) for k in keys if k in md}


def preview_loop(
    picam2: Picamera2,
    hold_s: float,
    preview_mode: str,
    save_dir: Path,
    config_name: str,
) -> None:
    if preview_mode == "none":
        time.sleep(hold_s)
        return

    if preview_mode == "drm":
        print(f"Holding config for {hold_s:.1f} seconds with DRM preview on Pi monitor...")
        time.sleep(hold_s)
        return

    if preview_mode == "save":
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = save_dir / f"{config_name}_{timestamp}.png"
        cv2.imwrite(str(filename), frame_bgr)
        print(f"[SAVE] Wrote {filename}")
        time.sleep(hold_s)
        return


def run_config(
    config: dict[str, Any],
    hold_s: float,
    preview_mode: str,
    save_dir: Path,
) -> None:
    print("\n" + "=" * 88)
    print(f"Running config: {config['name']}")
    print(json.dumps(config, indent=2))

    picam2 = make_camera(config["width"], config["height"], config["fps"], preview_mode)
    try:
        apply_manual_exposure_gain(picam2, config["shutter_us"], config["analogue_gain"])

        if config.get("awb_lock_after_settle", False):
            gains, temp = lock_awb_after_settle(picam2, float(config.get("awb_settle_s", 2.0)))
            print(f"Locked AWB with ColourGains={gains}, ColourTemperature={temp}")
        elif config.get("colour_gains") is not None:
            apply_manual_colour_gains(picam2, tuple(config["colour_gains"]))
            print(f"Applied manual ColourGains={config['colour_gains']}")
        else:
            print("Leaving white balance unchanged for this config.")

        if config.get("fixed_lens_position") is not None:
            apply_fixed_focus(picam2, float(config["fixed_lens_position"]))
            print(f"Applied fixed LensPosition={config['fixed_lens_position']}")
        else:
            print("Focus not changed by this runner.")

        md = grab_fresh_metadata(picam2)
        print("Observed metadata summary:")
        print(json.dumps(summarise(md), indent=2))

        preview_loop(
            picam2,
            hold_s=hold_s,
            preview_mode=preview_mode,
            save_dir=save_dir,
            config_name=str(config["name"]).replace(" ", "_"),
        )

    finally:
        picam2.stop()


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a list of Pi Camera test configurations.")
    parser.add_argument("--hold-s", type=float, default=12.0, help="Seconds to hold each config")
    parser.add_argument(
        "--preview",
        choices=["drm", "none", "save"],
        default="none",
        help="Preview mode: drm = Pi monitor, none = no preview, save = save one frame per config",
    )
    parser.add_argument(
        "--save-dir",
        type=str,
        default="camera_config_debug",
        help="Directory for saved frames when --preview save",
    )
    args = parser.parse_args()

    save_dir = Path(args.save_dir)
    if args.preview == "save":
        save_dir.mkdir(parents=True, exist_ok=True)

    print("Loaded configs:")
    for cfg in CONFIGS:
        print(f"- {cfg['name']}")

    try:
        for cfg in CONFIGS:
            run_config(
                cfg,
                hold_s=args.hold_s,
                preview_mode=args.preview,
                save_dir=save_dir,
            )
    except KeyboardInterrupt:
        print("\nStopped by user.")
        return 0

    print("\nFinished all configs.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())