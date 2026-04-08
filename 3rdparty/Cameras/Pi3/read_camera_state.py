# 3rdparty/Cameras/Pi3/read_camera_state.py
#!/usr/bin/env python3
"""Read and report current Pi Camera 3 state.

Use this for confirmation after setup or for sporadic checks later.
It does not intentionally change your camera configuration beyond opening the
camera in the requested mode.
"""

from __future__ import annotations

import argparse
import json
import time
from typing import Any

from picamera2 import Picamera2


def serialise(value: Any) -> Any:
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, (list, tuple)):
        return [serialise(v) for v in value]
    try:
        return list(value)
    except Exception:
        return str(value)


def frame_duration_us(fps: float) -> int:
    return int(round(1_000_000 / fps))


def grab_fresh_metadata(picam2: Picamera2, frames: int = 6, delay: float = 0.08) -> dict[str, Any]:
    md: dict[str, Any] = {}
    for _ in range(frames):
        md = picam2.capture_metadata()
        time.sleep(delay)
    return md


def main() -> int:
    parser = argparse.ArgumentParser(description="Read current camera properties, controls, and metadata.")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args()

    picam2 = Picamera2()
    fd = frame_duration_us(args.fps)
    config = picam2.create_preview_configuration(
        main={"size": (args.width, args.height), "format": "RGB888"},
        controls={"FrameDurationLimits": (fd, fd)},
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1.0)

    try:
        md = grab_fresh_metadata(picam2)

        print("=" * 88)
        print("CAMERA CONFIGURATION REQUEST")
        print("=" * 88)
        print(json.dumps({
            "width": args.width,
            "height": args.height,
            "fps": args.fps,
            "frame_duration_limits_us": [fd, fd],
        }, indent=2))

        print("\n" + "=" * 88)
        print("PICAMERA2 CONFIG")
        print("=" * 88)
        print(json.dumps({k: serialise(v) for k, v in config.items()}, indent=2))

        print("\n" + "=" * 88)
        print("CAMERA PROPERTIES")
        print("=" * 88)
        print(json.dumps({k: serialise(v) for k, v in picam2.camera_properties.items()}, indent=2))

        print("\n" + "=" * 88)
        print("CAMERA CONTROLS / RANGES")
        print("=" * 88)
        print(json.dumps({k: serialise(v) for k, v in picam2.camera_controls.items()}, indent=2))

        print("\n" + "=" * 88)
        print("CURRENT FRAME METADATA")
        print("=" * 88)
        print(json.dumps({k: serialise(v) for k, v in md.items()}, indent=2))

        print("\n" + "=" * 88)
        print("SHORT SUMMARY")
        print("=" * 88)
        summary_keys = [
            "ExposureTime",
            "AnalogueGain",
            "DigitalGain",
            "ColourGains",
            "ColourTemperature",
            "FrameDuration",
            "LensPosition",
            "AfMode",
            "AfState",
            "AeEnable",
            "AwbEnable",
            "Lux",
            "SensorTimestamp",
        ]
        summary = {k: serialise(md.get(k)) for k in summary_keys if k in md}
        print(json.dumps(summary, indent=2))

        print("\n" + "=" * 88)
        print("VERIFICATION CHECKS")
        print("=" * 88)

        af_mode = md.get("AfMode")
        lens = md.get("LensPosition")
        exposure = md.get("ExposureTime")
        gain = md.get("AnalogueGain")
        ae_enable = md.get("AeEnable")
        awb_enable = md.get("AwbEnable")
        frame_duration = md.get("FrameDuration")

        print("\n[Focus]")
        if af_mode == 0:
            print("AfMode: MANUAL ✅")
        else:
            print(f"AfMode: {af_mode} ⚠️ (expected 0 for manual focus)")
        print(f"LensPosition: {lens}")

        print("\n[Exposure]")
        if ae_enable is False:
            print("Auto Exposure: OFF (manual) ✅")
        else:
            print(f"Auto Exposure: {ae_enable} ⚠️ (expected False for manual exposure)")
        print(f"ExposureTime: {exposure}")
        print(f"AnalogueGain: {gain}")

        print("\n[White Balance]")
        if awb_enable is False:
            print("AWB: LOCKED / MANUAL ✅")
        else:
            print(f"AWB: {awb_enable} ⚠️ (expected False for fixed white balance)")
        print(f"ColourGains: {md.get('ColourGains')}")
        print(f"ColourTemperature: {md.get('ColourTemperature')}")

        print("\n[Timing]")
        if frame_duration is not None:
            approx_fps = round(1_000_000 / frame_duration, 2) if frame_duration else "N/A"
            print(f"FrameDuration: {frame_duration} us (~{approx_fps} FPS)")
        else:
            print("FrameDuration: not reported")

        print("\n[Resolution / Requested Mode]")
        print(f"Requested resolution: {args.width}x{args.height}")
        print(f"Requested fps: {args.fps}")

    finally:
        picam2.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
