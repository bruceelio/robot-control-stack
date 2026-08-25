# tools/setup/cameras/usb/camera_state.py

#!/usr/bin/env python3
"""Report USB/UVC camera state using v4l2-ctl.

Purpose:
- inspect the controls exposed by a USB camera;
- show the current value of each control;
- keep initial camera setup independent of robot configuration.

This is a camera setup tool, not a robot runtime diagnostic.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Report USB camera controls and current V4L2 state"
    )
    parser.add_argument(
        "--device",
        required=True,
        help="V4L2 camera device path, preferably /dev/v4l/by-id/...",
    )
    return parser


def run_v4l2_ctl(device: str, *args: str) -> int:
    command = ["v4l2-ctl", "-d", device, *args]

    result = subprocess.run(
        command,
        text=True,
        capture_output=True,
    )

    if result.stdout:
        print(result.stdout.rstrip())

    if result.stderr:
        print(result.stderr.rstrip(), file=sys.stderr)

    return result.returncode


def main() -> int:
    args = build_parser().parse_args()

    if shutil.which("v4l2-ctl") is None:
        print("[ERROR] v4l2-ctl was not found.")
        print("Install the v4l-utils package before running this tool.")
        return 1

    print("=== USB CAMERA STATE ===")
    print(f"Device: {args.device}")

    print("\n=== DEVICE INFORMATION ===")
    rc = run_v4l2_ctl(args.device, "--all")
    if rc != 0:
        print("[ERROR] Could not read camera state.")
        return rc

    print("\n=== CONTROLS AND RANGES ===")
    rc = run_v4l2_ctl(args.device, "--list-ctrls-menus")
    if rc != 0:
        print("[ERROR] Could not read camera controls.")
        return rc

    return 0


if __name__ == "__main__":
    raise SystemExit(main())