# tools/setup/cameras/usb/camera_configure.py

#!/usr/bin/env python3
"""Configure USB/UVC camera controls using v4l2-ctl.

Purpose:
- apply selected V4L2 controls during initial USB camera setup;
- keep setup independent of robot configuration;
- print the resulting control values after applying them.

Example:
    python3 tools/setup/cameras/usb/camera_configure.py \
        --device /dev/v4l/by-id/... \
        --set auto_exposure=1 \
        --set exposure_time_absolute=45 \
        --set gain=0

This is a camera setup tool, not a robot runtime diagnostic.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Apply USB camera V4L2 controls"
    )
    parser.add_argument(
        "--device",
        required=True,
        help="V4L2 camera device path, preferably /dev/v4l/by-id/...",
    )
    parser.add_argument(
        "--set",
        dest="controls",
        action="append",
        default=[],
        metavar="CONTROL=VALUE",
        help="Control assignment. Repeat for multiple controls.",
    )
    return parser


def parse_controls(items: list[str]) -> list[tuple[str, str]]:
    controls: list[tuple[str, str]] = []

    for item in items:
        if "=" not in item:
            raise ValueError(f"Invalid control assignment: {item!r}")

        name, value = item.split("=", 1)
        name = name.strip()
        value = value.strip()

        if not name or not value:
            raise ValueError(f"Invalid control assignment: {item!r}")

        controls.append((name, value))

    return controls


def run_command(command: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        text=True,
        capture_output=True,
    )


def main() -> int:
    args = build_parser().parse_args()

    if shutil.which("v4l2-ctl") is None:
        print("[ERROR] v4l2-ctl was not found.")
        print("Install the v4l-utils package before running this tool.")
        return 1

    try:
        controls = parse_controls(args.controls)
    except ValueError as exc:
        print(f"[ERROR] {exc}")
        return 2

    if not controls:
        print("[ERROR] No controls supplied.")
        print("Use one or more --set CONTROL=VALUE arguments.")
        return 2

    set_expression = ",".join(f"{name}={value}" for name, value in controls)

    print("=== USB CAMERA CONFIGURE ===")
    print(f"Device: {args.device}")

    print("\nApplying:")
    for name, value in controls:
        print(f"  {name} = {value}")

    result = run_command([
        "v4l2-ctl",
        "-d",
        args.device,
        "--set-ctrl",
        set_expression,
    ])

    if result.stdout:
        print(result.stdout.rstrip())

    if result.returncode != 0:
        if result.stderr:
            print(result.stderr.rstrip(), file=sys.stderr)
        print("[ERROR] Failed to apply one or more controls.")
        return result.returncode

    control_names = ",".join(name for name, _ in controls)

    print("\nApplied values:")
    result = run_command([
        "v4l2-ctl",
        "-d",
        args.device,
        "--get-ctrl",
        control_names,
    ])

    if result.stdout:
        print(result.stdout.rstrip())

    if result.returncode != 0:
        if result.stderr:
            print(result.stderr.rstrip(), file=sys.stderr)
        print("[ERROR] Controls were set, but verification failed.")
        return result.returncode

    print("\nStatus: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())