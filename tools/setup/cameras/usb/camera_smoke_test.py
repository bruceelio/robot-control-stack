# tools/setup/cameras/usb/camera_smoke_test.py

#!/usr/bin/env python3

"""Smoke test for USB/UVC cameras using OpenCV + V4L2.

Purpose:
- confirm the USB camera opens;
- request a capture format, resolution, and frame rate;
- report the actual values returned by OpenCV/V4L2;
- capture frames successfully;
- optionally display or save a frame.

This is a camera setup tool, not a robot runtime diagnostic.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2



def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="USB camera smoke test using OpenCV/V4L2"
    )
    parser.add_argument(
        "--device",
        required=True,
        help="V4L2 camera device path",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument(
        "--format",
        default="MJPG",
        help="Requested FOURCC capture format, e.g. MJPG or YUYV",
    )
    parser.add_argument(
        "--preview",
        choices=["none", "window", "save"],
        default="none",
        help="none = capture and exit, window = live OpenCV preview, save = save one frame",
    )
    parser.add_argument(
        "--save-path",
        default="usb_camera_smoke_test.jpg",
        help="Output filename used with --preview save",
    )
    parser.add_argument(
        "--settle-frames",
        type=int,
        default=10,
        help="Frames to discard before reporting success",
    )
    return parser


def fourcc_to_text(value: float) -> str:
    number = int(value)
    chars = [
        chr((number >> (8 * i)) & 0xFF)
        for i in range(4)
    ]
    return "".join(chars).rstrip("\x00")


def open_camera(args: argparse.Namespace) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)

    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera: {args.device}")

    fourcc = cv2.VideoWriter_fourcc(*args.format[:4])
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    return cap


def print_state(cap: cv2.VideoCapture, args: argparse.Namespace) -> None:
    actual_width = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
    actual_height = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    actual_fourcc = fourcc_to_text(cap.get(cv2.CAP_PROP_FOURCC))

    print("\n=== USB CAMERA SMOKE TEST ===")
    print(f"Device: {args.device}")

    print("\nRequested:")
    print(f"  Format:     {args.format}")
    print(f"  Resolution: {args.width} x {args.height}")
    print(f"  FPS:        {args.fps:g}")

    print("\nReported by OpenCV/V4L2:")
    print(f"  Format:     {actual_fourcc or 'unknown'}")
    print(f"  Resolution: {actual_width} x {actual_height}")
    print(f"  FPS:        {actual_fps:g}")


def capture_settled_frame(
    cap: cv2.VideoCapture,
    settle_frames: int,
) -> object:
    frame = None

    for index in range(max(1, settle_frames)):
        ok, frame = cap.read()
        if not ok or frame is None:
            raise RuntimeError(
                f"Frame capture failed while settling "
                f"(frame {index + 1}/{max(1, settle_frames)})"
            )

    return frame


def main() -> int:
    args = build_parser().parse_args()

    if len(args.format) != 4:
        print("[ERROR] --format must be a four-character FOURCC such as MJPG or YUYV")
        return 2

    cap = None

    try:
        cap = open_camera(args)
        print_state(cap, args)

        frame = capture_settled_frame(cap, args.settle_frames)
        height, width = frame.shape[:2]

        print("\nCapture:")
        print(f"  Frame received successfully: {width} x {height}")
        print(f"  Channels: {1 if frame.ndim == 2 else frame.shape[2]}")
        print("  Status: OK")

        if args.preview == "save":
            out_path = Path(args.save_path)
            if not cv2.imwrite(str(out_path), frame):
                raise RuntimeError(f"Could not save frame to {out_path}")
            print(f"\nSaved frame: {out_path}")

        elif args.preview == "window":
            print("\nLive preview: press q or Esc to quit.")
            while True:
                ok, frame = cap.read()
                if not ok or frame is None:
                    raise RuntimeError("Frame capture failed during preview")

                cv2.imshow("USB Camera Smoke Test", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break

        return 0

    except RuntimeError as exc:
        print(f"\n[ERROR] {exc}")
        return 1

    finally:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())