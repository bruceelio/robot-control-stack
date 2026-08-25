# tools/setup/cameras/camera_fov_check.py

# tools/setup/cameras/camera_fov_check.py

#!/usr/bin/env python3
"""Visual field-of-view check for cameras during initial setup.

Purpose:
- capture the actual image produced by a selected camera/interface;
- display or save that image so scene boundaries can be inspected;
- keep FOV verification separate from robot runtime configuration.

This tool does not calculate angular FOV. It is intended to answer the practical
question: "What scene is this camera/mode actually showing me?"

Supported backends:
- usb : USB/UVC camera through OpenCV + V4L2
- pi  : Pi CSI camera through Picamera2/libcamera
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Visually inspect the field of view produced by a camera"
    )

    parser.add_argument(
        "--backend",
        required=True,
        choices=["usb", "pi"],
        help="Camera interface/backend",
    )

    parser.add_argument(
        "--device",
        help="USB V4L2 device path. Required when --backend usb.",
    )

    parser.add_argument("--capture-width", type=int, required=True)
    parser.add_argument("--capture-height", type=int, required=True)
    parser.add_argument("--processing-width", type=int, required=True)
    parser.add_argument("--processing-height", type=int, required=True)
    parser.add_argument("--fps", type=float, default=30.0)

    parser.add_argument(
        "--format",
        default="MJPG",
        help="USB FOURCC format, e.g. MJPG or YUYV. Ignored for Pi.",
    )

    parser.add_argument(
        "--raw-width",
        type=int,
        help="Optional Pi raw/sensor-mode width.",
    )
    parser.add_argument(
        "--raw-height",
        type=int,
        help="Optional Pi raw/sensor-mode height.",
    )

    parser.add_argument(
        "--view",
        choices=["window", "save"],
        default="save",
        help="Display live view or save a still frame",
    )

    parser.add_argument(
        "--save-path",
        default="camera_fov_check.jpg",
        help="Filename used with --view save",
    )

    parser.add_argument(
        "--settle-frames",
        type=int,
        default=10,
        help="USB frames to discard before using an image",
    )

    return parser


def fourcc_to_text(value: float) -> str:
    number = int(value)
    return "".join(
        chr((number >> (8 * i)) & 0xFF)
        for i in range(4)
    ).rstrip("\x00")


def run_usb(args: argparse.Namespace) -> int:
    if not args.device:
        print("[ERROR] --device is required when --backend usb")
        return 2

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)

    if not cap.isOpened():
        print(f"[ERROR] Could not open camera: {args.device}")
        return 1

    try:
        if len(args.format) != 4:
            print("[ERROR] --format must be a four-character FOURCC such as MJPG or YUYV")
            return 2

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.format))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.capture_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.capture_height)
        cap.set(cv2.CAP_PROP_FPS, args.fps)

        actual_width = int(round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
        actual_height = int(round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        actual_format = fourcc_to_text(cap.get(cv2.CAP_PROP_FOURCC))

        print("=== CAMERA FOV CHECK ===")
        print("Backend: USB / V4L2 / OpenCV")
        print(f"Device: {args.device}")
        print(f"Requested: {args.capture_width} x {args.capture_height} @ {args.fps:g} FPS, {args.format}")
        print(
            f"Reported:  {actual_width} x {actual_height} "
            f"@ {actual_fps:g} FPS, {actual_format or 'unknown'}"
        )

        frame = None
        for index in range(max(1, args.settle_frames)):
            ok, frame = cap.read()
            if not ok or frame is None:
                print(
                    f"[ERROR] Frame capture failed "
                    f"({index + 1}/{max(1, args.settle_frames)})"
                )
                return 1

        if args.view == "save":
            output_dir = Path("camera_fov_check")
            output_dir.mkdir(parents=True, exist_ok=True)

            native_path = output_dir / "camera_fov_native.jpg"
            processed_path = output_dir / "camera_fov_processed.jpg"

            processed = cv2.resize(
                frame,
                (args.processing_width, args.processing_height),
                interpolation=cv2.INTER_AREA,
            )

            if not cv2.imwrite(str(native_path), frame):
                print(f"[ERROR] Could not save image: {native_path}")
                return 1
            if not cv2.imwrite(str(processed_path), processed):
                print(f"[ERROR] Could not save image: {processed_path}")
                return 1

            native_ratio = args.capture_width / args.capture_height
            processed_ratio = args.processing_width / args.processing_height

            print(f"Saved native image:    {native_path}")
            print(f"Saved processed image: {processed_path}")
            print(f"Native aspect:    {native_ratio:.4f}")
            print(f"Processed aspect: {processed_ratio:.4f}")

            if abs(native_ratio - processed_ratio) < 1e-9:
                print("Aspect ratios match; this resize does not introduce cropping.")
            else:
                print("[WARN] Aspect ratios differ; geometry is being changed.")

            print("Compare the left, right, top, and bottom scene boundaries.")
            return 0

        print("Live FOV view: press q or Esc to quit.")
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("[ERROR] Frame capture failed during live view.")
                return 1

            cv2.imshow("Camera FOV Check", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                return 0

    finally:
        cap.release()
        cv2.destroyAllWindows()


def run_pi(args: argparse.Namespace) -> int:
    try:
        from picamera2 import Picamera2
    except ImportError:
        print("[ERROR] Picamera2 is not available in this Python environment.")
        return 1

    if (args.raw_width is None) != (args.raw_height is None):
        print("[ERROR] Supply both --raw-width and --raw-height, or neither.")
        return 2

    picam2 = Picamera2()

    try:
        frame_duration = int(round(1_000_000 / args.fps))

        kwargs = {
            "main": {
                "size": (args.capture_width, args.capture_height),
                "format": "RGB888",
            },
            "controls": {
                "FrameDurationLimits": (frame_duration, frame_duration),
            },
        }

        if args.raw_width is not None:
            kwargs["raw"] = {
                "size": (args.raw_width, args.raw_height),
            }

        config = picam2.create_preview_configuration(**kwargs)
        picam2.configure(config)
        picam2.start()
        time.sleep(1.0)

        print("=== CAMERA FOV CHECK ===")
        print("Backend: Pi CSI / Picamera2")
        print(f"Requested main image: {args.capture_width} x {args.capture_height} @ {args.fps:g} FPS")

        if args.raw_width is not None:
            print(f"Requested raw/sensor mode: {args.raw_width} x {args.raw_height}")

        frame_rgb = picam2.capture_array()
        height, width = frame_rgb.shape[:2]
        print(f"Captured image: {width} x {height}")

        if args.view == "save":
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            output_dir = Path("camera_fov_check")
            output_dir.mkdir(parents=True, exist_ok=True)

            native_path = output_dir / "camera_fov_native.jpg"
            processed_path = output_dir / "camera_fov_processed.jpg"

            processed = cv2.resize(
                frame_bgr,
                (args.processing_width, args.processing_height),
                interpolation=cv2.INTER_AREA,
            )

            if not cv2.imwrite(str(native_path), frame_bgr):
                print(f"[ERROR] Could not save image: {native_path}")
                return 1
            if not cv2.imwrite(str(processed_path), processed):
                print(f"[ERROR] Could not save image: {processed_path}")
                return 1

            native_ratio = args.capture_width / args.capture_height
            processed_ratio = args.processing_width / args.processing_height

            print(f"Saved native image:    {native_path}")
            print(f"Saved processed image: {processed_path}")
            print(f"Native aspect:    {native_ratio:.4f}")
            print(f"Processed aspect: {processed_ratio:.4f}")

            if abs(native_ratio - processed_ratio) < 1e-9:
                print("Aspect ratios match; this resize does not introduce cropping.")
            else:
                print("[WARN] Aspect ratios differ; geometry is being changed.")

            print("Compare the left, right, top, and bottom scene boundaries.")
            return 0

        print("Live FOV view: press q or Esc to quit.")
        while True:
            frame_rgb = picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            cv2.imshow("Camera FOV Check", frame_bgr)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                return 0

    finally:
        picam2.stop()
        cv2.destroyAllWindows()


def main() -> int:
    args = build_parser().parse_args()

    if any(v <= 0 for v in (
        args.capture_width,
        args.capture_height,
        args.processing_width,
        args.processing_height,
    )):
        print("[ERROR] Capture and processing dimensions must be positive.")
        return 2

    if args.fps <= 0:
        print("[ERROR] --fps must be positive.")
        return 2

    if args.backend == "usb":
        return run_usb(args)

    return run_pi(args)


if __name__ == "__main__":
    sys.exit(main())