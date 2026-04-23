# 3rdparty/cameras/Pi3/apriltag_pi3_test.py
#!/usr/bin/env python3
"""
Standalone AprilTag detector for RaspberryPi 4B + Camera Module 3.

What it does:
- Opens the Pi camera using Picamera2 (libcamera backend)
- Detects AprilTags using pupil_apriltags
- Estimates pose if camera intrinsics are provided
- Prints a compact per-tag structure suitable for later mapping into camera.see()

Preview modes:
- drm  : live raw camera preview on the Pi monitor (best for SSH + local monitor)
- none : no preview at all
- save : save annotated debug frames to disk periodically

Tested design target:
- RaspberryPi OS Bookworm/Bullseye
- Picamera2
- pupil_apriltags
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from picamera2 import Picamera2, Preview
from pupil_apriltags import Detector


@dataclass
class TagObservation:
    tag_id: int
    family: str
    decision_margin: float
    center_px: tuple[float, float]
    corners_px: list[tuple[float, float]]
    x_m: float | None
    y_m: float | None
    z_m: float | None
    distance_m: float | None
    yaw_rad: float | None
    pose_err: float | None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Standalone Pi AprilTag tester")
    # parser.add_argument("--width", type=int, default=640, help="Capture width")
    # parser.add_argument("--height", type=int, default=480, help="Capture height")
    parser.add_argument("--width", type=int, default=2304, help="Capture width")
    parser.add_argument("--height", type=int, default=1296, help="Capture height")
    parser.add_argument("--fps", type=int, default=30, help="Requested frame rate")
    parser.add_argument("--families", type=str, default="tag36h11", help="AprilTag families")
    parser.add_argument("--tag-size-m", type=float, default=0.05, help="Physical tag size in metres")
    parser.add_argument("--fx", type=float, default=None, help="Camera focal length fx in pixels")
    parser.add_argument("--fy", type=float, default=None, help="Camera focal length fy in pixels")
    parser.add_argument("--cx", type=float, default=None, help="Camera principal point cx in pixels")
    parser.add_argument("--cy", type=float, default=None, help="Camera principal point cy in pixels")
    parser.add_argument("--quad-decimate", type=float, default=1.5, help="Detector speed/accuracy tradeoff")
    parser.add_argument("--nthreads", type=int, default=2, help="Detector threads")
    parser.add_argument(
        "--preview",
        choices=["drm", "none", "save"],
        default="none",
        help="Preview mode: drm = Pi monitor, none = no preview, save = save annotated frames",
    )
    parser.add_argument(
        "--save-dir",
        type=str,
        default="apriltag_debug",
        help="Directory for saved debug frames when --preview save",
    )
    parser.add_argument(
        "--save-every",
        type=float,
        default=1.0,
        help="Seconds between saved debug frames when --preview save",
    )
    parser.add_argument("--jsonl", action="store_true", help="Print one JSON object per detection line")
    return parser


def have_pose_args(args: argparse.Namespace) -> bool:
    return all(v is not None for v in (args.fx, args.fy, args.cx, args.cy))


def estimate_yaw_from_rotation(pose_R: np.ndarray | None) -> float | None:
    if pose_R is None:
        return None
    return math.atan2(float(pose_R[0, 2]), float(pose_R[2, 2]))


def make_detector(args: argparse.Namespace) -> Detector:
    return Detector(
        families=args.families,
        nthreads=args.nthreads,
        quad_decimate=args.quad_decimate,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0,
    )


def make_camera(args: argparse.Namespace) -> Picamera2:
    picam2 = Picamera2()
    frame_duration_us = int(round(1_000_000 / args.fps))
    config = picam2.create_preview_configuration(
        main={"size": (args.width, args.height), "format": "RGB888"},
        controls={"FrameDurationLimits": (frame_duration_us, frame_duration_us)},
    )
    picam2.configure(config)

    if args.preview == "drm":
        picam2.start_preview(Preview.DRM)

    picam2.start()
    time.sleep(1.0)
    return picam2


def to_observation(det: Any, pose_enabled: bool) -> TagObservation:
    x_m = y_m = z_m = distance_m = yaw_rad = pose_err = None

    if pose_enabled and getattr(det, "pose_t", None) is not None:
        tx, ty, tz = np.array(det.pose_t).reshape(-1).tolist()[:3]
        x_m = float(tx)
        y_m = float(ty)
        z_m = float(tz)
        distance_m = float(math.sqrt(tx * tx + ty * ty + tz * tz))
        pose_err = float(det.pose_err) if getattr(det, "pose_err", None) is not None else None
        yaw_rad = estimate_yaw_from_rotation(getattr(det, "pose_R", None))

    family = det.tag_family
    if isinstance(family, bytes):
        family = family.decode("utf-8", errors="replace")

    return TagObservation(
        tag_id=int(det.tag_id),
        family=str(family),
        decision_margin=float(det.decision_margin),
        center_px=(float(det.center[0]), float(det.center[1])),
        corners_px=[(float(x), float(y)) for x, y in det.corners],
        x_m=x_m,
        y_m=y_m,
        z_m=z_m,
        distance_m=distance_m,
        yaw_rad=yaw_rad,
        pose_err=pose_err,
    )


def draw_detection(frame_rgb: np.ndarray, obs: TagObservation) -> np.ndarray:
    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

    corners = np.array(obs.corners_px, dtype=np.int32)
    cv2.polylines(frame_bgr, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

    cx, cy = int(obs.center_px[0]), int(obs.center_px[1])
    cv2.circle(frame_bgr, (cx, cy), 4, (0, 0, 255), -1)

    label = f"ID {obs.tag_id}"
    if obs.z_m is not None:
        label += f" z={obs.z_m:.2f}m"

    cv2.putText(
        frame_bgr,
        label,
        (cx + 8, cy - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 0, 0),
        2,
        cv2.LINE_AA,
    )

    return frame_bgr


def main() -> int:
    args = build_parser().parse_args()

    pose_enabled = have_pose_args(args)
    camera_params = [args.fx, args.fy, args.cx, args.cy] if pose_enabled else None

    detector = make_detector(args)
    picam2 = make_camera(args)

    save_dir = Path(args.save_dir)
    if args.preview == "save":
        save_dir.mkdir(parents=True, exist_ok=True)

    running = True
    last_save_time = 0.0

    def _handle_signal(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        while running:
            frame_rgb = picam2.capture_array()
            gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)

            detections = detector.detect(
                gray,
                estimate_tag_pose=pose_enabled,
                camera_params=camera_params,
                tag_size=args.tag_size_m if pose_enabled else None,
            )

            observations = [to_observation(det, pose_enabled) for det in detections]

            if args.jsonl:
                for obs in observations:
                    print(json.dumps(asdict(obs), separators=(",", ":")))
            else:
                print(f"\nDetected {len(observations)} tag(s)")
                for obs in observations:
                    print(asdict(obs))

            if args.preview == "save":
                annotated = frame_rgb.copy()
                for obs in observations:
                    annotated = draw_detection(annotated, obs)

                now = time.time()
                if now - last_save_time >= args.save_every:
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    out_path = save_dir / f"apriltag_{timestamp}.png"
                    cv2.imwrite(str(out_path), annotated)
                    print(f"[SAVE] Wrote {out_path}")
                    last_save_time = now

    finally:
        picam2.stop()

    return 0


if __name__ == "__main__":
    sys.exit(main())