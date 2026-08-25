# tools/setup/cameras/camera_apriltag_validate.py

#!/usr/bin/env python3
"""Universal AprilTag validation for Pi CSI or USB cameras."""

from __future__ import annotations

import argparse
import importlib
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[3]

if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import cv2
import numpy as np
from pupil_apriltags import Detector


@dataclass(frozen=True)
class Calibration:
    camera_params: tuple[float, float, float, float]
    distortion: tuple[float, float, float, float, float]


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Validate AprilTag performance for Pi CSI or USB cameras")
    p.add_argument("--backend", required=True, choices=["usb", "pi"])
    p.add_argument("--device", help="USB V4L2 device path; required for USB")
    p.add_argument("--capture-width", type=int, required=True)
    p.add_argument("--capture-height", type=int, required=True)
    p.add_argument("--processing-width", type=int, required=True)
    p.add_argument("--processing-height", type=int, required=True)
    p.add_argument("--fps", type=float, default=30.0)
    p.add_argument("--format", default="MJPG", help="USB FOURCC; ignored for Pi")
    p.add_argument("--raw-width", type=int)
    p.add_argument("--raw-height", type=int)
    p.add_argument("--calibration-module", help="e.g. calibration.cameras.arducam_fullfov_640_400")
    p.add_argument("--families", default="tag36h11")
    p.add_argument("--tag-size-m", type=float, help="Required for pose/distance")
    p.add_argument("--target-id", type=int)
    p.add_argument("--min-decision-margin", type=float, default=20.0)
    p.add_argument("--quad-decimate", type=float, default=1.5)
    p.add_argument("--nthreads", type=int, default=2)
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument("--save", action="store_true")
    p.add_argument("--save-path", default="apriltag_validation.jpg")
    return p


def load_calibration(name: str | None) -> Calibration | None:
    if not name:
        return None
    m = importlib.import_module(name)
    params = getattr(m, "PNP_CAMERA_PARAMS", getattr(m, "CAMERA_PARAMS", None))
    if params is None:
        raise RuntimeError(f"{name} has no CAMERA_PARAMS/PNP_CAMERA_PARAMS")
    dist = getattr(
        m,
        "PNP_DISTORTION_COEFFICIENTS",
        getattr(m, "DISTORTION_COEFFICIENTS", (0, 0, 0, 0, 0)),
    )
    dist = tuple(float(v) for v in dist)
    if len(dist) < 5:
        dist = dist + (0.0,) * (5 - len(dist))
    return Calibration(tuple(float(v) for v in params), tuple(dist[:5]))


def camera_matrix(cal: Calibration) -> np.ndarray:
    fx, fy, cx, cy = cal.camera_params
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)


def object_point_variants(size_m: float) -> list[np.ndarray]:
    h = size_m / 2.0
    base = np.array([[-h, -h, 0.0], [h, -h, 0.0], [h, h, 0.0], [-h, h, 0.0]], dtype=np.float64)
    out = [np.roll(base, -s, axis=0) for s in range(4)]
    rev = base[::-1].copy()
    out += [np.roll(rev, -s, axis=0) for s in range(4)]
    return out


def solve_pose(corners: np.ndarray, size_m: float, cal: Calibration) -> tuple[float, float] | None:
    K = camera_matrix(cal)
    D = np.asarray(cal.distortion, dtype=np.float64)
    image_points = np.asarray(corners, dtype=np.float64)
    best = None
    for object_points in object_point_variants(size_m):
        try:
            ok, rvec, tvec = cv2.solvePnP(object_points, image_points, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except cv2.error:
            continue
        if not ok:
            continue
        x, y, z = (float(tvec[i][0]) for i in range(3))
        if z <= 0:
            continue
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, K, D)
        err = float(np.mean(np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1)))
        distance = math.sqrt(x*x + y*y + z*z)
        if best is None or err < best[1]:
            best = (distance, err)
    return best


def bearing_deg(center: tuple[float, float], cal: Calibration) -> float:
    fx, _, cx, _ = cal.camera_params
    return math.degrees(math.atan2(center[0] - cx, fx))


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


class USBCapture:
    def __init__(self, a: argparse.Namespace):
        if not a.device:
            raise RuntimeError("--device is required for USB")
        if len(a.format) != 4:
            raise RuntimeError("--format must be a four-character FOURCC")
        self.cap = cv2.VideoCapture(a.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open USB camera: {a.device}")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*a.format))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.capture_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.capture_height)
        self.cap.set(cv2.CAP_PROP_FPS, a.fps)
        self.processing_size = (a.processing_width, a.processing_height)
        for _ in range(10):
            ok, _ = self.cap.read()
            if not ok:
                raise RuntimeError("USB camera failed during settling")

    def read_rgb(self) -> np.ndarray:
        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError("USB frame capture failed")
        frame = cv2.resize(frame, self.processing_size, interpolation=cv2.INTER_AREA)
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def close(self) -> None:
        self.cap.release()


class PiCapture:
    def __init__(self, a: argparse.Namespace):
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise RuntimeError("Picamera2 is unavailable") from exc
        if (a.raw_width is None) != (a.raw_height is None):
            raise RuntimeError("Supply both --raw-width and --raw-height, or neither")
        self.picam2 = Picamera2()
        fd = int(round(1_000_000 / a.fps))
        kwargs: dict[str, Any] = {
            "main": {"size": (a.capture_width, a.capture_height), "format": "RGB888"},
            "controls": {"FrameDurationLimits": (fd, fd)},
        }
        if a.raw_width is not None:
            kwargs["raw"] = {"size": (a.raw_width, a.raw_height)}
        cfg = self.picam2.create_preview_configuration(**kwargs)
        self.picam2.configure(cfg)
        self.picam2.start()
        time.sleep(1.0)
        self.processing_size = (a.processing_width, a.processing_height)

    def read_rgb(self) -> np.ndarray:
        frame = self.picam2.capture_array()
        if (frame.shape[1], frame.shape[0]) != self.processing_size:
            frame = cv2.resize(frame, self.processing_size, interpolation=cv2.INTER_AREA)
        return frame

    def close(self) -> None:
        self.picam2.stop()


def draw(frame_rgb: np.ndarray, det: Any, label: str) -> np.ndarray:
    bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    corners = np.asarray(det.corners, dtype=np.int32)
    cv2.polylines(bgr, [corners], True, (0, 255, 0), 2)
    c = (int(round(float(det.center[0]))), int(round(float(det.center[1]))))
    cv2.circle(bgr, c, 4, (0, 0, 255), -1)
    cv2.putText(bgr, label, (c[0] + 8, c[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)
    return bgr


def main() -> int:
    a = build_parser().parse_args()
    if any(v <= 0 for v in (a.capture_width, a.capture_height, a.processing_width, a.processing_height, a.fps, a.duration)):
        print("[ERROR] Dimensions, FPS, and duration must be positive")
        return 2

    try:
        cal = load_calibration(a.calibration_module)
        detector = make_detector(a)
        camera = USBCapture(a) if a.backend == "usb" else PiCapture(a)
    except Exception as exc:
        print(f"[ERROR] {exc}")
        return 1

    print("=== APRILTAG VALIDATION ===")
    print(f"Backend: {a.backend}")
    if a.device:
        print(f"Device: {a.device}")
    print(f"Capture: {a.capture_width} x {a.capture_height} @ {a.fps:g} FPS")
    print(f"Processing: {a.processing_width} x {a.processing_height}")
    print(f"quad_decimate: {a.quad_decimate}")
    print(f"Minimum decision margin: {a.min_decision_margin}")
    print(f"Calibration: {a.calibration_module or 'none'}")
    print(f"Tag size: {a.tag_size_m if a.tag_size_m is not None else 'not supplied'}")
    print(f"Duration: {a.duration:.1f} s\n")

    frames = 0
    any_tag_frames = 0
    target_frames = 0
    margins: list[float] = []
    target_margins: list[float] = []
    last_annotated = None
    start = time.monotonic()

    try:
        while time.monotonic() - start < a.duration:
            frame_rgb = camera.read_rgb()
            gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
            detections = detector.detect(gray, estimate_tag_pose=False)
            detections = [d for d in detections if float(d.decision_margin) >= a.min_decision_margin]
            frames += 1
            if detections:
                any_tag_frames += 1
            target_seen = False

            for d in detections:
                tag_id = int(d.tag_id)
                margin = float(d.decision_margin)
                margins.append(margin)
                center = (float(d.center[0]), float(d.center[1]))
                fields = [f"id={tag_id}", f"margin={margin:.1f}"]
                if cal is not None:
                    fields.append(f"bearing={bearing_deg(center, cal):.1f}deg")
                if cal is not None and a.tag_size_m is not None:
                    pose = solve_pose(np.asarray(d.corners, dtype=np.float64), a.tag_size_m, cal)
                    if pose is not None:
                        distance, reproj = pose
                        fields += [f"distance={distance:.3f}m", f"reproj={reproj:.2f}px"]
                print("  " + "  ".join(fields))
                if a.target_id is not None and tag_id == a.target_id:
                    target_seen = True
                    target_margins.append(margin)
                last_annotated = draw(frame_rgb, d, f"ID {tag_id} margin {margin:.1f}")

            if target_seen:
                target_frames += 1

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        camera.close()

    elapsed = time.monotonic() - start
    rate = frames / elapsed if elapsed > 0 else 0.0
    print("\n=== SUMMARY ===")
    print(f"Frames processed: {frames}")
    print(f"Processing rate: {rate:.2f} frames/s")
    if frames:
        print(f"Frames with accepted tag: {any_tag_frames}/{frames} ({100*any_tag_frames/frames:.1f}%)")
    if margins:
        print(f"Decision margin: min={min(margins):.1f} mean={sum(margins)/len(margins):.1f} max={max(margins):.1f}")
    if a.target_id is not None and frames:
        print(f"Target ID {a.target_id}: {target_frames}/{frames} ({100*target_frames/frames:.1f}%)")
        if target_margins:
            print(f"Target margin: min={min(target_margins):.1f} mean={sum(target_margins)/len(target_margins):.1f} max={max(target_margins):.1f}")
    if a.save and last_annotated is not None:
        path = Path(a.save_path)
        if cv2.imwrite(str(path), last_annotated):
            print(f"Saved annotated frame: {path}")
        else:
            print(f"[WARN] Could not save annotated frame: {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())