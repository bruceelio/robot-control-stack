# hw_io/cameras/pi_libcamera_april.py
from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from dataclasses import dataclass, asdict
from typing import Any, Callable

import cv2
import numpy as np
from picamera2 import Picamera2
from pupil_apriltags import Detector


# --------------------------------------------------
# Notes
# --------------------------------------------------
#
# This file serves two purposes:
#
# 1. Reusable camera backend class
#    - import PiLibcameraAprilCamera and call cam.see()
#
# 2. Standalone tester
#    - run this file directly to preview detections and debug
#
# In testing, tags may be lost at very close range (~15 cm),
# likely due to framing/focus/blur. Final pickup logic should
# not assume vision remains valid at the very last approach.
#
# IMPORTANT:
# If camera calibration / pose parameters are not supplied,
# positional values are returned as None rather than fake zeros.
#
# IMPORTANT:
# The pupil_apriltags Detector.detect(...) API estimates pose using
# a single tag_size argument for the whole detection call. That means
# mixed-size tag scenes cannot be posed correctly in one simple pass.
#
# Therefore this class supports two modes:
#
# 1. Single-size mode:
#    - pass tag_size_m
#    - pose works if camera_params is provided
#
# 2. Mixed-size mode:
#    - pass tag_size_for_id(tag_id)
#    - per-tag size is attached to each Marker
#    - pose fields remain None for safety
# --------------------------------------------------


# --------------------------------------------------
# SR-like data model
# --------------------------------------------------

@dataclass
class MarkerPosition:
    distance: float | None          # mm
    horizontal_angle: float | None  # radians
    vertical_angle: float | None    # radians


@dataclass
class MarkerOrientation:
    yaw: float | None = None
    pitch: float | None = None
    roll: float | None = None


@dataclass
class Marker:
    id: int
    position: MarkerPosition
    orientation: MarkerOrientation
    size: float | None = None
    decision_margin: float = 0.0
    family: str = "tag36h11"
    center_px: tuple[float, float] | None = None
    corners_px: list[tuple[float, float]] | None = None
    x_m: float | None = None
    y_m: float | None = None
    z_m: float | None = None
    pose_err: float | None = None


# --------------------------------------------------
# Camera backend
# --------------------------------------------------

class PiLibcameraAprilCamera:
    def __init__(
        self,
        *,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        families: str = "tag36h11",
        tag_size_m: float | None = None,
        tag_size_for_id: Callable[[int], float] | None = None,
        camera_params: tuple[float, float, float, float] | None = None,
        quad_decimate: float = 1.5,
        nthreads: int = 2,
        min_decision_margin: float = 20.0,
    ) -> None:
        self.width = width
        self.height = height
        self.fps = fps
        self.families = families
        self.tag_size_m = tag_size_m
        self.tag_size_for_id = tag_size_for_id
        self.camera_params = camera_params
        self.pose_enabled = camera_params is not None
        self.min_decision_margin = min_decision_margin

        self._mixed_size_mode = tag_size_for_id is not None
        self._single_size_mode = tag_size_m is not None

        if self._mixed_size_mode and self._single_size_mode:
            raise ValueError(
                "Specify either tag_size_m or tag_size_for_id, not both"
            )

        self._picam2 = Picamera2()
        config = self._picam2.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"},
            controls={"FrameRate": fps},
        )
        self._picam2.configure(config)
        self._picam2.start()
        time.sleep(1.0)

        self._detector = Detector(
            families=families,
            nthreads=nthreads,
            quad_decimate=quad_decimate,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

    def __enter__(self) -> "PiLibcameraAprilCamera":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def close(self) -> None:
        try:
            self._picam2.stop()
        except Exception:
            pass

    def capture(self) -> np.ndarray:
        return self._picam2.capture_array()

    def see(self) -> list[Marker]:
        frame_rgb = self.capture()
        return self._detect_markers(frame_rgb)

    def see_with_frame(self) -> tuple[np.ndarray, list[Marker]]:
        frame_rgb = self.capture()
        markers = self._detect_markers(frame_rgb)
        return frame_rgb, markers

    def _detect_markers(self, frame_rgb: np.ndarray) -> list[Marker]:
        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)

        # --------------------------------------------------
        # Single-size mode:
        # Safe to ask the detector for pose directly.
        # --------------------------------------------------
        if self._single_size_mode:
            detections = self._detector.detect(
                gray,
                estimate_tag_pose=self.pose_enabled,
                camera_params=list(self.camera_params) if self.camera_params else None,
                tag_size=self.tag_size_m if self.pose_enabled else None,
            )

        # --------------------------------------------------
        # Mixed-size mode:
        # Detect only. Attach correct per-tag size later.
        # Pose remains None to avoid incorrect geometry.
        # --------------------------------------------------
        else:
            detections = self._detector.detect(
                gray,
                estimate_tag_pose=False,
                camera_params=None,
                tag_size=None,
            )

        detections = [
            det for det in detections
            if float(det.decision_margin) >= self.min_decision_margin
        ]

        return [self._detection_to_marker(det) for det in detections]

    def _resolve_size_for_detection(self, tag_id: int) -> float | None:
        if self.tag_size_for_id is not None:
            return self.tag_size_for_id(tag_id)
        return self.tag_size_m

    def _detection_to_marker(self, det: Any) -> Marker:
        family = det.tag_family
        if isinstance(family, bytes):
            family = family.decode("utf-8", errors="replace")

        tag_id = int(det.tag_id)
        resolved_size_m = self._resolve_size_for_detection(tag_id)

        x_m = y_m = z_m = pose_err = None
        distance_mm: float | None = None
        horizontal_angle: float | None = None
        vertical_angle: float | None = None
        yaw = pitch = roll = None

        if getattr(det, "pose_t", None) is not None:
            tx, ty, tz = np.array(det.pose_t).reshape(-1).tolist()[:3]
            x_m = float(tx)
            y_m = float(ty)
            z_m = float(tz)

            distance_mm = 1000.0 * math.sqrt(x_m * x_m + y_m * y_m + z_m * z_m)
            horizontal_angle = math.atan2(x_m, z_m)
            vertical_angle = math.atan2(y_m, z_m)

            yaw, pitch, roll = self._rotation_matrix_to_ypr(getattr(det, "pose_R", None))

            if getattr(det, "pose_err", None) is not None:
                pose_err = float(det.pose_err)

        return Marker(
            id=tag_id,
            position=MarkerPosition(
                distance=distance_mm,
                horizontal_angle=horizontal_angle,
                vertical_angle=vertical_angle,
            ),
            orientation=MarkerOrientation(
                yaw=yaw,
                pitch=pitch,
                roll=roll,
            ),
            size=resolved_size_m,
            decision_margin=float(det.decision_margin),
            family=str(family),
            center_px=(float(det.center[0]), float(det.center[1])),
            corners_px=[(float(x), float(y)) for x, y in det.corners],
            x_m=x_m,
            y_m=y_m,
            z_m=z_m,
            pose_err=pose_err,
        )

    @staticmethod
    def _rotation_matrix_to_ypr(
        pose_R: np.ndarray | None,
    ) -> tuple[float | None, float | None, float | None]:
        if pose_R is None:
            return None, None, None

        yaw = math.atan2(float(pose_R[0, 2]), float(pose_R[2, 2]))
        pitch = math.atan2(
            -float(pose_R[1, 2]),
            math.sqrt(float(pose_R[1, 0]) ** 2 + float(pose_R[1, 1]) ** 2),
        )
        roll = math.atan2(float(pose_R[1, 0]), float(pose_R[1, 1]))
        return yaw, pitch, roll


# --------------------------------------------------
# Standalone test helpers
# --------------------------------------------------

def draw_marker(frame_rgb: np.ndarray, marker: Marker) -> np.ndarray:
    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

    if marker.corners_px:
        corners = np.array(marker.corners_px, dtype=np.int32)
        cv2.polylines(frame_bgr, [corners], True, (0, 255, 0), 2)

    if marker.center_px:
        cx, cy = int(marker.center_px[0]), int(marker.center_px[1])
        cv2.circle(frame_bgr, (cx, cy), 4, (0, 0, 255), -1)

        dist_text = (
            f"{marker.position.distance:.0f}"
            if marker.position.distance is not None
            else "None"
        )
        bearing_text = (
            f"{math.degrees(marker.position.horizontal_angle):.1f}deg"
            if marker.position.horizontal_angle is not None
            else "None"
        )
        va_text = (
            f"{math.degrees(marker.position.vertical_angle):.2f}deg"
            if marker.position.vertical_angle is not None
            else "None"
        )
        size_text = f"{marker.size:.3f}m" if marker.size is not None else "None"

        label = (
            f"id={marker.id} "
            f"size={size_text} "
            f"dist={dist_text} "
            f"bearing={bearing_text} "
            f"va={va_text}"
        )

        cv2.putText(
            frame_bgr,
            label,
            (cx + 8, cy - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 0, 0),
            2,
            cv2.LINE_AA,
        )

    return frame_bgr


def print_marker_summary(marker: Marker) -> None:
    dist_text = (
        f"{marker.position.distance:.0f}"
        if marker.position.distance is not None
        else "None"
    )
    bearing_text = (
        f"{math.degrees(marker.position.horizontal_angle):.1f}deg"
        if marker.position.horizontal_angle is not None
        else "None"
    )
    va_text = (
        f"{math.degrees(marker.position.vertical_angle):.2f}deg"
        if marker.position.vertical_angle is not None
        else "None"
    )
    size_text = f"{marker.size:.3f}m" if marker.size is not None else "None"

    print(
        f"id={marker.id} "
        f"size={size_text} "
        f"dist={dist_text} "
        f"bearing={bearing_text} "
        f"va={va_text}"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pi libcamera AprilTag standalone tester")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--families", type=str, default="tag36h11")
    parser.add_argument("--tag-size-m", type=float, default=0.05)
    parser.add_argument("--fx", type=float, default=None)
    parser.add_argument("--fy", type=float, default=None)
    parser.add_argument("--cx", type=float, default=None)
    parser.add_argument("--cy", type=float, default=None)
    parser.add_argument("--quad-decimate", type=float, default=1.5)
    parser.add_argument("--nthreads", type=int, default=2)
    parser.add_argument("--min-decision-margin", type=float, default=20.0)
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--debug-dump", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()

    camera_params = None
    if all(v is not None for v in (args.fx, args.fy, args.cx, args.cy)):
        camera_params = (args.fx, args.fy, args.cx, args.cy)

    with PiLibcameraAprilCamera(
        width=args.width,
        height=args.height,
        fps=args.fps,
        families=args.families,
        tag_size_m=args.tag_size_m,
        camera_params=camera_params,
        quad_decimate=args.quad_decimate,
        nthreads=args.nthreads,
        min_decision_margin=args.min_decision_margin,
    ) as cam:
        running = True

        def _handle_signal(signum, frame):
            nonlocal running
            running = False

        signal.signal(signal.SIGINT, _handle_signal)
        signal.signal(signal.SIGTERM, _handle_signal)

        while running:
            if args.preview:
                frame_rgb, markers = cam.see_with_frame()
            else:
                frame_rgb = None
                markers = cam.see()

            print(f"\nSeen total={len(markers)}")
            for marker in markers:
                print_marker_summary(marker)
                if args.debug_dump:
                    print(asdict(marker))

            if args.preview and frame_rgb is not None:
                preview = frame_rgb.copy()
                for marker in markers:
                    preview = draw_marker(preview, marker)

                cv2.imshow("AprilTags", preview)
                key = cv2.waitKey(1) & 0xFF

                if key == ord("q"):
                    print("[MAIN] 'q' pressed, exiting")
                    break

    if args.preview:
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())