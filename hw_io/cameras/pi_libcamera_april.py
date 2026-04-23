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
from libcamera import controls
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
#    - detector-provided pose works if camera_params is provided
#
# 2. Mixed-size mode:
#    - pass tag_size_for_id(tag_id)
#    - tag size is resolved per detection
#    - pose is estimated per detection with OpenCV solvePnP
#
# SAFER HYBRID GEOMETRY:
# - horizontal_angle and vertical_angle are computed directly from
#   the image center + camera intrinsics
# - distance/orientation come from solvePnP
#
# This avoids 180-degree nonsense in bearing/vertical angle if PnP
# corner ordering is imperfect.
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
            quad_sigma: float = 0.0,
            refine_edges: int = 1,
            decode_sharpening: float = 0.25,
            apriltag_debug: int = 0,
            min_decision_margin: float = 20.0,
            sensor_width: int | None = None,
            sensor_height: int | None = None,
            sensor_output_size: tuple[int, int] | None = None,
            sensor_bit_depth: int | None = None,
            force_full_sensor_scaler_crop: bool = False,
            af_mode: str | None = None,
            lens_position: float | None = None,
            ae_enable: bool | None = None,
            exposure_time_us: int | None = None,
            analogue_gain: float | None = None,
            awb_enable: bool | None = None,
            colour_gains: tuple[float, float] | None = None,
    ) -> None:
        self.width = width
        self.height = height
        self.fps = fps
        self.sensor_width = sensor_width
        self.sensor_height = sensor_height
        self.sensor_output_size = sensor_output_size
        self.sensor_bit_depth = sensor_bit_depth
        self.force_full_sensor_scaler_crop = force_full_sensor_scaler_crop
        self.af_mode = af_mode
        self.lens_position = lens_position
        self.ae_enable = ae_enable
        self.exposure_time_us = exposure_time_us
        self.analogue_gain = analogue_gain
        self.awb_enable = awb_enable
        self.colour_gains = colour_gains
        self.families = families
        self.tag_size_m = tag_size_m
        self.tag_size_for_id = tag_size_for_id
        self.camera_params = camera_params
        self.pose_enabled = camera_params is not None
        self.quad_decimate = quad_decimate
        self.nthreads = nthreads
        self.quad_sigma = quad_sigma
        self.refine_edges = refine_edges
        self.decode_sharpening = decode_sharpening
        self.apriltag_debug = apriltag_debug
        self.min_decision_margin = min_decision_margin

        self._mixed_size_mode = tag_size_for_id is not None
        self._single_size_mode = tag_size_m is not None

        if self._mixed_size_mode and self._single_size_mode:
            raise ValueError("Specify either tag_size_m or tag_size_for_id, not both")

        self._picam2 = Picamera2()
        try:
            print(f"[PiCam] sensor_modes = {self._picam2.sensor_modes}")
        except Exception as e:
            print(f"[PiCam] Could not read sensor modes: {e}")

        config_kwargs = {
            "main": {"size": (width, height), "format": "RGB888"},
            "controls": {"FrameRate": fps},
        }

        if self.sensor_output_size is not None and self.sensor_bit_depth is not None:
            config_kwargs["sensor"] = {
                "output_size": self.sensor_output_size,
                "bit_depth": self.sensor_bit_depth,
            }

        config = self._picam2.create_preview_configuration(**config_kwargs)
        self._picam2.configure(config)
        try:
            print(f"[PiCam] Requested config kwargs = {config_kwargs}")
        except Exception:
            pass
        self._picam2.start()

        control_updates = {}

        if self.af_mode is not None:
            af_mode_map = {
                "manual": controls.AfModeEnum.Manual,
                "auto": controls.AfModeEnum.Auto,
                "continuous": controls.AfModeEnum.Continuous,
            }
            mapped = af_mode_map.get(str(self.af_mode).lower())
            if mapped is not None:
                control_updates["AfMode"] = mapped

        if self.lens_position is not None:
            control_updates["LensPosition"] = float(self.lens_position)

        if self.ae_enable is not None:
            control_updates["AeEnable"] = bool(self.ae_enable)

        if self.exposure_time_us is not None:
            control_updates["ExposureTime"] = int(self.exposure_time_us)

        if self.analogue_gain is not None:
            control_updates["AnalogueGain"] = float(self.analogue_gain)

        if self.awb_enable is not None:
            control_updates["AwbEnable"] = bool(self.awb_enable)

        if self.colour_gains is not None:
            control_updates["ColourGains"] = tuple(float(x) for x in self.colour_gains)

        if control_updates:
            try:
                self._picam2.set_controls(control_updates)
                print(f"[PiCam] Applied runtime controls = {control_updates}")
            except Exception as e:
                print(f"[PiCam] Could not apply runtime controls: {e}")

        if (
                self.force_full_sensor_scaler_crop
                and self.sensor_width is not None
                and self.sensor_height is not None
        ):
            try:
                self._picam2.set_controls(
                    {"ScalerCrop": (0, 0, self.sensor_width, self.sensor_height)}
                )
                print(
                    f"[PiLibcameraAprilCamera] Requested ScalerCrop="
                    f"(0, 0, {self.sensor_width}, {self.sensor_height})"
                )
            except Exception as e:
                print(f"[PiLibcameraAprilCamera] Could not set ScalerCrop: {e}")

        time.sleep(1.0)

        try:
            md = self._picam2.capture_metadata()
            print(f"[PiCam] Active ScalerCrop = {md.get('ScalerCrop')}")
        except Exception as e:
            print(f"[PiCam] Could not read metadata: {e}")

        try:
            applied = self._picam2.camera_configuration()
            print(f"[PiCam] Applied sensor config = {applied.get('sensor')}")
            print(f"[PiCam] Applied raw config = {applied.get('raw')}")
            print(f"[PiCam] Applied main config = {applied.get('main')}")
        except Exception as e:
            print(f"[PiCam] Could not read applied configuration: {e}")

        self._detector = Detector(
            families=families,
            nthreads=self.nthreads,
            quad_decimate=self.quad_decimate,
            quad_sigma=self.quad_sigma,
            refine_edges=self.refine_edges,
            decode_sharpening=self.decode_sharpening,
            debug=self.apriltag_debug,
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

        if self._single_size_mode:
            detections = self._detector.detect(
                gray,
                estimate_tag_pose=self.pose_enabled,
                camera_params=list(self.camera_params) if self.camera_params else None,
                tag_size=self.tag_size_m if self.pose_enabled else None,
            )
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

    def _angles_from_center(
        self,
        center_px: tuple[float, float],
    ) -> tuple[float | None, float | None]:
        """
        Compute horizontal and vertical angles directly from image center
        and camera intrinsics.

        This is safer than deriving these from PnP because a corner-order
        mistake in pose solve can flip pose by ~180 degrees while the pixel
        center still gives sensible line-of-sight angles.
        """
        if self.camera_params is None:
            return None, None

        fx, fy, cx, cy = self.camera_params
        px, py = center_px

        horizontal_angle = math.atan2(px - cx, fx)
        vertical_angle = math.atan2(py - cy, fy)
        return horizontal_angle, vertical_angle

    def _camera_matrix(self) -> np.ndarray | None:
        if self.camera_params is None:
            return None

        fx, fy, cx, cy = self.camera_params
        return np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    def _object_point_variants(self, tag_size_m: float) -> list[np.ndarray]:
        """
        Build several possible object-point orderings for the square tag.

        This is to tolerate uncertainty about the exact ordering returned
        by det.corners from the detector.
        """
        half = tag_size_m / 2.0

        base = np.array(
            [
                [-half, -half, 0.0],
                [ half, -half, 0.0],
                [ half,  half, 0.0],
                [-half,  half, 0.0],
            ],
            dtype=np.float64,
        )

        variants = []
        for shift in range(4):
            variants.append(np.roll(base, -shift, axis=0))

        reversed_base = base[::-1].copy()
        for shift in range(4):
            variants.append(np.roll(reversed_base, -shift, axis=0))

        return variants

    def _solve_pose_best(
        self,
        corners_px: list[tuple[float, float]],
        tag_size_m: float,
    ) -> tuple[
        float | None, float | None, float | None,
        float | None, float | None, float | None, float | None
    ]:
        """
        Estimate pose for one tag using OpenCV solvePnP.

        Returns:
            x_m, y_m, z_m, yaw, pitch, roll, reproj_err
        """
        camera_matrix = self._camera_matrix()
        if camera_matrix is None:
            return (None, None, None, None, None, None, None)

        dist_coeffs = np.zeros((4, 1), dtype=np.float64)
        image_points = np.array(corners_px, dtype=np.float64)

        best = None
        best_err = None

        for object_points in self._object_point_variants(tag_size_m):
            try:
                ok, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    camera_matrix,
                    dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
            except cv2.error:
                continue

            if not ok:
                continue

            pose_R, _ = cv2.Rodrigues(rvec)
            yaw, pitch, roll = self._rotation_matrix_to_ypr(pose_R)

            projected, _ = cv2.projectPoints(
                object_points,
                rvec,
                tvec,
                camera_matrix,
                dist_coeffs,
            )
            projected = projected.reshape(-1, 2)
            err = float(np.mean(np.linalg.norm(projected - image_points, axis=1)))

            x_m = float(tvec[0][0])
            y_m = float(tvec[1][0])
            z_m = float(tvec[2][0])

            candidate = (x_m, y_m, z_m, yaw, pitch, roll, err)

            # Prefer physically plausible forward-facing solutions
            # (positive z), then lowest reprojection error.
            if best is None:
                best = candidate
                best_err = err
                continue

            best_z = best[2]
            if best_z is not None and z_m > 0 and best_z <= 0:
                best = candidate
                best_err = err
                continue

            if (z_m > 0) == (best_z > 0) and err < best_err:
                best = candidate
                best_err = err

        if best is None:
            return (None, None, None, None, None, None, None)

        return best

    def _detection_to_marker(self, det: Any) -> Marker:
        family = det.tag_family
        if isinstance(family, bytes):
            family = family.decode("utf-8", errors="replace")

        tag_id = int(det.tag_id)
        center_px = (float(det.center[0]), float(det.center[1]))
        corners_px = [(float(x), float(y)) for x, y in det.corners]
        resolved_size_m = self._resolve_size_for_detection(tag_id)

        x_m = y_m = z_m = pose_err = None
        distance_mm: float | None = None
        yaw = pitch = roll = None

        # Bearing / vertical angle come from pixel geometry whenever possible.
        horizontal_angle, vertical_angle = self._angles_from_center(center_px)

        # --------------------------------------------------
        # Case 1: single-size mode with detector-provided pose
        # --------------------------------------------------
        if getattr(det, "pose_t", None) is not None:
            tx, ty, tz = np.array(det.pose_t).reshape(-1).tolist()[:3]
            x_m = float(tx)
            y_m = float(ty)
            z_m = float(tz)

            distance_mm = 1000.0 * math.sqrt(x_m * x_m + y_m * y_m + z_m * z_m)

            yaw, pitch, roll = self._rotation_matrix_to_ypr(getattr(det, "pose_R", None))

            if getattr(det, "pose_err", None) is not None:
                pose_err = float(det.pose_err)

        # --------------------------------------------------
        # Case 2: mixed-size mode with per-detection solvePnP
        # --------------------------------------------------
        elif resolved_size_m is not None and self.camera_params is not None:
            x_m, y_m, z_m, yaw, pitch, roll, pose_err = self._solve_pose_best(
                corners_px,
                resolved_size_m,
            )

            if x_m is not None and y_m is not None and z_m is not None:
                distance_mm = 1000.0 * math.sqrt(x_m * x_m + y_m * y_m + z_m * z_m)

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
            center_px=center_px,
            corners_px=corners_px,
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