# checkout/cameras/pi_libcamera_april.py

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from dataclasses import asdict
from typing import Callable

import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import controls
from perception.vision.apriltag_processor import (
    AprilTagProcessor,
    Marker,
)


# --------------------------------------------------
# Notes
# --------------------------------------------------
#
# Pi Camera / libcamera backend.
#
# Responsibilities:
# - open and configure the Pi camera
# - apply Pi-specific camera controls
# - capture RGB frames
# - pass frames to the shared AprilTagProcessor
#
# AprilTag detection, tag-size handling, marker geometry,
# and per-tag pose processing are implemented in:
#
#     perception.vision.apriltag_processor
#
# This file also contains a standalone camera/test utility.
# --------------------------------------------------


# --------------------------------------------------
# Camera backend
# --------------------------------------------------

class PiLibcameraAprilCamera:
    def __init__(
            self,
            *,
            device: int | None = None,
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
        self.device = device
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




        if self.device is None:
            raise ValueError("PiLibcameraAprilCamera requires a camera device")

        self._picam2 = Picamera2(camera_num=self.device)

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

        self._processor = AprilTagProcessor(
            families=families,
            tag_size_m=tag_size_m,
            tag_size_for_id=tag_size_for_id,
            camera_params=camera_params,
            quad_decimate=quad_decimate,
            nthreads=nthreads,
            quad_sigma=quad_sigma,
            refine_edges=refine_edges,
            decode_sharpening=decode_sharpening,
            apriltag_debug=apriltag_debug,
            min_decision_margin=min_decision_margin,
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
        return self._processor.process(frame_rgb)

    def see_with_frame(self) -> tuple[np.ndarray, list[Marker]]:
        frame_rgb = self.capture()
        markers = self._processor.process(frame_rgb)
        return frame_rgb, markers



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
    parser.add_argument("--device", type=int, default=0)
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
        device=args.device,
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