# checkout/cameras/opencv_usb.py

from __future__ import annotations

from typing import Callable

import cv2
import numpy as np

from perception.vision.apriltag_processor import (
    AprilTagProcessor,
    Marker,
)


class OpenCVUSBCamera:
    """
    USB camera backend using OpenCV / V4L2.

    Responsibilities:
    - open the USB camera
    - configure capture resolution / format / frame rate
    - capture frames
    - resize frames to the requested processing resolution
    - convert OpenCV BGR frames to RGB
    - pass RGB frames to the shared AprilTagProcessor
    """

    def __init__(
        self,
        *,
        device: str | int,
        capture_width: int = 1280,
        capture_height: int = 800,
        width: int = 640,
        height: int = 400,
        fps: int = 30,
        pixel_format: str = "MJPG",
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
    ) -> None:

        self.device = device

        self.capture_width = capture_width
        self.capture_height = capture_height

        self.width = width
        self.height = height

        self.fps = fps
        self.pixel_format = pixel_format

        if self.device is None:
            raise ValueError(
                "OpenCVUSBCamera requires a camera device"
            )

        # --------------------------------------------------
        # Open USB camera
        # --------------------------------------------------

        self._capture = cv2.VideoCapture(
            self.device,
            cv2.CAP_V4L2,
        )

        if not self._capture.isOpened():
            raise RuntimeError(
                f"Could not open USB camera device {self.device!r}"
            )

        # --------------------------------------------------
        # Configure USB capture
        # --------------------------------------------------

        if self.pixel_format:
            fourcc = cv2.VideoWriter_fourcc(
                *self.pixel_format
            )

            self._capture.set(
                cv2.CAP_PROP_FOURCC,
                fourcc,
            )

        self._capture.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            self.capture_width,
        )

        self._capture.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            self.capture_height,
        )

        self._capture.set(
            cv2.CAP_PROP_FPS,
            self.fps,
        )

        # --------------------------------------------------
        # Report actual configuration
        # --------------------------------------------------

        actual_width = int(
            self._capture.get(
                cv2.CAP_PROP_FRAME_WIDTH
            )
        )

        actual_height = int(
            self._capture.get(
                cv2.CAP_PROP_FRAME_HEIGHT
            )
        )

        actual_fps = self._capture.get(
            cv2.CAP_PROP_FPS
        )

        actual_fourcc_int = int(
            self._capture.get(
                cv2.CAP_PROP_FOURCC
            )
        )

        actual_fourcc = "".join(
            chr(
                (actual_fourcc_int >> 8 * i)
                & 0xFF
            )
            for i in range(4)
        )

        print(
            f"[OpenCVUSBCamera] "
            f"device={self.device!r}"
        )

        print(
            f"[OpenCVUSBCamera] "
            f"requested capture="
            f"{self.capture_width}x{self.capture_height} "
            f"{self.pixel_format} "
            f"{self.fps}fps"
        )

        print(
            f"[OpenCVUSBCamera] "
            f"actual capture="
            f"{actual_width}x{actual_height} "
            f"{actual_fourcc} "
            f"{actual_fps:.1f}fps"
        )

        print(
            f"[OpenCVUSBCamera] "
            f"processing resolution="
            f"{self.width}x{self.height}"
        )

        # --------------------------------------------------
        # Shared AprilTag processing
        # --------------------------------------------------

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

    def __enter__(self) -> "OpenCVUSBCamera":
        return self

    def __exit__(
        self,
        exc_type,
        exc,
        tb,
    ) -> None:
        self.close()

    def close(self) -> None:
        try:
            self._capture.release()
        except Exception:
            pass

    def capture(self) -> np.ndarray:
        """
        Capture one frame and return it as RGB at the
        configured processing resolution.
        """

        ok, frame_bgr = self._capture.read()

        if not ok or frame_bgr is None:
            raise RuntimeError(
                f"Failed to capture frame from "
                f"{self.device!r}"
            )

        if (
            frame_bgr.shape[1] != self.width
            or frame_bgr.shape[0] != self.height
        ):
            frame_bgr = cv2.resize(
                frame_bgr,
                (self.width, self.height),
                interpolation=cv2.INTER_AREA,
            )

        frame_rgb = cv2.cvtColor(
            frame_bgr,
            cv2.COLOR_BGR2RGB,
        )

        return frame_rgb

    def see(self) -> list[Marker]:
        frame_rgb = self.capture()
        return self._processor.process(frame_rgb)

    def see_with_frame(
        self,
    ) -> tuple[np.ndarray, list[Marker]]:

        frame_rgb = self.capture()

        markers = self._processor.process(
            frame_rgb
        )

        return frame_rgb, markers