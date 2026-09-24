# hw_io/cameras/vision_worker.py

from __future__ import annotations

import time
import traceback
from multiprocessing import Event, Queue
from typing import Any

from calibration import CALIBRATION
from config import CONFIG
from perception.vision.detection_pipeline import build_vision_message
from hw_io.cameras.resolve import resolve_camera

from queue import Empty, Full

def _publish_latest(output_queue: Queue, message: dict) -> None:
    """
    Publish latest vision state without ever blocking the vision worker.

    If the one-slot queue already contains an old message, discard it.
    If a multiprocessing queue race prevents replacement immediately,
    drop this publication rather than stall camera processing.
    """
    try:
        output_queue.put_nowait(message)
        return
    except Full:
        pass

    try:
        output_queue.get_nowait()
    except Empty:
        pass

    try:
        output_queue.put_nowait(message)
    except Full:
        pass

def run_vision_worker(
    *,
    camera_name: str,
    robot: Any,
    output_queue: Queue,
    stop_event: Event,
) -> None:
    """
    Run one configured camera continuously.

    This function is intended to run inside its own OS process.
    It owns the camera backend for camera_name.
    """
    camera = None
    _publish_latest(output_queue, {
        "camera": camera_name,
        "timestamp": time.time(),
        "detections": [],
        "markers": [],
        "status": "worker_started",
    })

    try:
        cam_cal = CALIBRATION.cameras[camera_name]


        # Important: resolve/open camera inside this worker process.
        camera_config = CONFIG.cameras[camera_name]

        camera_profile = camera_config["profile"]
        camera_device = camera_config["device"]

        _publish_latest(output_queue, {
            "camera": camera_name,
            "timestamp": time.time(),
            "detections": [],
            "markers": [],
            "status": "worker_started",
        })

        print(f"[VISION_WORKER] starting camera={camera_name}", flush=True)
        print(
            f"[VISION_WORKER] resolving "
            f"profile={camera_profile} device={camera_device}",
            flush=True,
        )

        camera = resolve_camera(
            camera_name=camera_profile,
            device=camera_device,
            robot=robot,
        )
        last_status = None
        last_marker_count = None
        last_marker_ids = None

        while not stop_event.is_set():
            timestamp = time.time()

            try:
                markers = camera.see()
                marker_count = len(markers)
                marker_ids = [int(marker.id) for marker in markers]

                if (
                        last_status != "ok"
                        or last_marker_count != marker_count
                        or last_marker_ids != marker_ids
                ):
                    print(
                        f"[VISION_WORKER] "
                        f"camera={camera_name} "
                        f"status=ok "
                        f"markers={marker_count} "
                        f"ids={marker_ids}",
                        flush=True,
                    )
                    last_status = "ok"
                    last_marker_count = marker_count
                    last_marker_ids = marker_ids

                vision_message = build_vision_message(
                    camera_name=camera_name,
                    timestamp=timestamp,
                    markers=markers,
                    cam_cal=cam_cal,
                    status="ok",
                )

                vision_message["markers"] = list(markers)


            except Exception as e:
                if last_status != "error":
                    print(
                        f"[VISION_WORKER] camera={camera_name} status=error error={e!r}",
                        flush=True,
                    )
                    last_status = "error"
                vision_message = {
                    "camera": camera_name,
                    "timestamp": time.time(),
                    "detections": [],
                    "markers": [],
                    "status": "error",
                    "error": repr(e),
                    "traceback": traceback.format_exc(),
                }

            # Latest-only behaviour: remove old queued messages.
            _publish_latest(output_queue, vision_message)

    finally:
        if camera is not None and hasattr(camera, "close"):
            try:
                camera.close()
            except Exception:
                pass