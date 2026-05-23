# hw_io/cameras/vision_worker.py

from __future__ import annotations

import time
import traceback
from multiprocessing import Event, Queue
from typing import Any

from calibration import CALIBRATION
from config import CONFIG
from hw_io.cameras.detection_pipeline import build_vision_message
from hw_io.cameras.resolve import resolve_camera


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
    output_queue.put({
        "camera": camera_name,
        "timestamp": time.time(),
        "detections": [],
        "markers": [],
        "status": "worker_started",
    })

    try:
        cam_cal = CALIBRATION.cameras[camera_name]

        # Important: resolve/open camera inside this worker process.
        camera_profile = CONFIG.cameras[camera_name]
        output_queue.put({
            "camera": camera_name,
            "timestamp": time.time(),
            "detections": [],
            "markers": [],
            "status": "profile_resolved",
            "camera_profile": camera_profile,
        })
        print(f"[VISION_WORKER] starting camera={camera_name}", flush=True)
        print(f"[VISION_WORKER] resolving profile={camera_profile}", flush=True)

        camera = resolve_camera(
            camera_name=camera_profile,
            robot=robot,
        )
        last_status = None
        last_marker_count = None

        while not stop_event.is_set():
            timestamp = time.time()

            try:
                markers = camera.see()
                marker_count = len(markers)

                if last_status != "ok" or last_marker_count != marker_count:
                    print(
                        f"[VISION_WORKER] camera={camera_name} status=ok markers={marker_count}",
                        flush=True,
                    )
                    last_status = "ok"
                    last_marker_count = marker_count

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
            try:
                while not output_queue.empty():
                    output_queue.get_nowait()
            except Exception:
                pass

            output_queue.put(vision_message)

    finally:
        if camera is not None and hasattr(camera, "close"):
            try:
                camera.close()
            except Exception:
                pass