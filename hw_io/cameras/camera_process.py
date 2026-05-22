# hw_io/cameras/camera_process.py

from __future__ import annotations

import time
from multiprocessing import Event, Process, Queue
from typing import Any

from hw_io.cameras.vision_worker import run_vision_worker


class CameraProcessManager:
    """
    Starts and manages one vision worker process per enabled camera.

    Main robot code should use this manager to read latest VisionMessage data.
    """

    def __init__(self, *, camera_names: list[str], robot: Any):
        self.camera_names = list(camera_names)
        self.robot = robot

        self._queues: dict[str, Queue] = {}
        self._stop_events: dict[str, Event] = {}
        self._processes: dict[str, Process] = {}
        self._latest: dict[str, dict] = {}

    def start(self) -> None:
        for camera_name in self.camera_names:
            if camera_name in self._processes:
                continue

            queue = Queue(maxsize=1)
            stop_event = Event()

            process = Process(
                target=run_vision_worker,
                kwargs={
                    "camera_name": camera_name,
                    "robot": self.robot,
                    "output_queue": queue,
                    "stop_event": stop_event,
                },
                daemon=True,
            )

            self._queues[camera_name] = queue
            self._stop_events[camera_name] = stop_event
            self._processes[camera_name] = process

            process.start()
            print(f"[CAMERA_PROCESS] started {camera_name} pid={process.pid}")

    def poll(self) -> None:
        """
        Pull newest messages from worker queues into local latest cache.

        Non-blocking.
        """
        for camera_name, process in self._processes.items():
            if not process.is_alive():
                print(
                    f"[CAMERA_PROCESS] "
                    f"{camera_name} process died "
                    f"exitcode={process.exitcode}"
                )

        for camera_name, queue in self._queues.items():
            latest = None

            while True:
                try:
                    latest = queue.get_nowait()
                except Exception:
                    break

            if latest is not None:
                self._latest[camera_name] = latest

    def get_latest(self, camera_name: str) -> dict | None:
        self.poll()
        return self._latest.get(camera_name)

    def get_latest_messages(self) -> dict[str, dict]:
        self.poll()
        return dict(self._latest)

    def get_fresh_messages(self, *, max_age_s: float, now: float | None = None) -> dict[str, dict]:
        if now is None:
            now = time.time()

        self.poll()

        fresh = {}
        for camera_name, message in self._latest.items():
            timestamp = float(message.get("timestamp", 0.0))
            if now - timestamp <= max_age_s:
                fresh[camera_name] = message

        return fresh

    def stop(self, *, timeout_s: float = 1.0) -> None:
        for stop_event in self._stop_events.values():
            stop_event.set()

        for process in self._processes.values():
            process.join(timeout=timeout_s)

        for process in self._processes.values():
            if process.is_alive():
                process.terminate()

        self._processes.clear()
        self._queues.clear()
        self._stop_events.clear()
        self._latest.clear()