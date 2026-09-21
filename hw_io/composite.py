# hw_io/composite.py

from __future__ import annotations

import time
from typing import Any, Dict, Optional

from hw_io.base import IOMap


class RoutedCollection:
    """
    Presents one semantic IO category while routing each named device
    to the backend selected by CONFIG.io.
    """

    def __init__(self, category: str, routes: dict[str, Any]):
        self._category = category
        self._routes = dict(routes)

    def __getitem__(self, name):
        backend = self._routes[name]
        collection = getattr(backend, self._category)
        return collection[name]

    def keys(self):
        return self._routes.keys()

    def values(self):
        return [
            self[name]
            for name in self._routes
        ]

    def items(self):
        return [
            (name, self[name])
            for name in self._routes
        ]

    def as_dict(self):
        return {
            name: self[name]
            for name in self._routes
        }

    def __len__(self):
        return len(self._routes)


class CompositeIO(IOMap):
    """
    Combines multiple hardware backends into one canonical IOMap.

    CONFIG.io decides which backend owns each semantic IO device.
    """

    def __init__(
        self,
        *,
        backends: dict[str, Any],
        io_config: dict[str, str | None],
    ):
        self._backends = dict(backends)
        self._io_config = dict(io_config)

        self._category_routes: dict[str, dict[str, Any]] = {}

        for semantic_name, backend_name in self._io_config.items():
            if backend_name is None:
                continue

            if backend_name not in self._backends:
                raise RuntimeError(
                    f"IO {semantic_name!r} requests backend "
                    f"{backend_name!r}, but it was not created"
                )

            category, name = semantic_name.split(".", 1)

            backend = self._backends[backend_name]

            collection = getattr(backend, category, None)
            if collection is None:
                raise RuntimeError(
                    f"Backend {backend_name!r} does not expose "
                    f"IO category {category!r} "
                    f"required by {semantic_name!r}"
                )

            if name not in collection.keys():
                raise RuntimeError(
                    f"Backend {backend_name!r} does not expose "
                    f"{semantic_name!r}"
                )

            self._category_routes.setdefault(
                category,
                {}
            )[name] = backend

    def _collection(self, category: str):
        return RoutedCollection(
            category,
            self._category_routes.get(category, {}),
        )


    # --------------------------------------------------
    # Canonical collections
    # --------------------------------------------------

    @property
    def bumper(self):
        return self._collection("bumper")

    @property
    def reflectance(self):
        return self._collection("reflectance")

    @property
    def ultrasonic(self):
        return self._collection("ultrasonic")

    @property
    def current(self):
        return self._collection("current")

    @property
    def voltage(self):
        return self._collection("voltage")

    @property
    def encoder(self):
        return self._collection("encoder")

    @property
    def camera(self):
        return self._collection("camera")

    @property
    def motor(self):
        return self._collection("motor")

    @property
    def drive(self):
        return self._collection("drive")

    @property
    def servo(self):
        return self._collection("servo")

    @property
    def limit(self):
        return self._collection("limit")

    @property
    def led(self):
        return self._collection("led")

    @property
    def audio(self):
        return self._collection("audio")

    @property
    def usb(self):
        return self._collection("usb")

    # --------------------------------------------------
    # Compatibility interfaces
    # --------------------------------------------------

    def cameras(self) -> Dict[str, Any]:
        return {
            name: self.camera[name]
            for name in self.camera.keys()
        }

    def bumpers(self) -> Dict[str, bool]:
        return self.bumper.as_dict()

    def reflectance_values(self) -> Dict[str, float]:
        return self.reflectance.as_dict()

    def ultrasonics(self) -> Dict[str, Optional[float]]:
        return self.ultrasonic.as_dict()

    @property
    def motors(self):
        return self.motor

    @property
    def servos(self):
        return self.servo

    # --------------------------------------------------
    # Outputs / system services
    # --------------------------------------------------

    @property
    def outputs(self):
        for backend in self._backends.values():
            outputs = getattr(backend, "outputs", None)
            if outputs is not None:
                return outputs

        return None

    def battery(self) -> Dict[str, Optional[float]]:
        result = {}

        if "battery" in self.voltage.keys():
            result["voltage"] = self.voltage["battery"].volts

        if "battery" in self.current.keys():
            result["current"] = self.current["battery"].amps

        return result

    @property
    def battery_sensor(self):
        return self.battery()

    def sleep(self, secs: float) -> None:
        # Prefer a backend-provided sleep because the Mega implementation,
        # for example, maintains its heartbeat while sleeping.
        for backend in self._backends.values():
            sleep_fn = getattr(backend, "sleep", None)

            if callable(sleep_fn):
                sleep_fn(secs)
                return

        time.sleep(secs)

    # --------------------------------------------------
    # Unified sensor snapshot
    # --------------------------------------------------

    def sense(self) -> Dict[str, Any]:
        return {
            "bumper": self.bumpers(),
            "reflectance": self.reflectance_values(),
            "ultrasonic": self.ultrasonics(),
            "limit": self.limit.as_dict(),
            "voltage": {
                name: self.voltage[name].volts
                for name in self.voltage.keys()
            },
            "current": {
                name: self.current[name].amps
                for name in self.current.keys()
            },
        }

    # --------------------------------------------------
    # Optional capabilities
    # --------------------------------------------------

    def kch(self):
        for backend in self._backends.values():
            value = getattr(backend, "kch", None)
            if value is not None:
                return value

        return None

    def buzzer(self):
        if "piezo" not in self.audio.keys():
            return None

        return self.audio["piezo"]

    def wait_start(self):
        for backend in self._backends.values():
            fn = getattr(backend, "wait_start", None)

            if callable(fn):
                return fn()

        return None

    def close(self):
        for backend in self._backends.values():
            fn = getattr(backend, "close", None)

            if callable(fn):
                fn()