# hw_io/hw_pi.py

from config import CONFIG
from hw_io.cameras.async_camera_proxy import AsyncCameraProxy
from hw_io.cameras.resolve import resolve_camera
from hw_io.clients import UsbMediaClient


class PiReadOnlyCollection:
    def __init__(self, getters):
        self._getters = dict(getters)

    def __getitem__(self, key):
        return self._getters[key]()

    def keys(self):
        return self._getters.keys()


class PiBackend:
    """
    Raspberry Pi-local IO provider.

    This is a partial backend, not a complete IOMap.
    """

    def __init__(self, robot=None, camera_manager=None):
        self.robot = robot
        self.camera_manager = camera_manager
        self.usb_media = UsbMediaClient()

        self._camera = {}
        self._usb = PiReadOnlyCollection({
            "match_zone": self._read_match_zone,
        })

        self._detect_cameras()

    @property
    def camera(self):
        return self._camera

    @property
    def usb(self):
        return self._usb

    def _detect_cameras(self):
        if not bool(getattr(CONFIG, "cameras_enabled", True)):
            return

        if bool(getattr(CONFIG, "async_vision_enabled", False)):
            if self.camera_manager is None:
                print(
                    "[PI CAMERA WARN] async vision enabled "
                    "but no camera_manager provided"
                )
                return

            for name in CONFIG.cameras.keys():
                self._camera[name] = AsyncCameraProxy(
                    camera_name=name,
                    camera_manager=self.camera_manager,
                )
                print(f"[PI CAMERA] async proxy connected: {name}")

            return

        for name, camera_config in CONFIG.cameras.items():
            try:
                self._camera[name] = resolve_camera(
                    camera_name=camera_config["profile"],
                    device=camera_config["device"],
                    robot=self.robot,
                )
                print(f"[PI CAMERA] connected: {name}")

            except Exception as exc:
                print(
                    f"[PI CAMERA WARN] "
                    f"{name} unavailable: {exc}"
                )

    def _read_match_zone(self) -> int:
        source = CONFIG.match_zone_source.value.lower()

        if "auto" in source:
            try:
                zone = self.usb_media.read_int(
                    CONFIG.usb_match_zone_file
                )
                print(f"[MATCH ZONE] USB zone={zone}")
                return int(zone)
            except Exception:
                print(
                    f"[MATCH ZONE] FIXED "
                    f"zone={CONFIG.match_zone_fixed}"
                )
                return int(CONFIG.match_zone_fixed)

        if "usb" in source:
            return int(
                self.usb_media.read_int(
                    CONFIG.usb_match_zone_file
                )
            )

        if "fixed" in source:
            return int(CONFIG.match_zone_fixed)

        if "sr" in source:
            if self.robot is None:
                raise RuntimeError(
                    "SR match zone requested without SR robot"
                )
            return int(self.robot.zone)

        raise ValueError(
            f"Unknown MATCH_ZONE_SOURCE: "
            f"{CONFIG.match_zone_source}"
        )