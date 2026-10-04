# skills/perception/acquire_wall_geometry.py

from __future__ import annotations

import math
import time

from navigation.wall_geometry.models import RangeRay2D
from primitives.base import Primitive, PrimitiveStatus
from primitives.motion import Rotate


class AcquireWallGeometry(Primitive):
    """
    Active perception for wall-geometry evidence.

    Current implementation:
        SCAN_SINGLE_SENSOR

    It deliberately returns RangeRay2D observations rather than
    declaring that they all belong to one wall.

    Wall association / corner discrimination belongs to the
    wall-geometry interpretation layer.
    """

    def __init__(
        self,
        *,
        config,
        wall_status,
    ):
        super().__init__()

        self.config = config
        self.wall_status = wall_status

        self.range_rays: tuple[RangeRay2D, ...] = ()
        self.failure_reason: str | None = None
        self.sensor_key: str | None = None

        self._io = None
        self._mount = None

        self._angles_deg: tuple[float, ...] = ()
        self._angle_index = 0
        self._current_angle_deg = 0.0
        self._pending_angle_deg = 0.0

        self._samples: list[float] = []
        self._sample_attempts = 0

        self._rays: list[RangeRay2D] = []

        self._rotate: Rotate | None = None
        self._settle_until: float | None = None

        self._phase = "IDLE"
        self._fail_after_recenter = False

        # Generic active-perception acquisition policy.
        #
        # New configuration may override these using the
        # wall_acquisition_* names below. The defaults preserve the
        # existing +/-8 degree, 3-sample, 0.10 s scan behaviour
        # without depending on the retired wall-angle configuration.

        self.scan_angles_deg = tuple(
            getattr(
                config,
                "wall_acquisition_scan_angles_deg",
                (-8.0, 8.0),
            )
        )

        self.samples_per_angle = int(
            getattr(
                config,
                "wall_acquisition_samples_per_angle",
                3,
            )
        )

        self.settle_time_s = float(
            getattr(
                config,
                "wall_acquisition_settle_time_s",
                0.10,
            )
        )

    # --------------------------------------------------
    # Public API
    # --------------------------------------------------

    def start(
        self,
        *,
        perception=None,
        motion_backend=None,
        **_,
    ):
        self.range_rays = ()
        self.failure_reason = None

        self._rays = []
        self._samples = []
        self._sample_attempts = 0

        self._angle_index = 0
        self._current_angle_deg = 0.0
        self._pending_angle_deg = 0.0

        self._rotate = None
        self._settle_until = None
        self._fail_after_recenter = False

        action = getattr(
            getattr(self.wall_status, "action", None),
            "value",
            None,
        )

        if action != "scan_single_sensor":
            return self._fail(
                f"unsupported_action:{action}"
            )

        self._io = self._resolve_io(
            perception=perception,
            motion_backend=motion_backend,
        )

        if self._io is None:
            return self._fail("no_io")

        candidates = self._configured_range_sensors()

        if len(candidates) != 1:
            return self._fail(
                "scan_single_sensor_requires_"
                f"exactly_one_sensor:{len(candidates)}"
            )

        self.sensor_key = candidates[0]

        self._mount = (
            self.config
            .range_sensor_mounts[self.sensor_key]
        )

        if len(self.scan_angles_deg) < 2:
            return self._fail(
                "scan_single_sensor_requires_"
                "at_least_two_scan_angles"
            )

        # Include the unrotated observation as well as the configured
        # scan headings.
        #
        # Three observations are more useful than the old two-ray
        # wall-angle scan because the interpretation layer can test
        # whether the hit points are actually consistent with one wall.
        self._angles_deg = (
            0.0,
            *(
                float(angle_deg)
                for angle_deg
                in self.scan_angles_deg
            ),
        )

        self._phase = "SETTLE"

        self._settle_until = (
            time.monotonic()
            + self.settle_time_s
        )

        self.status = PrimitiveStatus.RUNNING

        print(
            "[WALL_ACQUIRE] start "
            f"sensor={self.sensor_key} "
            f"angles={self._angles_deg}"
        )

        return self.status

    def update(
        self,
        *,
        perception=None,
        motion_backend=None,
        **_,
    ):
        if self.status != PrimitiveStatus.RUNNING:
            return self.status

        if self._phase == "ROTATE":
            return self._update_rotate(
                motion_backend=motion_backend,
            )

        if self._phase == "SETTLE":
            return self._update_settle()

        if self._phase == "SAMPLE":
            return self._update_sample(
                motion_backend=motion_backend,
            )

        if self._phase == "RECENTER":
            return self._update_recenter(
                motion_backend=motion_backend,
            )

        return self._fail("invalid_phase")

    def stop(
        self,
        *,
        motion_backend=None,
        **_,
    ):
        if self._rotate is not None:
            try:
                self._rotate.stop(
                    motion_backend=motion_backend
                )
            except Exception:
                pass

        self._rotate = None
        self.status = PrimitiveStatus.FAILED

    # --------------------------------------------------
    # Capability / IO
    # --------------------------------------------------

    def _resolve_io(
        self,
        *,
        perception,
        motion_backend,
    ):
        if motion_backend is not None:
            io = getattr(
                motion_backend,
                "io",
                None,
            )

            if io is not None:
                return io

            lvl2 = getattr(
                motion_backend,
                "lvl2",
                None,
            )

            if lvl2 is not None:
                io = getattr(
                    lvl2,
                    "io",
                    None,
                )

                if io is not None:
                    return io

        if perception is not None:
            io = getattr(
                perception,
                "io",
                None,
            )

            if io is not None:
                return io

            io = getattr(
                perception,
                "_io",
                None,
            )

            if io is not None:
                return io

        return None

    def _configured_range_sensors(
        self,
    ) -> tuple[str, ...]:

        io_map = getattr(
            self.config,
            "io",
            {},
        ) or {}

        mounts = getattr(
            self.config,
            "range_sensor_mounts",
            {},
        ) or {}

        result: list[str] = []

        for sensor_key, mount in mounts.items():

            sensor_key = str(sensor_key)

            if io_map.get(sensor_key) is None:
                continue

            if not isinstance(mount, dict):
                continue

            has_position = (
                (
                    "x_mm" in mount
                    and "y_mm" in mount
                )
                or (
                    "x_m" in mount
                    and "y_m" in mount
                )
            )

            has_heading = (
                "yaw_deg" in mount
                or "yaw_rad" in mount
            )

            if has_position and has_heading:
                result.append(sensor_key)

        return tuple(result)

    # --------------------------------------------------
    # Sensor reading
    # --------------------------------------------------

    def _read_range_mm(
        self,
    ) -> float | None:

        if self.sensor_key is None:
            return None

        try:
            category, name = (
                self.sensor_key.split(".", 1)
            )
        except ValueError:
            return None

        try:
            collection = getattr(
                self._io,
                category,
            )
            raw = collection[name]
        except Exception:
            return None

        try:
            value = float(raw)
        except (TypeError, ValueError):
            return None

        if (
            not math.isfinite(value)
            or value <= 0.0
        ):
            return None

        # Technology-specific range validity belongs at the sensor /
        # perception-provider boundary rather than in this generic
        # active-acquisition state machine.
        #
        # At this layer a usable range is simply finite and positive.
        # Sensor-specific min/max limits can later be supplied through
        # normalized sensor capability metadata.

        return value

    # --------------------------------------------------
    # Range-ray construction
    # --------------------------------------------------

    def _mount_xy_mm(
        self,
    ) -> tuple[float, float]:

        mount = self._mount

        if (
            "x_mm" in mount
            and "y_mm" in mount
        ):
            return (
                float(mount["x_mm"]),
                float(mount["y_mm"]),
            )

        return (
            1000.0 * float(mount["x_m"]),
            1000.0 * float(mount["y_m"]),
        )

    def _mount_yaw_rad(
        self,
    ) -> float:

        if "yaw_rad" in self._mount:
            return float(
                self._mount["yaw_rad"]
            )

        return math.radians(
            float(
                self._mount["yaw_deg"]
            )
        )

    def _make_ray(
        self,
        *,
        distance_mm: float,
        robot_angle_deg: float,
    ) -> RangeRay2D:

        robot_angle_rad = math.radians(
            robot_angle_deg
        )

        sensor_x_mm, sensor_y_mm = (
            self._mount_xy_mm()
        )

        c = math.cos(robot_angle_rad)
        s = math.sin(robot_angle_rad)

        # Express the sensor origin in the base_link frame
        # which existed at the start of the scan.
        origin_x_mm = (
            c * sensor_x_mm
            - s * sensor_y_mm
        )

        origin_y_mm = (
            s * sensor_x_mm
            + c * sensor_y_mm
        )

        ray_heading_rad = (
            robot_angle_rad
            + self._mount_yaw_rad()
        )

        return RangeRay2D(
            distance_mm=float(distance_mm),
            origin_x_mm=origin_x_mm,
            origin_y_mm=origin_y_mm,
            ray_heading_rad=ray_heading_rad,
        )

    # --------------------------------------------------
    # State machine
    # --------------------------------------------------

    def _update_settle(self):

        if (
            self._settle_until is not None
            and time.monotonic()
            < self._settle_until
        ):
            return self.status

        self._settle_until = None
        self._samples = []
        self._sample_attempts = 0
        self._phase = "SAMPLE"

        return self.status

    def _update_sample(
        self,
        *,
        motion_backend,
    ):
        required = max(
            1,
            self.samples_per_angle,
        )

        self._sample_attempts += 1

        value = self._read_range_mm()

        print(
            "[WALL_ACQUIRE][RANGE] "
            f"sensor={self.sensor_key} "
            f"angle={self._current_angle_deg:+.1f}deg "
            f"sample={self._sample_attempts} "
            f"value={value}"
        )

        if value is not None:
            self._samples.append(value)

        if len(self._samples) >= required:

            values = sorted(self._samples)

            distance_mm = values[
                len(values) // 2
            ]

            ray = self._make_ray(
                distance_mm=distance_mm,
                robot_angle_deg=(
                    self._current_angle_deg
                ),
            )

            self._rays.append(ray)

            print(
                "[WALL_ACQUIRE][RAY] "
                f"angle="
                f"{self._current_angle_deg:+.1f}deg "
                f"distance={distance_mm:.0f}mm"
            )

            return self._advance_angle(
                motion_backend=motion_backend,
            )

        max_attempts = required * 3

        if self._sample_attempts >= max_attempts:
            self.failure_reason = (
                "insufficient_valid_samples"
            )
            self._fail_after_recenter = True

            return self._begin_recenter(
                motion_backend=motion_backend,
            )

        return self.status

    def _advance_angle(
        self,
        *,
        motion_backend,
    ):
        self._angle_index += 1

        if self._angle_index >= len(
            self._angles_deg
        ):
            return self._begin_recenter(
                motion_backend=motion_backend,
            )

        target_deg = self._angles_deg[
            self._angle_index
        ]

        return self._begin_rotate_to(
            target_deg=target_deg,
            motion_backend=motion_backend,
        )

    def _begin_rotate_to(
        self,
        *,
        target_deg: float,
        motion_backend,
    ):
        delta_deg = (
            float(target_deg)
            - self._current_angle_deg
        )

        self._pending_angle_deg = float(
            target_deg
        )

        if abs(delta_deg) <= 1e-6:
            self._current_angle_deg = float(
                target_deg
            )
            self._phase = "SETTLE"
            self._settle_until = (
                time.monotonic()
                + self.settle_time_s
            )
            return self.status

        print(
            "[WALL_ACQUIRE][ROTATE] "
            f"from={self._current_angle_deg:+.1f}deg "
            f"to={target_deg:+.1f}deg "
            f"delta={delta_deg:+.1f}deg"
        )

        self._rotate = Rotate(
            angle_deg=delta_deg
        )

        self._rotate.start(
            motion_backend=motion_backend
        )

        self._phase = "ROTATE"

        return self.status

    def _update_rotate(
        self,
        *,
        motion_backend,
    ):
        if self._rotate is None:
            return self._fail(
                "missing_rotate"
            )

        st = self._rotate.update(
            motion_backend=motion_backend
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        if st == PrimitiveStatus.FAILED:
            return self._fail(
                "scan_rotate_failed"
            )

        self._rotate = None
        self._current_angle_deg = (
            self._pending_angle_deg
        )

        self._phase = "SETTLE"
        self._settle_until = (
            time.monotonic()
            + self.settle_time_s
        )

        return self.status

    def _begin_recenter(
        self,
        *,
        motion_backend,
    ):
        if abs(
            self._current_angle_deg
        ) <= 1e-6:
            return self._finish_recentered()

        print(
            "[WALL_ACQUIRE][RECENTER] "
            f"angle="
            f"{-self._current_angle_deg:+.1f}deg"
        )

        self._rotate = Rotate(
            angle_deg=-self._current_angle_deg
        )

        self._rotate.start(
            motion_backend=motion_backend
        )

        self._phase = "RECENTER"

        return self.status

    def _update_recenter(
        self,
        *,
        motion_backend,
    ):
        if self._rotate is None:
            return self._fail(
                "missing_recenter_rotate"
            )

        st = self._rotate.update(
            motion_backend=motion_backend
        )

        if st == PrimitiveStatus.RUNNING:
            return self.status

        self._rotate = None

        if st == PrimitiveStatus.FAILED:
            return self._fail(
                "recenter_failed"
            )

        self._current_angle_deg = 0.0

        return self._finish_recentered()

    def _finish_recentered(self):

        if self._fail_after_recenter:
            self.status = PrimitiveStatus.FAILED
            return self.status

        self.range_rays = tuple(
            self._rays
        )

        print(
            "[WALL_ACQUIRE] complete "
            f"rays={len(self.range_rays)}"
        )

        self.status = PrimitiveStatus.SUCCEEDED
        return self.status

    def _fail(
        self,
        reason: str,
    ):
        self.failure_reason = reason

        print(
            "[WALL_ACQUIRE] failed "
            f"reason={reason}"
        )

        self.status = PrimitiveStatus.FAILED
        return self.status