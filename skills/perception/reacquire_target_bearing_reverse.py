# skills/perception/reacquire_target_bearing_reverse.py

from __future__ import annotations

import math
from typing import Optional

from motion_backends.velocity import VelocityMotionBackend
from navigation.command.velocity_arbiter import VelocityCommand
from primitives.base import Primitive, PrimitiveStatus
from primitives.motion import Rotate


class ReacquireTargetBearingReverse(Primitive):
    """
    Reacquire one locked target after a manoeuvre has moved the
    robot too close or too far off-axis to see it.

    Flow:

        target visible
            -> success

        otherwise
            -> rotate toward expected target bearing

        target still not visible
            -> reverse while checking vision continuously

        target reappears
            -> stop immediately
            -> success

        reverse budget exhausted
            -> fail

    This skill never selects a different target and does not own
    higher-level recovery policy.
    """

    def __init__(
        self,
        *,
        config,
        kind: str,
        target_id: int,
        expected_bearing_rad: float,
        reverse_limit_mm: float,
        max_age_s: float,
        reverse_linear_mps: Optional[float] = None,
    ):
        super().__init__()

        self.config = config
        self.kind = str(kind)
        self.target_id = int(target_id)

        self.expected_bearing_rad = float(
            expected_bearing_rad
        )

        self.reverse_limit_mm = max(
            0.0,
            float(reverse_limit_mm),
        )

        self.max_age_s = float(max_age_s)

        if reverse_linear_mps is None:
            reverse_linear_mps = float(
                getattr(
                    config,
                    "reacquire_bearing_reverse_linear_mps",
                    0.20,
                )
            )

        self.reverse_linear_mps = abs(
            float(reverse_linear_mps)
        )

        self.found_target = None
        self.failure_reason = None

        self._phase = "CHECK"

        self._align = None
        self._velocity_backend = None
        self._io = None

        self._reverse_distance_mm = 0.0
        self._last_reverse_time_s = None
        self._reverse_started_logged = False

    def start(
        self,
        *,
        lvl2,
        motion_backend,
        io,
        **_,
    ):
        self.found_target = None
        self.failure_reason = None

        self._phase = "CHECK"

        self._align = None

        self._velocity_backend = (
            VelocityMotionBackend(
                lvl2=lvl2,
                config=self.config,
            )
        )

        self._io = io

        self._reverse_distance_mm = 0.0
        self._last_reverse_time_s = None
        self._reverse_started_logged = False

        self.status = PrimitiveStatus.RUNNING

        print(
            "[REACQUIRE_BEARING_REVERSE] start "
            f"id={self.target_id} "
            f"expected="
            f"{math.degrees(self.expected_bearing_rad):+.1f}deg "
            f"reverse_limit={self.reverse_limit_mm:.0f}mm "
            f"speed={self.reverse_linear_mps:.2f}m/s"
        )

        return self.status

    def _try_reacquire(
        self,
        *,
        perception,
    ):
        if perception is None:
            return None

        # Perception timestamps currently use the wall-clock
        # domain, so do not pass io.time() here.
        from perception.perception import (
            get_visible_targets,
        )

        visible = get_visible_targets(
            perception,
            self.kind,
            max_age_s=self.max_age_s,
        )

        for target in visible:
            try:
                target_id = int(
                    target.get("id", -1)
                )
            except (TypeError, ValueError):
                continue

            if target_id == self.target_id:
                return target

        return None

    def _succeed(
        self,
        *,
        target,
        motion_backend,
    ):
        if self._align is not None:
            try:
                self._align.stop(
                    motion_backend=motion_backend
                )
            except TypeError:
                self._align.stop()

            self._align = None

        if self._velocity_backend is not None:
            self._velocity_backend.stop()

        self.found_target = target
        self.failure_reason = None
        self.status = PrimitiveStatus.SUCCEEDED

        print(
            "[REACQUIRE_BEARING_REVERSE] "
            f"reacquired id={self.target_id} "
            f"phase={self._phase} "
            f"reverse={self._reverse_distance_mm:.0f}mm"
        )

        return self.status

    def _fail(
        self,
        *,
        reason: str,
    ):
        if self._velocity_backend is not None:
            self._velocity_backend.stop()

        self.failure_reason = str(reason)
        self.status = PrimitiveStatus.FAILED

        print(
            "[REACQUIRE_BEARING_REVERSE] "
            f"FAILED reason={self.failure_reason} "
            f"reverse={self._reverse_distance_mm:.0f}mm"
        )

        return self.status

    def update(
        self,
        *,
        perception,
        motion_backend,
        **_,
    ):
        if self.status != PrimitiveStatus.RUNNING:
            return self.status

        # Check the locked target before doing any motion.
        # This also means a target which reappears DURING
        # alignment or reverse causes an immediate stop.
        target = self._try_reacquire(
            perception=perception
        )

        if target is not None:
            return self._succeed(
                target=target,
                motion_backend=motion_backend,
            )

        if (
            self._velocity_backend is None
            or self._io is None
        ):
            return self._fail(
                reason="not_started"
            )

        # ------------------------------------------
        # Align toward propagated expected bearing
        # ------------------------------------------

        if self._phase == "CHECK":
            angle_deg = math.degrees(
                self.expected_bearing_rad
            )

            max_rotate_deg = abs(
                float(
                    self.config.max_rotate_deg
                )
            )

            angle_deg = max(
                -max_rotate_deg,
                min(
                    max_rotate_deg,
                    angle_deg,
                ),
            )

            min_rotate_deg = abs(
                float(
                    self.config.min_rotate_deg
                )
            )

            if abs(angle_deg) < min_rotate_deg:
                print(
                    "[REACQUIRE_BEARING_REVERSE] "
                    f"alignment residual="
                    f"{angle_deg:+.1f}deg "
                    "below executable rotation "
                    "-> reverse"
                )

                self._phase = "REVERSE"

            else:
                print(
                    "[REACQUIRE_BEARING_REVERSE] "
                    "align toward expected target "
                    f"{angle_deg:+.1f}deg"
                )

                self._align = Rotate(
                    angle_deg=angle_deg
                )

                self._align.start(
                    motion_backend=motion_backend
                )

                self._phase = "ALIGN"

                return self.status

        if self._phase == "ALIGN":
            if self._align is None:
                return self._fail(
                    reason="align_missing"
                )

            st = self._align.update(
                motion_backend=motion_backend
            )

            if st == PrimitiveStatus.RUNNING:
                return self.status

            self._align = None

            if st == PrimitiveStatus.FAILED:
                return self._fail(
                    reason="align_failed"
                )

            print(
                "[REACQUIRE_BEARING_REVERSE] "
                "alignment complete; "
                "target still not visible -> reverse"
            )

            self._phase = "REVERSE"

        # ------------------------------------------
        # Interruptible reverse
        # ------------------------------------------

        if self._phase == "REVERSE":
            if (
                self.reverse_limit_mm <= 0.0
                or self.reverse_linear_mps <= 0.0
            ):
                return self._fail(
                    reason="reverse_unavailable"
                )

            now_s = float(
                self._io.time()
            )

            if self._last_reverse_time_s is None:
                self._last_reverse_time_s = now_s

            else:
                dt_s = max(
                    0.0,
                    now_s
                    - self._last_reverse_time_s,
                )

                self._last_reverse_time_s = now_s

                self._reverse_distance_mm += (
                    self.reverse_linear_mps
                    * 1000.0
                    * dt_s
                )

            if (
                self._reverse_distance_mm
                >= self.reverse_limit_mm
            ):
                return self._fail(
                    reason="reverse_limit"
                )

            if not self._reverse_started_logged:
                print(
                    "[REACQUIRE_BEARING_REVERSE] "
                    "reverse while watching "
                    f"id={self.target_id}"
                )

                self._reverse_started_logged = True

            command = VelocityCommand(
                linear_x_mps=(
                    -self.reverse_linear_mps
                ),
                angular_z_rps=0.0,
                lateral_y_mps=0.0,
                timestamp=now_s,
            )

            self._velocity_backend.update(
                command
            )

            return self.status

        return self._fail(
            reason="invalid_phase"
        )

    def stop(
        self,
        *,
        motion_backend=None,
        **_,
    ):
        if self._align is not None:
            try:
                if motion_backend is not None:
                    self._align.stop(
                        motion_backend=motion_backend
                    )
                else:
                    self._align.stop()
            except Exception:
                pass

        self._align = None

        if self._velocity_backend is not None:
            self._velocity_backend.stop()

        self._last_reverse_time_s = None