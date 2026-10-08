# skills/perception/reacquire_target_sweep.py


import math

from config import CONFIG
from primitives.base import Primitive, PrimitiveStatus
from skills.navigation.search_sweep import SearchSweep
from skills.perception.select_target_utils import get_closest_target


class ReacquireTarget(Primitive):
    def __init__(
        self,
        *,
        kind: str,
        step_deg: float,
        max_sweep_deg: float,
        max_age_s: float,
        target_id: int | None = None,
        cap_rel_deg: float = 30.0,
        **_,
    ):
        super().__init__()
        self.kind = kind
        self.target_id = int(target_id) if target_id is not None else None

        self.step_deg = float(step_deg)
        self.max_sweep_deg = float(max_sweep_deg)
        self.max_age_s = float(max_age_s)

        self.cap_rel_deg = float(cap_rel_deg)

        self.vision_loss_s = float(
            getattr(
                CONFIG,
                "reacquire_target_vision_loss",
                3.0,
            )
        )

        self._io = None
        self._sweep: SearchSweep | None = None
        self._sweep_started = False

        self.found_target = None

    def start(
            self,
            *,
            motion_backend,
            io,
            **_,
    ):
        self._io = io
        self._sweep = None
        self._sweep_started = False
        self.found_target = None

        cap = min(
            abs(self.cap_rel_deg),
            2.0 * abs(self.step_deg),
        )

        if cap < 1e-6:
            print(
                "[REACQUIRE] "
                "invalid sweep cap -> FAILED"
            )
            return PrimitiveStatus.FAILED

        angles_deg = (
            +cap,
            -(2.0 * cap),
            +cap,
        )

        angular_speed_rad_s = abs(
            float(
                CONFIG.servoing_angular_max_rad_s
            )
        )

        timeout_s = self.vision_loss_s

        if (
                timeout_s > 0.0
                and angular_speed_rad_s > 0.0
        ):
            sweep_duration_s = (
                    math.radians(
                        sum(
                            abs(angle)
                            for angle in angles_deg
                        )
                    )
                    / angular_speed_rad_s
            )

            timeout_s = max(
                timeout_s,
                sweep_duration_s + 1.0,
            )

        self._sweep = SearchSweep(
            angles_deg=angles_deg,
            timeout_s=timeout_s,
            config=CONFIG,
            label="REACQUIRE_SWEEP",
        )

        print(
            "[REACQUIRE] "
            f"continuous sweep "
            f"cap={cap:.1f}deg "
            f"timeout={timeout_s:.2f}s"
        )

        return PrimitiveStatus.RUNNING

    def _try_reacquire(self, *, perception, now: float):
        if perception is None:
            return None

        if self.target_id is not None:
            from perception.perception import get_visible_targets

            visible = get_visible_targets(
                perception,
                self.kind,
                now=now,
                max_age_s=self.max_age_s,
            )
            for t in visible:
                if int(t.get("id", -1)) == self.target_id:
                    return t
            return None

        return get_closest_target(
            perception,
            self.kind,
            now=now,
            max_age_s=self.max_age_s,
        )

    def update(
            self,
            *,
            motion_backend,
            perception=None,
            **_,
    ):
        now = float(
            self._io.time()
        )

        target = self._try_reacquire(
            perception=perception,
            now=now,
        )

        # Preserve the old fast path:
        # do not start search motion if the target is already visible.
        if (
                not self._sweep_started
                and target is not None
        ):
            self.found_target = target

            print(
                "[REACQUIRE] "
                "target already visible -> SUCCEEDED"
            )

            return PrimitiveStatus.SUCCEEDED

        if self._sweep is None:
            print(
                "[REACQUIRE] "
                "missing sweep -> FAILED"
            )
            return PrimitiveStatus.FAILED

        if not self._sweep_started:
            st = self._sweep.start(
                motion_backend=motion_backend,
            )

            self._sweep_started = True

            if st == PrimitiveStatus.FAILED:
                return PrimitiveStatus.FAILED

            return PrimitiveStatus.RUNNING

        st = self._sweep.update(
            motion_backend=motion_backend,
            found_item=target,
        )

        if st == PrimitiveStatus.SUCCEEDED:
            self.found_target = (
                self._sweep.found_item
            )

            print(
                "[REACQUIRE] "
                "target reacquired -> SUCCEEDED"
            )

            return PrimitiveStatus.SUCCEEDED

        if st == PrimitiveStatus.FAILED:
            print(
                "[REACQUIRE] "
                "sweep complete -> FAILED"
            )

        return st

    def stop(
            self,
            *,
            motion_backend=None,
    ):
        if self._sweep is not None:
            self._sweep.stop(
                motion_backend=motion_backend,
            )

        self._sweep = None
        self._sweep_started = False

