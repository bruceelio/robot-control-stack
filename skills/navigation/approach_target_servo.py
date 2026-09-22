# skills/navigation/approach_target_servo.py

from __future__ import annotations

import math
import time
from typing import Optional

from skills.navigation.approach_target import _marker_elevation
from motion_backends.velocity import VelocityMotionBackend
from navigation.servoing.servoing_controller import (
    ServoingController,
    ServoingStatus,
)
from perception.robot_geometry import target_from_gripper
from primitives.base import Primitive, PrimitiveStatus


class ApproachTargetServo(Primitive):
    """
    Stage 2 perception-guided approach to one locked target.

    Responsibilities:
        - follow the already-selected target id
        - convert camera-relative geometry to gripper-relative geometry
        - keep the shared HeightModel updated
        - dynamically choose LOW/HIGH pickup commit distance
        - feed continuous observations to ServoingController
        - execute VelocityCommand through VelocityMotionBackend
        - stop at pickup commit and expose final pickup geometry

    Does NOT:
        - select a different target
        - perform localisation/path planning
        - perform Stage 1 dead reckoning
        - decide fallback policy

    AcquireObject owns fallback between navigation stages.
    """

    FAILURE_PERCEPTION = "perception"
    FAILURE_HEIGHT = "height"

    def __init__(
        self,
        *,
        config,
        kind: str,
        height_model,
        locked_target_id: Optional[int],
    ):
        super().__init__()

        self.config = config
        self.kind = kind
        self.height_model = height_model

        self.locked_target_id = (
            int(locked_target_id)
            if locked_target_id is not None
            else None
        )

        self.controller = ServoingController()
        self.velocity_backend = None

        self.failure_reason: Optional[str] = None

        self.final_distance_mm: Optional[float] = None
        self.final_bearing_deg: Optional[float] = None
        self.target_is_high: Optional[bool] = None

        self._last_height_timestamp: Optional[float] = None

    @property
    def approached_target_id(self) -> Optional[int]:
        return self.locked_target_id

    def start(self, *, lvl2, **_):
        self.failure_reason = None

        self.final_distance_mm = None
        self.final_bearing_deg = None
        self.target_is_high = None

        self._last_height_timestamp = None

        self.velocity_backend = VelocityMotionBackend(
            lvl2=lvl2,
            config=self.config,
        )

        self.status = PrimitiveStatus.RUNNING

        print(
            "[SERVO_APPROACH] start "
            f"kind={self.kind} "
            f"target_id={self.locked_target_id}"
        )

        return self.status

    def _get_locked_target(self, perception):
        if self.locked_target_id is None:
            return None

        memory = perception.objects.get(self.kind, {})

        return memory.get(self.locked_target_id)

    def _update_height_model(
            self,
            *,
            target,
            distance_mm: float,
            observation_timestamp: float,
    ):
        """
        Update HeightModel once per fresh camera observation.

        Uses the same height evidence and commit rules as
        Stage 1 ApproachTarget.
        """

        # Do not count the same camera frame more than once.
        if self._last_height_timestamp == observation_timestamp:
            return

        self._last_height_timestamp = observation_timestamp

        # Match Stage 1: only classify height while within the
        # configured useful height-measurement range.
        if (
                distance_mm
                > float(self.config.marker_height_max_distance_mm)
        ):
            return

        pitch, src = _marker_elevation(
            target.get("marker", target),
        )

        print(
            f"[SERVO_APPROACH][HEIGHT][RAW] "
            f"src={src} pitch={pitch:.3f}"
        )

        if src != "none":
            self.height_model.update(
                pitch_rad=pitch,
                distance_mm=float(distance_mm),
                high_thresh=float(
                    self.config.marker_pitch_high_deg
                ),
                low_thresh=float(
                    self.config.marker_pitch_low_deg
                ),
            )

        if not self.height_model.is_committed():
            decision = self.height_model.try_commit(
                distance_mm=float(distance_mm),
                high_thresh=float(
                    self.config.marker_pitch_high_deg
                ),
                low_thresh=float(
                    self.config.marker_pitch_low_deg
                ),
                decision_deadline_mm=float(
                    self.config.height_decision_deadline_mm
                ),
            )

            if decision.committed:
                print(
                    "[SERVO_APPROACH][HEIGHT] committed "
                    f"{'HIGH' if self.height_model.is_high() else 'LOW'} "
                    f"reason={decision.reason} "
                    f"score={self.height_model.score:.2f} "
                    f"samples={self.height_model.samples}"
                )

    def _commit_distance_mm(self) -> float:
        if (
            self.height_model.is_committed()
            and self.height_model.is_high()
        ):
            return float(
                self.config.final_commit_distance_high_mm
            )

        # UNKNOWN deliberately uses LOW geometry while approaching.
        return float(
            self.config.final_commit_distance_mm
        )

    def update(self, *, perception, **_) -> PrimitiveStatus:
        if self.velocity_backend is None:
            self.failure_reason = self.FAILURE_PERCEPTION
            return PrimitiveStatus.FAILED

        now = time.time()

        target = self._get_locked_target(perception)

        if target is None:
            print(
                "[SERVO_APPROACH] locked target missing "
                "-> Stage 2 unavailable"
            )

            self.velocity_backend.stop()
            self.failure_reason = self.FAILURE_PERCEPTION
            return PrimitiveStatus.FAILED

        timestamp = float(
            target.get("last_seen", 0.0)
        )

        camera_distance = float(target["distance"])
        camera_bearing = float(target["bearing"])

        distance_mm, bearing_deg = target_from_gripper(
            distance_mm=camera_distance,
            bearing_deg=camera_bearing,
            config=self.config,
        )

        self._update_height_model(
            target=target,
            distance_mm=distance_mm,
            observation_timestamp=timestamp,
        )

        commit_distance_mm = self._commit_distance_mm()

        # Same safety rule as Stage 1:
        # UNKNOWN may approach using LOW geometry,
        # but may not cross the LOW commit point.
        if (
            distance_mm <= commit_distance_mm
            and not self.height_model.is_committed()
        ):
            print(
                "[SERVO_APPROACH][HEIGHT] "
                f"unresolved at commit "
                f"distance={distance_mm:.0f}mm "
                f"commit={commit_distance_mm:.0f}mm"
            )

            self.velocity_backend.stop()
            self.failure_reason = self.FAILURE_HEIGHT
            return PrimitiveStatus.FAILED

        result = self.controller.update(
            distance_mm=distance_mm,
            target_angle_rad=math.radians(bearing_deg),
            timestamp=timestamp,
            stop_distance_mm=commit_distance_mm,
            now=now,
        )

        if result.status == ServoingStatus.FAILED_STALE:
            # Controller has already commanded zero velocity because the
            # observation is too old for closed-loop driving.
            self.velocity_backend.update(result.command)

            age_s = now - timestamp

            # Do not abandon Stage 2 for a brief missed-tag interval.
            # Keep stopped and wait for fresh perception to return.
            fail_age_s = (
                    float(self.config.visible_max_age_s)
                    + float(self.config.vision_loss_timeout_s)
            )

            if age_s <= fail_age_s:
                print(
                    "[SERVO_APPROACH] perception stale "
                    f"age={age_s:.3f}s "
                    f"waiting until {fail_age_s:.3f}s"
                )
                return PrimitiveStatus.RUNNING

            print(
                "[SERVO_APPROACH] perception stale "
                f"age={age_s:.3f}s "
                "-> Stage 2 unavailable"
            )

            self.failure_reason = self.FAILURE_PERCEPTION
            return PrimitiveStatus.FAILED

        if result.status == ServoingStatus.SUCCESS:
            self.velocity_backend.update(result.command)

            if not self.height_model.is_committed():
                self.failure_reason = self.FAILURE_HEIGHT
                return PrimitiveStatus.FAILED

            self.final_distance_mm = float(distance_mm)
            self.final_bearing_deg = float(bearing_deg)
            latest_height = self.height_model.last_good_is_high()

            self.target_is_high = (
                latest_height
                if latest_height is not None
                else self.height_model.is_high()
            )

            print(
                "[SERVO_APPROACH][HANDOFF] "
                f"mode={'HIGH' if self.target_is_high else 'LOW'} "
                f"dist={distance_mm:.0f}mm "
                f"bearing={bearing_deg:.1f}deg "
                f"commit={commit_distance_mm:.0f}mm "
                "-> FINAL_PICKUP"
            )

            return PrimitiveStatus.SUCCEEDED

        self.velocity_backend.update(result.command)

        print(
            "[SERVO_APPROACH] "
            f"dist={distance_mm:.0f}mm "
            f"bearing={bearing_deg:+.1f}deg "
            f"commit={commit_distance_mm:.0f}mm "
            f"vx={result.command.linear_x_mps:.3f}m/s "
            f"wz={result.command.angular_z_rps:+.3f}rad/s"
        )

        return PrimitiveStatus.RUNNING

    def stop(self, **_):
        if self.velocity_backend is not None:
            self.velocity_backend.stop()