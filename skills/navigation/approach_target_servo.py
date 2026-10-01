# skills/navigation/approach_target_servo.py

from __future__ import annotations

import math
import time

from enum import Enum
from typing import Optional

from skills.perception.marker_elevation import _marker_elevation
from skills.perception.select_target import ApproachServoMethod
from motion_backends.velocity import VelocityMotionBackend
from navigation.command.velocity_arbiter import VelocityCommand
from navigation.control.distance_angle_controller import (
    DistanceAngleController,
    DistanceAngleParams,
)
from perception.robot_geometry import (
    relative_target_from_base_link,
    relative_target_from_gripper,
)

from navigation.geometry.relative_goal_pose import (
    relative_goal_pose_from_reference,
)
from primitives.base import Primitive, PrimitiveStatus

from calibration import CALIBRATION

from navigation.visual_servoing.pose_servo_controller import (
    PoseServoController,
)

from navigation.control.smooth_control_law import (
    Pose2D,
    SmoothControlParams,
)

class ApproachServoStrategy(Enum):
    RANGE_BEARING = "range_bearing"
    POSE_BEARING = "pose_bearing"

class ApproachTargetServo(Primitive):
    """
    Stage 2 perception-guided approach to one locked target.

    Responsibilities:
        - follow the already-selected target id
        - convert camera-relative geometry to gripper-relative geometry
        - keep the shared HeightModel updated
        - dynamically choose LOW/HIGH pickup commit distance
        - feed continuous observations to DistanceAngleController
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
    FAILURE_POSE_UNAVAILABLE = "pose_unavailable"

    POSE_FOV_ACTIVATION_MARGIN_DEG = 3.0

    def __init__(
            self,
            *,
            config,
            kind: str,
            height_model,
            locked_target_id: Optional[int],
            servo_method: ApproachServoMethod,
            pose_bearing_allowed: bool = False,
    ):
        super().__init__()

        self.config = config
        self.kind = kind
        self.height_model = height_model
        self.servo_method = servo_method

        self.pose_bearing_allowed = bool(
            pose_bearing_allowed
        )

        self.active_strategy = (
            ApproachServoStrategy.RANGE_BEARING
        )

        self.locked_target_id = (
            int(locked_target_id)
            if locked_target_id is not None
            else None
        )

        self.range_bearing_controller = DistanceAngleController(
            controller_id="approach_target",
            params=DistanceAngleParams(
                linear_kp=float(
                    self.config.approach_target_servo_linear_kp
                ),
                linear_ki=float(
                    self.config.approach_target_servo_linear_ki
                ),
                linear_kd=float(
                    self.config.approach_target_servo_linear_kd
                ),

                angular_kp=float(
                    self.config.approach_target_servo_angular_kp
                ),
                angular_ki=float(
                    self.config.approach_target_servo_angular_ki
                ),
                angular_kd=float(
                    self.config.approach_target_servo_angular_kd
                ),

                linear_max_mps=float(
                    self.config.approach_range_linear_max_mps
                ),
                angular_max_rad_s=float(
                    self.config.approach_range_angular_max_rad_s
                ),

                angle_drive_slowdown_start_rad=None,

                angle_drive_cutoff_rad=math.radians(
                    float(
                        self.config
                        .approach_target_servo_drive_cutoff_deg
                    )
                ),

                distance_tolerance_m=(
                        float(
                            self.config
                            .approach_target_servo_stop_tolerance_mm
                        )
                        / 1000.0
                ),

                derivative_mode=(
                    self.config
                    .approach_target_servo_derivative_mode
                ),
            ),
        )

        self.pose_bearing_controller = None
        self._pose_bearing_camera_name: Optional[str] = None
        self._pose_bearing_standoff_mm: Optional[float] = None

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
        self.range_bearing_controller.reset()

        if self.pose_bearing_controller is not None:
            self.pose_bearing_controller.reset()

        self.active_strategy = (
            ApproachServoStrategy.RANGE_BEARING
        )

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
            f"target_id={self.locked_target_id} "
            f"servo={self.servo_method.value} "
            f"pose_allowed={self.pose_bearing_allowed}"
        )

        return self.status

    def _get_locked_target(self, perception):
        if self.locked_target_id is None:
            return None

        memory = perception.objects.get(self.kind, {})

        return memory.get(self.locked_target_id)

    def _get_terminal_target_bearing_geometry(
            self,
            perception,
            *,
            commit_distance_mm: float,
            tolerance_mm: float,
    ):
        """
        Resolve duplicate observations of the locked target only when
        TARGET_BEARING is entering its final-pickup region.

        If multiple detections of the same tag ID are present in the
        current frame, choose the observation whose gripper-relative
        bearing is closest to zero.

        Outside the terminal region, normal target tracking is unchanged.
        """

        if (
                self.servo_method
                != ApproachServoMethod.TARGET_BEARING
        ):
            return None

        frame_memory = getattr(
            perception,
            "current_object_observations",
            None,
        )

        if frame_memory is None:
            return None

        candidates = (
            frame_memory
            .get(self.kind, {})
            .get(self.locked_target_id, [])
        )

        if len(candidates) < 2:
            return None

        geometries = []

        for observation in candidates:
            try:
                candidate = relative_target_from_gripper(
                    observation=observation,
                    config=self.config,
                )
            except (KeyError, TypeError, ValueError):
                continue

            geometries.append(
                candidate
            )

        if len(geometries) < 2:
            return None

        terminal_limit_m = (
                                   float(commit_distance_mm)
                                   + float(tolerance_mm)
                           ) / 1000.0

        # Do nothing until at least one duplicate observation says
        # we have reached the normal final-handoff region.
        if not any(
                geometry.distance_m <= terminal_limit_m
                for geometry in geometries
        ):
            return None

        selected = min(
            geometries,
            key=lambda geometry: abs(
                geometry.bearing_rad
            ),
        )

        print(
            "[SERVO_APPROACH][FINAL_SELECT] "
            f"id={self.locked_target_id} "
            f"candidates={len(geometries)} "
            f"selected_dist="
            f"{selected.distance_m * 1000.0:.0f}mm "
            f"selected_bearing="
            f"{-math.degrees(selected.bearing_rad):+.1f}deg"
        )

        return selected

    def _get_locked_pose_face(self, perception):
        """
        Return the best current pose-capable face observation
        for the locked target.

        Use the same face-selection rule validated by the
        pose-servo challenge: prefer the face with the smallest
        absolute pitch + roll.
        """

        if self.locked_target_id is None:
            return None

        object_faces = getattr(
            perception,
            "object_faces",
            None,
        )

        if object_faces is None:
            return None

        kind_faces = object_faces.get(
            self.kind,
            {},
        )

        faces = kind_faces.get(
            self.locked_target_id,
            [],
        )

        if not faces:
            return None

        return min(
            faces,
            key=lambda face: (
                abs(float(face["pitch_deg"]))
                + abs(float(face["roll_deg"]))
            ),
        )

    @staticmethod
    def _pose_face_is_usable(face) -> bool:
        """
        Return True when a face contains the minimum valid
        geometry required by Pose-Bearing control.

        This is deliberately only a validity gate for now.
        It does not yet impose empirical pose-quality limits.
        """

        if face is None:
            return False

        numeric_fields = (
            "distance",
            "bearing",
            "yaw_deg",
            "pitch_deg",
            "roll_deg",
        )

        try:
            for name in numeric_fields:
                value = float(face[name])

                if not math.isfinite(value):
                    return False

            camera_name = str(face["camera"]).strip()

        except (KeyError, TypeError, ValueError):
            return False

        return bool(camera_name)

    def _set_active_strategy(
            self,
            strategy: ApproachServoStrategy,
    ) -> None:
        """
        Change Stage-2 servo strategy cleanly.

        Controller state is reset on a strategy transition so
        PID history and PBVS/Smooth latch state do not leak
        across control methods.
        """

        if strategy == self.active_strategy:
            return

        previous = self.active_strategy

        self.range_bearing_controller.reset()

        if self.pose_bearing_controller is not None:
            self.pose_bearing_controller.reset()

        self.active_strategy = strategy

        print(
            "[SERVO_APPROACH][STRATEGY] "
            f"{previous.value} -> {strategy.value}"
        )

    def _ensure_pose_bearing_controller(
            self,
            *,
            face,
            standoff_mm: float,
    ) -> PoseServoController:
        """
        Create/configure the Pose-Bearing controller for the
        camera which produced the selected face observation.

        Rebuild if the camera changes.

        If the LOW/HIGH pickup decision changes the requested
        standoff distance, reset the controller so Smooth
        admissibility is re-evaluated for the new target pose.
        """

        camera_name = str(face["camera"])

        camera_calibration = CALIBRATION.cameras.get(
            camera_name
        )

        if camera_calibration is None:
            raise RuntimeError(
                f"No calibration for camera {camera_name!r}"
            )

        camera_fov_deg = float(
            camera_calibration.meta.fov_deg
        )

        if camera_fov_deg <= 0.0:
            raise RuntimeError(
                f"Camera {camera_name!r} has no usable FOV"
            )

        margin_rad = math.radians(
            self.POSE_FOV_ACTIVATION_MARGIN_DEG
        )

        camera_half_fov_rad = math.radians(
            camera_fov_deg / 2.0
        )

        safe_half_fov_rad = (
            camera_half_fov_rad
            - margin_rad
        )

        if safe_half_fov_rad <= 0.0:
            raise RuntimeError(
                f"Camera {camera_name!r} FOV is too small "
                f"for {self.POSE_FOV_ACTIVATION_MARGIN_DEG:.1f}deg margin"
            )

        rebuild = (
            self.pose_bearing_controller is None
            or self._pose_bearing_camera_name != camera_name
        )

        if rebuild:
            pbvs_linear_max_mps = float(
                self.config.approach_pose_pbvs_linear_max_mps
            )

            smooth_linear_max_mps = float(
                self.config.approach_pose_smooth_linear_max_mps
            )

            pose_angular_max_rps = float(
                self.config.approach_pose_angular_max_rad_s
            )

            self.pose_bearing_controller = PoseServoController(


                camera_half_fov_rad=camera_half_fov_rad,
                fov_activation_margin_rad=margin_rad,
                safe_half_fov_rad=safe_half_fov_rad,

                smooth_params=SmoothControlParams(
                    k_phi=3.0,
                    linear_max_mps=smooth_linear_max_mps,
                    angular_max_rps=pose_angular_max_rps,
                ),

                pbvs_linear_max_mps=pbvs_linear_max_mps,
                pbvs_angular_max_rps=pose_angular_max_rps,

                config=self.config,
            )

            self._pose_bearing_camera_name = camera_name
            self._pose_bearing_standoff_mm = float(
                standoff_mm
            )

            print(
                "[SERVO_APPROACH][POSE] controller created "
                f"camera={camera_name} "
                f"fov={camera_fov_deg:.1f}deg "
                f"safe_half="
                f"{math.degrees(safe_half_fov_rad):.1f}deg "
                f"standoff={standoff_mm:.0f}mm"
            )

        elif (
            self._pose_bearing_standoff_mm is None
            or abs(
                self._pose_bearing_standoff_mm
                - float(standoff_mm)
            ) > 1e-6
        ):

            self.pose_bearing_controller.reset()

            self._pose_bearing_standoff_mm = float(
                standoff_mm
            )

            print(
                "[SERVO_APPROACH][POSE] "
                f"standoff changed to {standoff_mm:.0f}mm "
                "-> controller reset"
            )

        return self.pose_bearing_controller

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

        # --------------------------------------------------
        # Common perception freshness policy
        # --------------------------------------------------

        age_s = now - timestamp

        visible_age_s = float(
            self.config.visible_max_age_s
        )

        fail_age_s = (
            visible_age_s
            + float(self.config.vision_loss_timeout_s)
        )

        if age_s > visible_age_s:
            self.velocity_backend.stop()

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

        target_geometry = relative_target_from_gripper(
            observation=target,
            config=self.config,
        )

        distance_m = target_geometry.distance_m
        bearing_rad = target_geometry.bearing_rad

        # Legacy values retained only for existing height,
        # handoff and diagnostic interfaces.
        distance_mm = (
                distance_m * 1000.0
        )

        bearing_deg = -math.degrees(
            bearing_rad
        )

        self._update_height_model(
            target=target,
            distance_mm=distance_mm,
            observation_timestamp=timestamp,
        )

        commit_distance_mm = self._commit_distance_mm()

        commit_tolerance_mm = float(
            self.config.approach_target_servo_stop_tolerance_mm
        )

        terminal_geometry = (
            self._get_terminal_target_bearing_geometry(
                perception,
                commit_distance_mm=commit_distance_mm,
                tolerance_mm=commit_tolerance_mm,
            )
        )

        if terminal_geometry is not None:
            distance_m = (
                terminal_geometry.distance_m
            )

            bearing_rad = (
                terminal_geometry.bearing_rad
            )

            distance_mm = (
                    distance_m * 1000.0
            )

            bearing_deg = -math.degrees(
                bearing_rad
            )

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

        # --------------------------------------------------
        # Final-pickup handoff
        # --------------------------------------------------
        #
        # ApproachTargetServo owns terminal arrival.
        #
        # Preserve the existing Stage 2 stopping semantics:
        #
        #     remaining <= configured stop tolerance
        #
        # For example:
        #     commit = 300 mm
        #     tolerance = 10 mm
        #     handoff at <= 310 mm
        #
        # FinalPickup then owns:
        #     final alignment -> blind drive -> grasp.


        remaining_distance_mm = (
            distance_mm - commit_distance_mm
        )

        if (
            self.height_model.is_committed()
            and remaining_distance_mm <= commit_tolerance_mm
        ):
            self.velocity_backend.stop()

            self.final_distance_mm = float(distance_mm)
            self.final_bearing_deg = float(bearing_deg)

            latest_height = (
                self.height_model.last_good_is_high()
            )

            self.target_is_high = (
                latest_height
                if latest_height is not None
                else self.height_model.is_high()
            )

            print(
                "[SERVO_APPROACH][HANDOFF] "
                f"approach_mode="
                f"{'HIGH' if self.height_model.is_high() else 'LOW'} "
                f"pickup_height="
                f"{'HIGH' if self.target_is_high else 'LOW'} "
                f"dist={distance_mm:.0f}mm "
                f"bearing={bearing_deg:.1f}deg "
                f"commit={commit_distance_mm:.0f}mm "
                f"tolerance={commit_tolerance_mm:.0f}mm "
                "-> FINAL_PICKUP"
            )

            return PrimitiveStatus.SUCCEEDED

        # --------------------------------------------------
        # Select Stage-2 servo strategy
        # --------------------------------------------------

        pose_face = None

        if (
                self.servo_method
                == ApproachServoMethod.TARGET_BEARING
        ):
            desired_strategy = (
                ApproachServoStrategy.RANGE_BEARING
            )

        elif (
                self.servo_method
                == ApproachServoMethod.POSE_BEARING
        ):

            if not self.pose_bearing_allowed:
                print(
                    "[SERVO_APPROACH] "
                    "POSE_BEARING requested but "
                    "pose bearing is not available"
                )

                self.velocity_backend.stop()
                self.failure_reason = (
                    self.FAILURE_POSE_UNAVAILABLE
                )
                return PrimitiveStatus.FAILED

            candidate_pose_face = (
                self._get_locked_pose_face(
                    perception
                )
            )

            if not self._pose_face_is_usable(
                    candidate_pose_face
            ):
                print(
                    "[SERVO_APPROACH] "
                    "POSE_BEARING requested but "
                    "no usable pose face is available"
                )

                self.velocity_backend.stop()
                self.failure_reason = (
                    self.FAILURE_POSE_UNAVAILABLE
                )
                return PrimitiveStatus.FAILED

            pose_face = candidate_pose_face

            desired_strategy = (
                ApproachServoStrategy.POSE_BEARING
            )

        else:
            raise RuntimeError(
                "Unsupported selected servo method: "
                f"{self.servo_method!r}"
            )

        self._set_active_strategy(
            desired_strategy
        )


        # --------------------------------------------------
        # Servo strategy routing
        # --------------------------------------------------

        active_strategy = self.active_strategy.value
        pose_mode = None

        if self.active_strategy == ApproachServoStrategy.RANGE_BEARING:

            result = self.range_bearing_controller.update(
                distance_m=distance_m,
                target_distance_m=(
                        float(commit_distance_mm)
                        / 1000.0
                ),
                angle_rad=bearing_rad,
                target_angle_rad=0.0,
                timestamp=timestamp,
            )

            command = VelocityCommand(
                linear_x_mps=result.linear_mps,
                angular_z_rps=result.angular_rps,
                lateral_y_mps=0.0,
                timestamp=timestamp,
            )

        elif self.active_strategy == ApproachServoStrategy.POSE_BEARING:

            face = pose_face

            pose_controller = (
                self._ensure_pose_bearing_controller(
                    face=face,
                    standoff_mm=commit_distance_mm,
                )
            )

            pose_timestamp = float(
                face.get(
                    "last_seen",
                    timestamp,
                )
            )

            # --------------------------------------------------
            # Construct desired relative pose
            # --------------------------------------------------
            #
            # Position reference:
            #     centre of the locked object
            #
            # Orientation reference:
            #     normal to the selected pose-capable face
            #
            # The object centre is therefore propagated outward
            # along the selected face normal by the requested
            # standoff distance.

            reference_target = (
                relative_target_from_base_link(
                    observation=target,
                    config=self.config,
                )
            )

            camera_name = str(face["camera"])

            camera_mount = self.config.camera_mounts[
                camera_name
            ]

            camera_yaw_rad = float(
                camera_mount.get(
                    "yaw_rad",
                    0.0,
                )
            )

            face_yaw_rad = math.radians(
                float(face["yaw_deg"])
            )

            # Same face-heading convention already validated by
            # the previous pose geometry:
            #
            #     desired heading = camera yaw - face yaw
            #
            # This points the controlled frame square toward the
            # selected object face.
            controlled_heading_rad = (
                    camera_yaw_rad
                    - face_yaw_rad
            )

            gripper_mount = self.config.gripper_mount

            (
                goal_x_m,
                goal_y_m,
                goal_heading_rad,
            ) = relative_goal_pose_from_reference(
                reference_x_m=(
                    reference_target.x_m
                ),
                reference_y_m=(
                    reference_target.y_m
                ),
                controlled_heading_rad=(
                    controlled_heading_rad
                ),
                standoff_m=(
                        float(commit_distance_mm)
                        / 1000.0
                ),
                controlled_frame_x_m=float(
                    gripper_mount["x_m"]
                ),
                controlled_frame_y_m=float(
                    gripper_mount["y_m"]
                ),
                controlled_frame_yaw_rad=float(
                    gripper_mount.get(
                        "yaw_rad",
                        0.0,
                    )
                ),
            )

            target_pose = Pose2D(
                x_m=goal_x_m,
                y_m=goal_y_m,
                heading_rad=goal_heading_rad,
            )

            print(
                "[SERVO_APPROACH][POSE_GOAL] "
                f"face_dist={float(face['distance']):.0f}mm "
                f"face_bearing={float(face['bearing']):+.2f}deg "
                f"face_yaw={float(face['yaw_deg']):+.2f}deg "
                f"goal_x={goal_x_m * 1000.0:+.1f}mm "
                f"goal_y={goal_y_m * 1000.0:+.1f}mm "
                f"goal_heading="
                f"{math.degrees(goal_heading_rad):+.2f}deg "
                f"goal_r="
                f"{math.hypot(goal_x_m, goal_y_m) * 1000.0:.1f}mm"
            )

            pose_result = pose_controller.update(
                tag_id=int(self.locked_target_id),
                observation=face,
                target_pose=target_pose,
                timestamp=pose_timestamp,
            )

            command = pose_result.command
            pose_mode = pose_result.mode.value

        else:
            raise RuntimeError(
                f"Unsupported approach servo strategy: "
                f"{self.active_strategy!r}"
            )

        self.velocity_backend.update(command)

        mode_text = (
            f" mode={pose_mode}"
            if pose_mode is not None
            else ""
        )

        print(
            "[SERVO_APPROACH] "
            f"strategy={active_strategy}"
            f"{mode_text} "
            f"dist={distance_mm:.0f}mm "
            f"bearing={bearing_deg:+.1f}deg "
            f"commit={commit_distance_mm:.0f}mm "
            f"vx={command.linear_x_mps:.3f}m/s "
            f"wz={command.angular_z_rps:+.3f}rad/s"
        )

        return PrimitiveStatus.RUNNING

    def stop(self, **_):
        if self.velocity_backend is not None:
            self.velocity_backend.stop()