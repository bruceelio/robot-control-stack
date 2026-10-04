# navigation/visual_servoing/pose_servo_controller.py

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import math
import time

from config import CONFIG
from navigation.command.velocity_arbiter import VelocityCommand

from navigation.control.smooth_control_law import (
    Pose2D,
    SmoothControlLaw,
    SmoothControlParams,
)

from navigation.control.smooth_fov_admissibility import (
    SmoothFovPrediction,
)

from navigation.control.smooth_pose_selector import (
    SmoothPoseSelection,
    select_smooth_pose,
)

from navigation.visual_servoing.pbvs_nonholonomic import (
    NonHolonomicPBVS,
)
from navigation.visual_servoing.pbvs_fov_visibility_constraint import (
    PBVSFovVisibilityConstraint,
    PBVSFovVisibilityConstraintParams,
)
from navigation.visual_servoing.servoing_types import (
    NonHolonomicVelocity2D,
    Pose2D as PBVSPose2D,
)

from vision.apriltag.visibility_geometry import (
    horizontal_tag_visibility,
)

class PoseServoMode(Enum):
    PBVS = "pbvs"
    SMOOTH = "smooth"


@dataclass(frozen=True)
class PoseServoResult:
    command: VelocityCommand
    mode: PoseServoMode
    mode_changed: bool

    smooth_selection: SmoothPoseSelection | None = None
    fov_prediction: SmoothFovPrediction | None = None

    pbvs_fov_result: object | None = None
    pbvs_result: object | None = None
    smooth_result: object | None = None

class PoseServoController:
    """
    Perception-only pose-servo supervisor.

    The controller operates entirely from the current relative target
    observation. It does not require robot localisation.

    Selection policy used by this first implementation:

        1. Predict the intended SmoothControlLaw trajectory.
        2. If that trajectory is not FOV-admissible, use PBVS to
        control the relative pose.
        3. Re-evaluate Smooth admissibility on each PBVS observation.
        4. Once Smooth becomes admissible, hand over and latch Smooth
           for the remainder of this visual_servoing operation.

    Target identity, face selection, observation freshness, target-loss
    handling and terminal arrival remain with the caller for now.
    """

    def __init__(
        self,
        *,
        safe_half_fov_rad: float,

        camera_half_fov_rad: float,
        fov_activation_margin_rad: float,


        smooth_params: SmoothControlParams,

        pbvs_linear_max_mps: float = 0.90,
        pbvs_angular_max_rps: float = 0.35,
        prediction_dt_s: float = 0.10,
        prediction_max_time_s: float = 5.0,
        prediction_position_tolerance_m: float = 0.01,
        config=CONFIG,
    ):


        if pbvs_linear_max_mps <= 0.0:
            raise ValueError("pbvs_linear_max_mps must be > 0")

        if pbvs_angular_max_rps <= 0.0:
            raise ValueError("pbvs_angular_max_rps must be > 0")

        self.config = config

        self.safe_half_fov_rad = float(safe_half_fov_rad)
        self.camera_half_fov_rad = float(
            camera_half_fov_rad
        )
        self.fov_activation_margin_rad = float(
            fov_activation_margin_rad
        )

        self.pbvs_linear_max_mps = float(pbvs_linear_max_mps)
        self.pbvs_angular_max_rps = float(pbvs_angular_max_rps)
        self.prediction_dt_s = float(prediction_dt_s)
        self.prediction_max_time_s = float(prediction_max_time_s)
        self.prediction_position_tolerance_m = float(
            prediction_position_tolerance_m
        )


        self.pbvs_fov = PBVSFovVisibilityConstraint(
            PBVSFovVisibilityConstraintParams(
                half_fov_rad=(
                    self.camera_half_fov_rad
                ),
                activation_margin_rad=(
                    self.fov_activation_margin_rad
                ),
                linear_scale_mps=(
                    self.pbvs_linear_max_mps
                ),
                angular_scale_rps=(
                    self.pbvs_angular_max_rps
                ),
            )
        )

        self.smooth = SmoothControlLaw(smooth_params)
        self.pbvs = NonHolonomicPBVS()

        self._mode: PoseServoMode | None = None

        # Smooth mode may be latched, but the relative target pose must
        # remain live because it is expressed in the robot's CURRENT
        # base_link frame.
        #
        # Only the selected terminal-heading relaxation is persistent.
        self._smooth_heading_offset_rad: float | None = None

    @property
    def mode(self) -> PoseServoMode | None:
        return self._mode

    def reset(self) -> None:
        self._mode = None
        self._smooth_heading_offset_rad = None
        self.pbvs.reset()

    def update(
            self,
            *,
            tag_id: int,
            observation: dict,
            target_pose: Pose2D,
            timestamp: float | None = None,
    ) -> PoseServoResult:

        if timestamp is None:
            timestamp = time.time()

        tag_visibility = horizontal_tag_visibility(
            tag_id=tag_id,
            observation=observation,
            config=self.config,
        )

        previous_mode = self._mode
        fov_prediction = None
        pbvs_result = None
        smooth_result = None
        pbvs_fov_result = None
        smooth_selection = None

        # --------------------------------------------------
        # Select Smooth terminal pose
        # --------------------------------------------------
        #
        # Do not force Smooth.
        #
        # The selector first tests the exact requested pose. If that
        # trajectory is not visibility-admissible, it tests the
        # configured relaxed terminal-heading candidates:
        #
        #     50% -> 75% -> 100%
        #
        # The staging x/y position remains unchanged.
        #
        # Once an admissible Smooth strategy is selected, latch Smooth
        # mode and the chosen terminal-heading relaxation.
        #
        # Do NOT latch the relative target pose itself. target_pose is
        # expressed in the CURRENT base_link frame and therefore must be
        # refreshed from perception on every control cycle.

        if self._mode != PoseServoMode.SMOOTH:

            camera_name = str(
                observation["camera"]
            )

            camera_config = (
                self.config.camera_mounts[
                    camera_name
                ]
            )

            camera_mount = Pose2D(
                x_m=float(
                    camera_config["x_m"]
                ),
                y_m=float(
                    camera_config["y_m"]
                ),
                heading_rad=float(
                    camera_config.get(
                        "yaw_rad",
                        0.0,
                    )
                ),
            )

            smooth_selection = select_smooth_pose(
                law=self.smooth,
                ideal_goal_pose=target_pose,
                visibility_points=(
                    (
                        tag_visibility.left.base_x_m,
                        tag_visibility.left.base_y_m,
                    ),
                    (
                        tag_visibility.right.base_x_m,
                        tag_visibility.right.base_y_m,
                    ),
                ),
                safe_half_fov_rad=(
                    self.safe_half_fov_rad
                ),
                dt_s=self.prediction_dt_s,
                max_time_s=(
                    self.prediction_max_time_s
                ),
                position_tolerance_m=(
                    self.prediction_position_tolerance_m
                ),
                camera_mount=camera_mount,
            )

            if smooth_selection.admissible:
                self._mode = PoseServoMode.SMOOTH

                self._smooth_heading_offset_rad = float(
                    smooth_selection.heading_offset_rad
                )

                fov_prediction = (
                    smooth_selection.prediction
                )

            else:
                self._mode = PoseServoMode.PBVS
                self._smooth_heading_offset_rad = None

        if self._mode == PoseServoMode.SMOOTH:

            if self._smooth_heading_offset_rad is None:
                raise RuntimeError(
                    "Smooth mode selected without a heading offset"
                )

            # target_pose is freshly reconstructed by the caller from the
            # current camera observation on every control cycle.
            #
            # Preserve the Smooth selector's chosen terminal-heading
            # relaxation, but apply it to the CURRENT relative goal.
            live_heading_rad = (
                    target_pose.heading_rad
                    + self._smooth_heading_offset_rad
            )

            live_heading_rad = math.atan2(
                math.sin(live_heading_rad),
                math.cos(live_heading_rad),
            )

            live_smooth_target_pose = Pose2D(
                x_m=target_pose.x_m,
                y_m=target_pose.y_m,
                heading_rad=live_heading_rad,
            )

            smooth_result = self.smooth.calculate_regular_velocity(
                live_smooth_target_pose
            )

            command = VelocityCommand(
                linear_x_mps=(
                    smooth_result.command.linear_x_mps
                ),
                angular_z_rps=(
                    smooth_result.command.angular_z_rps
                ),
                lateral_y_mps=0.0,
                timestamp=float(timestamp),
            )



        else:

            pbvs_target_pose = PBVSPose2D(
                x_m=target_pose.x_m,
                y_m=target_pose.y_m,
                heading_rad=target_pose.heading_rad,
            )

            pbvs_result = self.pbvs.calculate(

                current=PBVSPose2D(

                    x_m=0.0,

                    y_m=0.0,

                    heading_rad=0.0,

                ),

                desired=pbvs_target_pose,

            )

            linear_mps = pbvs_result.velocity.linear_mps

            angular_rps = pbvs_result.velocity.angular_rps

            # Scale v and omega together.

            #

            # This preserves the trajectory curvature omega / v,

            # unlike independent velocity clipping.

            scale = max(

                1.0,

                abs(linear_mps) / self.pbvs_linear_max_mps,

                abs(angular_rps) / self.pbvs_angular_max_rps,

            )

            linear_mps /= scale

            angular_rps /= scale

            visibility_feature = (
                tag_visibility.most_endangered
            )

            pbvs_fov_result = self.pbvs_fov.calculate(
                feature_bearing_rad=(
                    visibility_feature.bearing_rad
                ),
                feature_range_m=(
                    visibility_feature.range_m
                ),
                nominal_velocity=NonHolonomicVelocity2D(
                    linear_mps=linear_mps,
                    angular_rps=angular_rps,
                ),
            )

            linear_mps = (
                pbvs_fov_result.velocity.linear_mps
            )

            angular_rps = (
                pbvs_fov_result.velocity.angular_rps
            )

            command = VelocityCommand(

                linear_x_mps=linear_mps,

                angular_z_rps=angular_rps,

                lateral_y_mps=0.0,

                timestamp=float(timestamp),

            )

        return PoseServoResult(
            command=command,
            mode=self._mode,
            mode_changed=(self._mode != previous_mode),
            smooth_selection=smooth_selection,
            fov_prediction=fov_prediction,
            pbvs_fov_result=pbvs_fov_result,
            pbvs_result=pbvs_result,
            smooth_result=smooth_result,
        )
