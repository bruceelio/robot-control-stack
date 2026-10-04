# skills/navigation/approach_feasibility.py

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from calibration import CALIBRATION

from navigation.control.smooth_control_law import (
    Pose2D,
    SmoothControlLaw,
    SmoothControlParams,
)

from navigation.control.smooth_pose_selector import (
    SmoothPoseSelection,
    select_smooth_pose,
)

from navigation.geometry.relative_goal_pose import (
    relative_goal_pose_from_reference,
)

from perception.robot_geometry import (
    relative_target_from_base_link,
)

from perception.providers.acquisition import (
    WallGeometryStatus,
    get_wall_geometry_status,
)

from vision.apriltag.visibility_geometry import (
    horizontal_tag_visibility,
)


class ApproachFeasibilityMode(Enum):
    """
    Result of testing one already-ranked target candidate.
    """

    SMOOTH = "smooth"
    RANGE_BEARING = "range_bearing"
    UNAVAILABLE = "unavailable"


@dataclass(frozen=True)
class ApproachFeasibilityResult:
    """
    Feasibility result for one target candidate.

    This does not choose between targets.

    SelectTarget owns:
        - hard filtering
        - preference ordering
        - nearest-first ordering
        - trying the next candidate

    This object answers only:

        "How, if at all, can this candidate currently
         be approached?"
    """

    mode: ApproachFeasibilityMode
    reason: str | None

    pose_face: dict | None = None
    target_pose: Pose2D | None = None
    smooth_selection: SmoothPoseSelection | None = None
    wall_geometry_status: WallGeometryStatus | None = None

    @property
    def viable(self) -> bool:
        return (
            self.mode
            != ApproachFeasibilityMode.UNAVAILABLE
        )


class ApproachFeasibilityEvaluator:
    """
    Evaluate Stage-2 approach feasibility for one target.

    Current policy
    --------------

    LOW target:

        usable pose face
            -> try Smooth exact / relaxed

        Smooth succeeds
            -> SMOOTH

        no usable pose face OR Smooth fails
            -> RANGE_BEARING

        HIGH target:

        preferred wall-aware / pose-aware geometry
            -> try preferred controller when available

        preferred controller succeeds
            -> use it

        wall / pose geometry unavailable OR preferred controller fails
            -> visual fallback

        only an explicit physical-reachability failure may reject
        the target

    UNKNOWN elevation:

        -> UNAVAILABLE

    HIGH deliberately remains viable when preferred geometry is
    unavailable. Wall geometry will later be allowed to reject a HIGH
    target only when it positively establishes that the target lies
    outside the gripper's reachable region.

    SelectTarget may then try another candidate.

    This class deliberately does NOT decide which object is preferable.
    """

    def __init__(
        self,
        *,
        config,
        pose_bearing_allowed: bool,
        calibration=CALIBRATION,
        fov_margin_deg: float = 3.0,
    ):
        self.config = config

        self.calibration = calibration

        self.pose_bearing_allowed = bool(
            pose_bearing_allowed
        )

        self.fov_margin_rad = math.radians(
            float(fov_margin_deg)
        )

        # Keep this identical to the Smooth controller used by
        # ApproachTargetServo so feasibility and execution use the
        # same controller parameters.
        self.smooth = SmoothControlLaw(
            SmoothControlParams(
                k_phi=3.0,

                linear_max_mps=float(
                    self.config
                    .approach_pose_smooth_linear_max_mps
                ),

                angular_max_rps=float(
                    self.config
                    .approach_pose_angular_max_rad_s
                ),
            )
        )

    def evaluate(
        self,
        *,
        kind: str,
        target: dict,
        elevation: str,
        perception,
        wall_geometry=None,
    ) -> ApproachFeasibilityResult:

        elevation = str(
            elevation
        ).lower()

        # --------------------------------------------------
        # Require a definite LOW/HIGH classification
        # --------------------------------------------------

        if elevation not in (
            "high",
            "low",
        ):
            return ApproachFeasibilityResult(
                mode=(
                    ApproachFeasibilityMode.UNAVAILABLE
                ),
                reason="elevation_unknown",
            )

        # --------------------------------------------------
        # HIGH wall-relative geometry
        # --------------------------------------------------
        #
        # HIGH pickup geometry is defined by the supporting wall,
        # not by the orientation of an observed cube face.
        #
        # Ask the perception acquisition layer whether the required
        # wall geometry:
        #
        #     - already exists;
        #     - can be acquired;
        #     - is unavailable.
        #
        # Active acquisition is not executed here. This evaluator
        # only reports feasibility and carries the wall status
        # forward for the approach behaviour.
        #
        # Until wall-relative pose construction is connected, HIGH
        # remains viable through the proven visual range/bearing
        # fallback in every case except a future explicit physical
        # reachability failure.
        #
        # Do not allow HIGH to fall through to the cube-face-derived
        # LOW pose path below.

        if elevation == "high":

            wall_status = get_wall_geometry_status(
                config=self.config,
                estimated_geometry=wall_geometry,
                require_heading=True,
                require_distance=True,
            )

            if wall_status.available:
                return ApproachFeasibilityResult(
                    mode=(
                        ApproachFeasibilityMode
                        .RANGE_BEARING
                    ),
                    reason=(
                        "wall_geometry_available_"
                        "high_pending_wall_pose"
                    ),
                    wall_geometry_status=wall_status,
                )

            if wall_status.acquisition_required:
                return ApproachFeasibilityResult(
                    mode=(
                        ApproachFeasibilityMode
                        .RANGE_BEARING
                    ),
                    reason=(
                        "wall_geometry_acquirable_"
                        "high_pending_acquisition"
                    ),
                    wall_geometry_status=wall_status,
                )

            return ApproachFeasibilityResult(
                mode=(
                    ApproachFeasibilityMode
                    .RANGE_BEARING
                ),
                reason=(
                    "wall_geometry_unavailable_"
                    "high_visual_fallback"
                ),
                wall_geometry_status=wall_status,
            )

        # --------------------------------------------------
        # LOW pose geometry
        # --------------------------------------------------

        pose_face = self._get_pose_face(
            perception=perception,
            kind=kind,
            target=target,
        )

        pose_available = (
            self.pose_bearing_allowed
            and self._pose_face_is_usable(
                pose_face
            )
        )

        # --------------------------------------------------
        # No pose geometry
        # --------------------------------------------------

        if not pose_available:

            # LOW retains the simple range/bearing visual-servo
            # fallback.
            if elevation == "low":

                return ApproachFeasibilityResult(
                    mode=(
                        ApproachFeasibilityMode
                        .RANGE_BEARING
                    ),
                    reason=(
                        "pose_unavailable_low_fallback"
                    ),
                )

            # HIGH remains viable even when preferred pose geometry
            # is unavailable. The continuous visual fallback is the
            # baseline approach until wall-aware geometry is available.
            return ApproachFeasibilityResult(
                mode=(
                    ApproachFeasibilityMode
                    .RANGE_BEARING
                ),
                reason=(
                    "pose_unavailable_high_"
                    "visual_fallback"
                ),
            )

        # --------------------------------------------------
        # Pose exists: evaluate Smooth
        # --------------------------------------------------

        try:
            (
                target_pose,
                smooth_selection,
            ) = self._select_smooth_pose(
                target=target,
                pose_face=pose_face,
                elevation=elevation,
            )

        except (
            KeyError,
            TypeError,
            ValueError,
            RuntimeError,
        ) as exc:

            # Geometry failure is equivalent to Smooth being
            # unavailable for the current observation.

            if elevation == "low":

                return ApproachFeasibilityResult(
                    mode=(
                        ApproachFeasibilityMode
                        .RANGE_BEARING
                    ),
                    reason=(
                        "smooth_geometry_failed_"
                        "low_fallback:"
                        f"{type(exc).__name__}"
                    ),
                    pose_face=pose_face,
                )

            return ApproachFeasibilityResult(
                mode=(
                    ApproachFeasibilityMode
                    .RANGE_BEARING
                ),
                reason=(
                    "smooth_geometry_failed_high_"
                    "visual_fallback:"
                    f"{type(exc).__name__}"
                ),
                pose_face=pose_face,
            )

        # --------------------------------------------------
        # Smooth exact / relaxed succeeded
        # --------------------------------------------------

        if smooth_selection.admissible:

            return ApproachFeasibilityResult(
                mode=ApproachFeasibilityMode.SMOOTH,
                reason=None,
                pose_face=pose_face,
                target_pose=target_pose,
                smooth_selection=smooth_selection,
            )

        # --------------------------------------------------
        # Smooth failed all candidates
        # --------------------------------------------------

        if elevation == "low":

            return ApproachFeasibilityResult(
                mode=(
                    ApproachFeasibilityMode
                    .RANGE_BEARING
                ),
                reason=(
                    "smooth_unavailable_low_fallback"
                ),
                pose_face=pose_face,
                target_pose=target_pose,
                smooth_selection=smooth_selection,
            )

        return ApproachFeasibilityResult(
            mode=(
                ApproachFeasibilityMode
                .RANGE_BEARING
            ),
            reason=(
                "smooth_unavailable_high_"
                "visual_fallback"
            ),
            pose_face=pose_face,
            target_pose=target_pose,
            smooth_selection=smooth_selection,
        )

    # ==================================================
    # Pose-face selection
    # ==================================================

    def _get_pose_face(
        self,
        *,
        perception,
        kind: str,
        target: dict,
    ):
        """
        Return the best usable current face for this target.

        Match the rule already used by ApproachTargetServo:
        smallest absolute pitch + roll.
        """

        target_id = target.get(
            "id"
        )

        if target_id is None:
            return None

        try:
            target_id = int(
                target_id
            )

        except (
            TypeError,
            ValueError,
        ):
            return None

        object_faces = getattr(
            perception,
            "object_faces",
            None,
        )

        if object_faces is None:
            return None

        faces = (
            object_faces
            .get(kind, {})
            .get(target_id, [])
        )

        usable_faces = [
            face
            for face in faces
            if self._pose_face_is_usable(
                face
            )
        ]

        if not usable_faces:
            return None

        return min(
            usable_faces,
            key=lambda face: (
                abs(
                    float(
                        face["pitch_deg"]
                    )
                )
                + abs(
                    float(
                        face["roll_deg"]
                    )
                )
            ),
        )

    @staticmethod
    def _pose_face_is_usable(
        face,
    ) -> bool:

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

                value = float(
                    face[name]
                )

                if not math.isfinite(
                    value
                ):
                    return False

            camera_name = str(
                face["camera"]
            ).strip()

        except (
            KeyError,
            TypeError,
            ValueError,
        ):
            return False

        return bool(
            camera_name
        )

    # ==================================================
    # Smooth feasibility
    # ==================================================

    def _select_smooth_pose(
        self,
        *,
        target: dict,
        pose_face: dict,
        elevation: str,
    ) -> tuple[
        Pose2D,
        SmoothPoseSelection,
    ]:

        target_id = int(
            target["id"]
        )

        camera_name = str(
            pose_face["camera"]
        )

        # --------------------------------------------------
        # Camera FOV
        # --------------------------------------------------

        camera_calibration = (
            self.calibration.cameras.get(
                camera_name
            )
        )

        if camera_calibration is None:

            raise RuntimeError(
                f"No calibration for camera "
                f"{camera_name!r}"
            )

        camera_fov_deg = float(
            camera_calibration.meta.fov_deg
        )

        if camera_fov_deg <= 0.0:

            raise RuntimeError(
                f"Camera {camera_name!r} "
                "has no usable FOV"
            )

        camera_half_fov_rad = math.radians(
            camera_fov_deg / 2.0
        )

        safe_half_fov_rad = (
            camera_half_fov_rad
            - self.fov_margin_rad
        )

        if safe_half_fov_rad <= 0.0:

            raise RuntimeError(
                f"Camera {camera_name!r} "
                "FOV is too small"
            )

        # --------------------------------------------------
        # Target reference geometry
        # --------------------------------------------------

        # Use one internally consistent pose observation for all
        # Smooth geometry:
        #
        #     target position
        #     face orientation
        #     tag visibility edges
        #
        # Mixing the aggregate tracked target with a different selected
        # pose face can place the terminal camera significantly sideways
        # from the tag and produce a false FOV restriction.
        # Keep execution geometry identical to the feasibility
        # geometry used by SelectTarget. Position, face heading
        # and tag visibility must all come from the same selected
        # pose-capable face.
        reference_target = (
            relative_target_from_base_link(
                observation=pose_face,
                config=self.config,
            )
        )

        camera_mount_config = (
            self.config.camera_mounts[
                camera_name
            ]
        )

        camera_yaw_rad = float(
            camera_mount_config.get(
                "yaw_rad",
                0.0,
            )
        )

        face_yaw_rad = math.radians(
            float(
                pose_face["yaw_deg"]
            )
        )

        # Same convention already used by ApproachTargetServo.
        controlled_heading_rad = (
            camera_yaw_rad
            - face_yaw_rad
        )

        # --------------------------------------------------
        # LOW / HIGH standoff
        # --------------------------------------------------

        if elevation == "high":

            standoff_mm = float(
                self.config
                .final_commit_distance_high_mm
            )

        else:

            standoff_mm = float(
                self.config
                .final_commit_distance_mm
            )

        gripper_mount = (
            self.config.gripper_mount
        )

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
                standoff_mm / 1000.0
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

        # --------------------------------------------------
        # Full horizontal target visibility geometry
        # --------------------------------------------------

        tag_visibility = (
            horizontal_tag_visibility(
                tag_id=target_id,
                observation=pose_face,
                config=self.config,
            )
        )

        camera_mount = Pose2D(
            x_m=float(
                camera_mount_config["x_m"]
            ),
            y_m=float(
                camera_mount_config["y_m"]
            ),
            heading_rad=float(
                camera_mount_config.get(
                    "yaw_rad",
                    0.0,
                )
            ),
        )

        # --------------------------------------------------
        # 0 / 50 / 75 / 100 Smooth selector
        # --------------------------------------------------

        smooth_selection = (
            select_smooth_pose(
                law=self.smooth,

                ideal_goal_pose=(
                    target_pose
                ),

                visibility_points=(
                    (
                        tag_visibility
                        .left.base_x_m,

                        tag_visibility
                        .left.base_y_m,
                    ),
                    (
                        tag_visibility
                        .right.base_x_m,

                        tag_visibility
                        .right.base_y_m,
                    ),
                ),

                safe_half_fov_rad=(
                    safe_half_fov_rad
                ),

                camera_mount=camera_mount,
            )
        )

        return (
            target_pose,
            smooth_selection,
        )


__all__ = [
    "ApproachFeasibilityEvaluator",
    "ApproachFeasibilityMode",
    "ApproachFeasibilityResult",
]