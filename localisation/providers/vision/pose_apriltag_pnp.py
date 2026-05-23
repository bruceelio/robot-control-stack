# localisation/providers/vision/pose_apriltag_pnp.py

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class AprilTagPnPPoseResult:
    source_id: str
    pose_x_m: float | None
    pose_y_m: float | None
    pose_theta_rad: float | None
    ambiguity_score: float | None
    reprojection_score: float | None
    tag_count: int
    timestamp_ms: int
    valid: bool


class AprilTagPnPPoseProvider:
    """
    Generic AprilTag PnP pose provider.

    Consumes apriltag observations from any configured vision source:
        vision1, vision2, etc.

    Does not care whether the physical camera is named:
        front, rear, front_left, etc.
    """

    def estimate(
        self,
        *,
        source_id: str,
        apriltag_observations: list[dict],
        intrinsic_matrix: Any,
        distortion_coefficients: Any,
        camera_to_robot_transform: Any,
    ) -> AprilTagPnPPoseResult:

        tag_count = len(apriltag_observations)

        timestamp_ms = 0
        if apriltag_observations:
            timestamp_ms = int(float(apriltag_observations[0].get("timestamp", 0.0)) * 1000)

        if tag_count == 0:
            return AprilTagPnPPoseResult(
                source_id=source_id,
                pose_x_m=None,
                pose_y_m=None,
                pose_theta_rad=None,
                ambiguity_score=None,
                reprojection_score=None,
                tag_count=0,
                timestamp_ms=timestamp_ms,
                valid=False,
            )

        # TODO:
        # 1. Convert tag observations into object points and image points.
        # 2. Run cv2.solvePnP / solvePnPGeneric.
        # 3. Transform camera pose into robot/base_link pose.
        # 4. Score ambiguity and reprojection error.
        # 5. Return valid=True only if pose passes quality gates.

        return AprilTagPnPPoseResult(
            source_id=source_id,
            pose_x_m=None,
            pose_y_m=None,
            pose_theta_rad=None,
            ambiguity_score=None,
            reprojection_score=None,
            tag_count=tag_count,
            timestamp_ms=timestamp_ms,
            valid=False,
        )