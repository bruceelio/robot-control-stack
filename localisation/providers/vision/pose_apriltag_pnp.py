# localisation/providers/vision/pose_apriltag_pnp.py

from __future__ import annotations

from dataclasses import dataclass
from typing import Any
from config import CONFIG
from config.arena import marker_poses

import numpy as np
import cv2

def build_pnp_points(
    usable_observations: list[tuple[dict, dict]],
) -> tuple[np.ndarray, np.ndarray]:
    object_points = []
    image_points = []

    for obs, field_pose in usable_observations:
        corners_px = obs.get("corners_px")
        tag_size_m = obs.get("tag_size_m")

        if not corners_px or tag_size_m is None:
            continue

        cx = float(field_pose["x_m"])
        cy = float(field_pose["y_m"])
        cz = float(field_pose.get("z_m", 0.0))

        half = float(tag_size_m) / 2.0

        yaw = float(field_pose.get("yaw_rad", 0.0))

        # Wall tag model:
        # - u moves horizontally along the wall
        # - v moves vertically up/down
        normal_x = np.cos(yaw)
        normal_y = np.sin(yaw)

        tangent_x = -normal_y
        tangent_y = normal_x

        local_corners = [
            (+half, -half),
            (+half, +half),
            (-half, +half),
            (-half, -half),
        ]

        tag_object_points = []

        for u, v in local_corners:
            tag_object_points.append([
                cx + (tangent_x * u),
                cy + (tangent_y * u),
                cz + v,
            ])

        object_points.extend(tag_object_points)
        image_points.extend(corners_px)

    return (
        np.array(object_points, dtype=np.float64),
        np.array(image_points, dtype=np.float64),
    )

def filter_observations_by_tag_ids(
    usable_observations: list[tuple[dict, dict]],
    wanted_tag_ids: set[int],
) -> list[tuple[dict, dict]]:
    return [
        (obs, field_pose)
        for obs, field_pose in usable_observations
        if int(obs["tag_id"]) in wanted_tag_ids
    ]


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

        usable_observations = []

        for obs in apriltag_observations:
            tag_id = int(obs["tag_id"])
            field_pose = marker_poses(CONFIG.arena_size).get(tag_id)

            if field_pose is None:
                continue

            usable_observations.append((obs, field_pose))

            print(
                f"[PNP_FIELD_TAG] "
                f"tag_id={tag_id} "
                f"x_m={float(field_pose.get('x_m', 0.0)):.3f} "
                f"y_m={float(field_pose.get('y_m', 0.0)):.3f} "
                f"z_m={float(field_pose.get('z_m', 0.0)):.3f} "
                f"yaw_rad={float(field_pose.get('yaw_rad', 0.0)):.3f}"
            )

        usable_tag_count = len(usable_observations)

        usable_tag_ids = [
            int(obs["tag_id"])
            for obs, _field_pose in usable_observations
        ]

        debug_tag_sets = [
            {18},
            {19},
            {18, 19},
        ]

        for debug_tags in debug_tag_sets:
            if not debug_tags.issubset(set(usable_tag_ids)):
                continue

            debug_observations = filter_observations_by_tag_ids(
                usable_observations,
                debug_tags,
            )

            debug_object_points, debug_image_points = build_pnp_points(debug_observations)

            if len(debug_object_points) < 4:
                continue

            debug_success, debug_rvec, debug_tvec = cv2.solvePnP(
                debug_object_points,
                debug_image_points,
                np.array(intrinsic_matrix, dtype=np.float64),
                np.array(distortion_coefficients, dtype=np.float64),
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

            if not debug_success:
                continue

            debug_projected, _ = cv2.projectPoints(
                debug_object_points,
                debug_rvec,
                debug_tvec,
                np.array(intrinsic_matrix, dtype=np.float64),
                np.array(distortion_coefficients, dtype=np.float64),
            )

            debug_projected = debug_projected.reshape(-1, 2)
            debug_errors = np.linalg.norm(debug_projected - debug_image_points, axis=1)
            debug_reproj = float(np.mean(debug_errors))

            debug_rotation_matrix, _ = cv2.Rodrigues(debug_rvec)
            debug_camera_position_world = -debug_rotation_matrix.T @ debug_tvec
            debug_camera_position_world = debug_camera_position_world.reshape(3)

            print(
                f"[PNP_COMPARE] "
                f"source={source_id} "
                f"tag_ids={sorted(debug_tags)} "
                f"reproj={debug_reproj:.2f}px "
                f"x_m={float(debug_camera_position_world[0]):.3f} "
                f"y_m={float(debug_camera_position_world[1]):.3f} "
                f"z_m={float(debug_camera_position_world[2]):.3f}"
            )

        object_points, image_points = build_pnp_points(usable_observations)

        point_count = len(object_points)

        if point_count < 4:
            return AprilTagPnPPoseResult(
                source_id=source_id,
                pose_x_m=None,
                pose_y_m=None,
                pose_theta_rad=None,
                ambiguity_score=None,
                reprojection_score=None,
                tag_count=usable_tag_count,
                timestamp_ms=timestamp_ms,
                valid=False,
            )

        print(
            f"[PNP_POINTS] "
            f"source={source_id} "
            f"tag_ids={usable_tag_ids} "
            f"usable_tags={usable_tag_count} "
            f"points={point_count}"
        )

        camera_matrix = np.array(intrinsic_matrix, dtype=np.float64)
        dist_coeffs = np.array(distortion_coefficients, dtype=np.float64)

        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

        is_valid = bool(success) and usable_tag_count >= 2

        projected_points, _ = cv2.projectPoints(
            object_points,
            rvec,
            tvec,
            camera_matrix,
            dist_coeffs,
        )

        projected_points = projected_points.reshape(-1, 2)

        pixel_errors = np.linalg.norm(
            projected_points - image_points,
            axis=1,
        )

        reprojection_score = float(np.mean(pixel_errors))

        rotation_matrix, _ = cv2.Rodrigues(rvec)

        camera_position_world = -rotation_matrix.T @ tvec
        camera_position_world = camera_position_world.reshape(3)

        pnp_x_m = float(camera_position_world[0])
        pnp_y_m = float(camera_position_world[1])
        pnp_z_m = float(camera_position_world[2])

        if isinstance(camera_to_robot_transform, dict):
            mount_x_m = float(camera_to_robot_transform.get("x_mm", 0.0)) / 1000.0
            mount_y_m = float(camera_to_robot_transform.get("y_mm", 0.0)) / 1000.0
        else:
            mount_x_m = float(getattr(camera_to_robot_transform, "x_mm", 0.0)) / 1000.0
            mount_y_m = float(getattr(camera_to_robot_transform, "y_mm", 0.0)) / 1000.0

        robot_x_m = pnp_x_m - mount_x_m
        robot_y_m = pnp_y_m - mount_y_m

        print(
            f"[PNP_TVEC] "
            f"source={source_id} "
            f"x={float(tvec[0][0]):.3f} "
            f"y={float(tvec[1][0]):.3f} "
            f"z={float(tvec[2][0]):.3f}"
        )

        print(
            f"[PNP_CAMERA_MATRIX] "
            f"fx={camera_matrix[0][0]:.2f} "
            f"fy={camera_matrix[1][1]:.2f} "
            f"cx={camera_matrix[0][2]:.2f} "
            f"cy={camera_matrix[1][2]:.2f}"
        )

        print(
            f"[PNP_IMAGE_POINTS] "
            f"min_x={float(np.min(image_points[:, 0])):.1f} "
            f"max_x={float(np.max(image_points[:, 0])):.1f} "
            f"min_y={float(np.min(image_points[:, 1])):.1f} "
            f"max_y={float(np.max(image_points[:, 1])):.1f}"
        )

        is_valid = is_valid and reprojection_score <= 10.0

        print(
            f"[PNP_SOLVE] "
            f"source={source_id} "
            f"success={success} "
            f"reproj={reprojection_score:.2f}px "
            f"rvec={rvec.ravel() if success else None} "
            f"tvec={tvec.ravel() if success else None}"
        )

        print(
            f"[PNP_CAMERA_POSE] "
            f"source={source_id} "
            f"x_m={pnp_x_m:.3f} "
            f"y_m={pnp_y_m:.3f} "
            f"z_m={pnp_z_m:.3f}"
        )

        print(
            f"[PNP_ROBOT_POSE_APPROX] "
            f"source={source_id} "
            f"x_m={robot_x_m:.3f} "
            f"y_m={robot_y_m:.3f}"
        )

        if usable_tag_count == 0:
            return AprilTagPnPPoseResult(
                source_id=source_id,
                pose_x_m=None,
                pose_y_m=None,
                pose_theta_rad=None,
                ambiguity_score=None,
                reprojection_score=reprojection_score,
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
            pose_x_m=robot_x_m,
            pose_y_m=robot_y_m,
            pose_theta_rad=None,
            ambiguity_score=None,
            reprojection_score=reprojection_score,
            tag_count=usable_tag_count,
            timestamp_ms=timestamp_ms,
            valid=is_valid,
        )