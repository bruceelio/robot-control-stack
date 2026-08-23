from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable

import cv2
import numpy as np
from pupil_apriltags import Detector


# --------------------------------------------------
# Generic AprilTag data model
# --------------------------------------------------

@dataclass
class MarkerPosition:
    distance: float | None          # mm
    horizontal_angle: float | None  # radians
    vertical_angle: float | None    # radians


@dataclass
class MarkerOrientation:
    yaw: float | None = None
    pitch: float | None = None
    roll: float | None = None


@dataclass
class Marker:
    id: int
    position: MarkerPosition
    orientation: MarkerOrientation
    size: float | None = None
    decision_margin: float = 0.0
    family: str = "tag36h11"
    center_px: tuple[float, float] | None = None
    corners_px: list[tuple[float, float]] | None = None
    x_m: float | None = None
    y_m: float | None = None
    z_m: float | None = None
    pose_err: float | None = None


# --------------------------------------------------
# Shared AprilTag processor
# --------------------------------------------------

class AprilTagProcessor:
    """
    Camera-independent AprilTag detection and marker processing.

    Input:
        RGB image

    Output:
        list[Marker]

    This class does not open or configure camera hardware.
    """

    def __init__(
        self,
        *,
        families: str = "tag36h11",
        tag_size_m: float | None = None,
        tag_size_for_id: Callable[[int], float] | None = None,
        camera_params: tuple[float, float, float, float] | None = None,
        quad_decimate: float = 1.5,
        nthreads: int = 2,
        quad_sigma: float = 0.0,
        refine_edges: int = 1,
        decode_sharpening: float = 0.25,
        apriltag_debug: int = 0,
        min_decision_margin: float = 20.0,
    ) -> None:

        self.families = families
        self.tag_size_m = tag_size_m
        self.tag_size_for_id = tag_size_for_id
        self.camera_params = camera_params

        self.pose_enabled = camera_params is not None

        self.quad_decimate = quad_decimate
        self.nthreads = nthreads
        self.quad_sigma = quad_sigma
        self.refine_edges = refine_edges
        self.decode_sharpening = decode_sharpening
        self.apriltag_debug = apriltag_debug
        self.min_decision_margin = min_decision_margin

        self._mixed_size_mode = tag_size_for_id is not None
        self._single_size_mode = tag_size_m is not None

        if self._mixed_size_mode and self._single_size_mode:
            raise ValueError(
                "Specify either tag_size_m or tag_size_for_id, not both"
            )

        self._detector = Detector(
            families=self.families,
            nthreads=self.nthreads,
            quad_decimate=self.quad_decimate,
            quad_sigma=self.quad_sigma,
            refine_edges=self.refine_edges,
            decode_sharpening=self.decode_sharpening,
            debug=self.apriltag_debug,
        )

    def process(self, frame_rgb: np.ndarray) -> list[Marker]:
        """
        Detect and process AprilTags in one RGB image.
        """
        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)

        if self._single_size_mode:
            detections = self._detector.detect(
                gray,
                estimate_tag_pose=self.pose_enabled,
                camera_params=(
                    list(self.camera_params)
                    if self.camera_params
                    else None
                ),
                tag_size=(
                    self.tag_size_m
                    if self.pose_enabled
                    else None
                ),
            )
        else:
            detections = self._detector.detect(
                gray,
                estimate_tag_pose=False,
                camera_params=None,
                tag_size=None,
            )

        detections = [
            det
            for det in detections
            if float(det.decision_margin)
            >= self.min_decision_margin
        ]

        return [
            self._detection_to_marker(det)
            for det in detections
        ]

    def _resolve_size_for_detection(
        self,
        tag_id: int,
    ) -> float | None:
        if self.tag_size_for_id is not None:
            return self.tag_size_for_id(tag_id)

        return self.tag_size_m

    def _angles_from_center(
        self,
        center_px: tuple[float, float],
    ) -> tuple[float | None, float | None]:
        """
        Compute horizontal and vertical angles directly from image
        centre and camera intrinsics.
        """
        if self.camera_params is None:
            return None, None

        fx, fy, cx, cy = self.camera_params
        px, py = center_px

        horizontal_angle = math.atan2(px - cx, fx)
        vertical_angle = math.atan2(py - cy, fy)

        return horizontal_angle, vertical_angle

    def _camera_matrix(self) -> np.ndarray | None:
        if self.camera_params is None:
            return None

        fx, fy, cx, cy = self.camera_params

        return np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    def _object_point_variants(
        self,
        tag_size_m: float,
    ) -> list[np.ndarray]:
        """
        Build possible square-tag object-point orderings.

        This retains the current behaviour which tolerates uncertainty
        in detector corner ordering.
        """
        half = tag_size_m / 2.0

        base = np.array(
            [
                [-half, -half, 0.0],
                [ half, -half, 0.0],
                [ half,  half, 0.0],
                [-half,  half, 0.0],
            ],
            dtype=np.float64,
        )

        variants = []

        for shift in range(4):
            variants.append(
                np.roll(base, -shift, axis=0)
            )

        reversed_base = base[::-1].copy()

        for shift in range(4):
            variants.append(
                np.roll(reversed_base, -shift, axis=0)
            )

        return variants

    def _solve_pose_best(
        self,
        corners_px: list[tuple[float, float]],
        tag_size_m: float,
    ) -> tuple[
        float | None,
        float | None,
        float | None,
        float | None,
        float | None,
        float | None,
        float | None,
    ]:
        """
        Estimate the pose of one tag using the existing OpenCV
        solvePnP approach.

        Returns:
            x_m, y_m, z_m,
            yaw, pitch, roll,
            reprojection_error
        """
        camera_matrix = self._camera_matrix()

        if camera_matrix is None:
            return (
                None, None, None,
                None, None, None,
                None,
            )

        dist_coeffs = np.zeros(
            (4, 1),
            dtype=np.float64,
        )

        image_points = np.array(
            corners_px,
            dtype=np.float64,
        )

        best = None
        best_err = None

        for object_points in self._object_point_variants(
            tag_size_m
        ):
            try:
                ok, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    camera_matrix,
                    dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )

            except cv2.error:
                continue

            if not ok:
                continue

            pose_R, _ = cv2.Rodrigues(rvec)

            yaw, pitch, roll = (
                self._rotation_matrix_to_ypr(pose_R)
            )

            projected, _ = cv2.projectPoints(
                object_points,
                rvec,
                tvec,
                camera_matrix,
                dist_coeffs,
            )

            projected = projected.reshape(-1, 2)

            err = float(
                np.mean(
                    np.linalg.norm(
                        projected - image_points,
                        axis=1,
                    )
                )
            )

            x_m = float(tvec[0][0])
            y_m = float(tvec[1][0])
            z_m = float(tvec[2][0])

            candidate = (
                x_m,
                y_m,
                z_m,
                yaw,
                pitch,
                roll,
                err,
            )

            if best is None:
                best = candidate
                best_err = err
                continue

            best_z = best[2]

            if (
                best_z is not None
                and z_m > 0
                and best_z <= 0
            ):
                best = candidate
                best_err = err
                continue

            if (
                (z_m > 0) == (best_z > 0)
                and err < best_err
            ):
                best = candidate
                best_err = err

        if best is None:
            return (
                None, None, None,
                None, None, None,
                None,
            )

        return best

    def _detection_to_marker(
        self,
        det: Any,
    ) -> Marker:

        family = det.tag_family

        if isinstance(family, bytes):
            family = family.decode(
                "utf-8",
                errors="replace",
            )

        tag_id = int(det.tag_id)

        center_px = (
            float(det.center[0]),
            float(det.center[1]),
        )

        corners_px = [
            (float(x), float(y))
            for x, y in det.corners
        ]

        resolved_size_m = (
            self._resolve_size_for_detection(tag_id)
        )

        x_m = None
        y_m = None
        z_m = None
        pose_err = None

        distance_mm: float | None = None

        yaw = None
        pitch = None
        roll = None

        horizontal_angle, vertical_angle = (
            self._angles_from_center(center_px)
        )

        # --------------------------------------------------
        # Single-size detector-provided pose
        # --------------------------------------------------

        if getattr(det, "pose_t", None) is not None:

            tx, ty, tz = (
                np.array(det.pose_t)
                .reshape(-1)
                .tolist()[:3]
            )

            x_m = float(tx)
            y_m = float(ty)
            z_m = float(tz)

            distance_mm = (
                1000.0
                * math.sqrt(
                    x_m * x_m
                    + y_m * y_m
                    + z_m * z_m
                )
            )

            yaw, pitch, roll = (
                self._rotation_matrix_to_ypr(
                    getattr(det, "pose_R", None)
                )
            )

            if getattr(det, "pose_err", None) is not None:
                pose_err = float(det.pose_err)

        # --------------------------------------------------
        # Mixed-size per-tag solvePnP
        # --------------------------------------------------

        elif (
            resolved_size_m is not None
            and self.camera_params is not None
        ):

            (
                x_m,
                y_m,
                z_m,
                yaw,
                pitch,
                roll,
                pose_err,
            ) = self._solve_pose_best(
                corners_px,
                resolved_size_m,
            )

            if (
                x_m is not None
                and y_m is not None
                and z_m is not None
            ):
                distance_mm = (
                    1000.0
                    * math.sqrt(
                        x_m * x_m
                        + y_m * y_m
                        + z_m * z_m
                    )
                )

        return Marker(
            id=tag_id,

            position=MarkerPosition(
                distance=distance_mm,
                horizontal_angle=horizontal_angle,
                vertical_angle=vertical_angle,
            ),

            orientation=MarkerOrientation(
                yaw=yaw,
                pitch=pitch,
                roll=roll,
            ),

            size=resolved_size_m,
            decision_margin=float(
                det.decision_margin
            ),
            family=str(family),

            center_px=center_px,
            corners_px=corners_px,

            x_m=x_m,
            y_m=y_m,
            z_m=z_m,

            pose_err=pose_err,
        )

    @staticmethod
    def _rotation_matrix_to_ypr(
        pose_R: np.ndarray | None,
    ) -> tuple[
        float | None,
        float | None,
        float | None,
    ]:

        if pose_R is None:
            return None, None, None

        yaw = math.atan2(
            float(pose_R[0, 2]),
            float(pose_R[2, 2]),
        )

        pitch = math.atan2(
            -float(pose_R[1, 2]),
            math.sqrt(
                float(pose_R[1, 0]) ** 2
                + float(pose_R[1, 1]) ** 2
            ),
        )

        roll = math.atan2(
            float(pose_R[1, 0]),
            float(pose_R[1, 1]),
        )

        return yaw, pitch, roll