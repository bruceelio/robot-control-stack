# perception/object_reconstruction.py

"""
Rigid-object reconstruction from changing fiducial-face visibility.

Purpose
-------
A physical object may expose one, two, or more tagged faces to the
camera. Navigation should not steer towards whichever individual face
happens to be visible.

Every usable visible face is therefore converted into an estimate of
the SAME rigid-body feature:

    the physical centre of the cube

Those centre estimates are fused to provide a stable object heading.

Range has deliberately different semantics:

    object heading
        -> physical cube centre

    object distance
        -> nearest currently visible exterior tagged face

The exterior distance is the operationally useful quantity for
approach, pickup, collision clearance and final commitment. A range to
the centre of a solid cube is generally not useful to those consumers
and would also change the established pickup-distance calibration.

Individual face observations are NOT modified. They remain available
through perception.object_faces for PBVS and other pose-aware
consumers.


Coordinate convention
---------------------
Internal reconstruction uses the canonical navigation convention:

    +x       = forward
    +y       = left
    +bearing = counter-clockwise / left

Units:

    position = metres
    distance = metres
    angles   = radians

The existing perception object interface still exposes:

    target["distance"]
        millimetres

    target["bearing"]
        degrees, positive image-right

for compatibility with callers which have not yet migrated to the
canonical SI contract.

Canonical values are also written as:

    target["distance_m"]
        exterior distance in metres

    target["bearing_rad"]
        reconstructed centre heading in radians


Literature
----------
The implementation does not reproduce the control laws in these
papers directly. It applies their central continuity principle in the
perception layer: appearance or disappearance of visual features
should not change the semantic meaning of the feature supplied to the
controller.

N. M. Garcia and E. Malis,
"Preserving the continuity of visual servoing despite changing image
features,"
IEEE/RSJ International Conference on Intelligent Robots and Systems
(IROS), 2004, pp. 1383-1388.
DOI: 10.1109/IROS.2004.1389589

N. Garcia-Aracil, E. Malis, R. Aracil-Santonja and C. Perez-Vidal,
"Continuous visual servoing despite the changes of visibility in image
features,"
IEEE Transactions on Robotics,
vol. 21, no. 6, pp. 1214-1220, 2005.
DOI: 10.1109/TRO.2005.855995

N. R. Gans and S. A. Hutchinson,
"Stable Visual Servoing Through Hybrid Switched-System Control,"
IEEE Transactions on Robotics,
vol. 23, no. 3, pp. 530-540, 2007.
DOI: 10.1109/TRO.2007.895067

E. Olson,
"AprilTag: A robust and flexible visual fiducial system,"
IEEE International Conference on Robotics and Automation (ICRA),
2011, pp. 3400-3407.
DOI: 10.1109/ICRA.2011.5979561


Design rule
-----------
Perception owns:

    faces -> object

Navigation owns:

    object -> motion

ApproachTargetServo therefore does not need to know how many faces are
visible or how the object heading was reconstructed.
"""

from __future__ import annotations

import math


def _finite_float(
    value,
) -> float | None:
    """
    Return a finite float or None.
    """

    if value is None:
        return None

    try:
        value = float(
            value
        )
    except (
        TypeError,
        ValueError,
    ):
        return None

    if not math.isfinite(
        value
    ):
        return None

    return value


def _face_object_estimate(
    *,
    face: dict,
    half_size_m: float,
) -> dict | None:
    """
    Convert one tagged cube face into:

        - one estimate of the physical cube centre
        - one exterior-distance measurement

    The detected tag centre lies on an exterior cube face.

    The tag pose provides the face normal. Moving half a cube width
    along the inward normal therefore gives an estimate of the
    physical cube centre.

    AprilTag/OpenCV-style camera axes:

        +x = image right
        +y = image down
        +z = camera forward

    Canonical planar geometry:

        +x = forward
        +y = left

    The existing pose convention gives the horizontal component of the
    tag +z normal as:

        forward =  cos(yaw) * cos(pitch)
        left    = -sin(yaw) * cos(pitch)

    Roll does not change the face normal and is not required for this
    reconstruction.
    """

    distance_mm = _finite_float(
        face.get(
            "distance"
        )
    )

    bearing_deg = _finite_float(
        face.get(
            "bearing"
        )
    )

    vertical_angle_rad = (
        _finite_float(
            face.get(
                "vertical_angle_rad"
            )
        )
    )

    yaw_rad = _finite_float(
        face.get(
            "yaw_rad"
        )
    )

    pitch_rad = _finite_float(
        face.get(
            "pitch_rad"
        )
    )

    if (
        distance_mm is None
        or distance_mm <= 0.0
        or bearing_deg is None
        or yaw_rad is None
        or pitch_rad is None
    ):
        return None

    if vertical_angle_rad is None:
        vertical_angle_rad = 0.0

    outside_distance_m = (
        distance_mm
        / 1000.0
    )

    # Raw perception bearing:
    #
    #     +bearing = image right
    #
    # Canonical geometry:
    #
    #     +bearing = left
    #
    bearing_rad = -math.radians(
        bearing_deg
    )

    # The detector distance is a 3-D range. Project it into the
    # horizontal plane before reconstructing planar object geometry.
    horizontal_distance_m = (
        outside_distance_m
        * math.cos(
            vertical_angle_rad
        )
    )

    face_x_m = (
        horizontal_distance_m
        * math.cos(
            bearing_rad
        )
    )

    face_y_m = (
        horizontal_distance_m
        * math.sin(
            bearing_rad
        )
    )

    # Horizontal projection of the face normal.
    #
    # For a top/bottom-facing face cos(pitch) approaches zero, so that
    # face naturally contributes very little horizontal centre offset.
    normal_x = (
        math.cos(
            yaw_rad
        )
        * math.cos(
            pitch_rad
        )
    )

    normal_y = (
        -math.sin(
            yaw_rad
        )
        * math.cos(
            pitch_rad
        )
    )

    centre_x_m = (
        face_x_m
        + (
            half_size_m
            * normal_x
        )
    )

    centre_y_m = (
        face_y_m
        + (
            half_size_m
            * normal_y
        )
    )

    return {
        "centre_x_m": (
            centre_x_m
        ),
        "centre_y_m": (
            centre_y_m
        ),
        "outside_distance_m": (
            outside_distance_m
        ),
        "camera": face.get(
            "camera"
        ),
    }


def _select_camera_estimates(
    *,
    estimates: list[dict],
    target: dict,
) -> list[dict]:
    """
    Do not fuse coordinates originating in different camera frames.

    Prefer the camera already associated with the canonical target.
    If no camera is recorded, use the camera which contributed the
    largest number of usable faces.

    A tie is resolved in favour of the camera with the nearest exterior
    observation.
    """

    if not estimates:
        return []

    by_camera: dict[
        str,
        list[dict],
    ] = {}

    for estimate in estimates:
        camera_name = estimate.get(
            "camera"
        )

        if camera_name is None:
            camera_name = ""

        camera_name = str(
            camera_name
        )

        by_camera.setdefault(
            camera_name,
            [],
        ).append(
            estimate
        )

    target_camera = target.get(
        "camera"
    )

    if target_camera is not None:
        target_camera = str(
            target_camera
        )

        if target_camera in by_camera:
            return by_camera[
                target_camera
            ]

    return max(
        by_camera.values(),
        key=lambda group: (
            len(group),
            -min(
                estimate[
                    "outside_distance_m"
                ]
                for estimate
                in group
            ),
        ),
    )


def reconstruct_cube_centres(
    *,
    perception,
    half_size_m: float,
    debug: bool = False,
) -> None:
    """
    Reconstruct one stable object observation for every currently
    visible tagged cube.

    Heading
    -------
    Each visible face independently estimates the same physical cube
    centre. Those Cartesian centre estimates are averaged and the
    bearing to that fused centre becomes the object's canonical heading.

    Distance
    --------
    Distance remains a distance to the OUTSIDE of the object.

    Specifically, it is the range to the nearest currently visible
    tagged face.

    This preserves the useful physical interpretation of distance for:

        approach
        pickup
        stacking
        collision clearance
        final blind commitment

    and avoids introducing a 65 mm centre-range offset into existing
    pickup calibration.

    Pose
    ----
    perception.object_faces is never modified.

    PBVS therefore continues to receive the original individual face
    pose information:

        yaw
        pitch
        roll
        face range
        face bearing

    Filtering
    ---------
    No temporal smoothing is performed here yet.

    First establish correct same-frame rigid-body reconstruction.
    Temporal filtering, if required, belongs after this reconstruction
    step rather than on the raw individual-face observations.
    """

    half_size_m = float(
        half_size_m
    )

    if (
        not math.isfinite(
            half_size_m
        )
        or half_size_m <= 0.0
    ):
        raise ValueError(
            "half_size_m must be "
            "finite and > 0"
        )

    object_faces = getattr(
        perception,
        "object_faces",
        None,
    )

    if not object_faces:
        return

    for kind in (
        "acidic",
        "basic",
    ):
        kind_faces = object_faces.get(
            kind,
            {},
        )

        object_memory = (
            perception.objects.get(
                kind,
                {},
            )
        )

        for (
            target_id,
            faces,
        ) in kind_faces.items():

            target_id = int(
                target_id
            )

            target = object_memory.get(
                target_id
            )

            if target is None:
                continue

            estimates = []

            for face in faces:
                estimate = (
                    _face_object_estimate(
                        face=face,
                        half_size_m=(
                            half_size_m
                        ),
                    )
                )

                if estimate is not None:
                    estimates.append(
                        estimate
                    )

            if not estimates:
                continue

            estimates = (
                _select_camera_estimates(
                    estimates=estimates,
                    target=target,
                )
            )

            if not estimates:
                continue

            # --------------------------------------------------
            # Rigid-object centre fusion
            # --------------------------------------------------
            #
            # Every estimate now refers to the SAME physical
            # point, so Cartesian averaging is meaningful.

            centre_x_m = (
                sum(
                    estimate[
                        "centre_x_m"
                    ]
                    for estimate
                    in estimates
                )
                / len(
                    estimates
                )
            )

            centre_y_m = (
                sum(
                    estimate[
                        "centre_y_m"
                    ]
                    for estimate
                    in estimates
                )
                / len(
                    estimates
                )
            )

            centre_bearing_rad = (
                math.atan2(
                    centre_y_m,
                    centre_x_m,
                )
            )

            # --------------------------------------------------
            # Exterior range
            # --------------------------------------------------
            #
            # Range deliberately remains a measurement to the
            # outside of the cube rather than its centre.
            #
            # When multiple faces are visible, use the nearest
            # visible exterior tagged face.

            outside_distance_m = min(
                estimate[
                    "outside_distance_m"
                ]
                for estimate
                in estimates
            )

            # --------------------------------------------------
            # Reconstruction consistency diagnostic
            # --------------------------------------------------

            centre_spread_m = max(
                math.hypot(
                    (
                        estimate[
                            "centre_x_m"
                        ]
                        - centre_x_m
                    ),
                    (
                        estimate[
                            "centre_y_m"
                        ]
                        - centre_y_m
                    ),
                )
                for estimate
                in estimates
            )

            # --------------------------------------------------
            # Canonical perception contract
            # --------------------------------------------------
            #
            # Range:
            #     nearest visible exterior face
            #
            # Heading:
            #     reconstructed physical object centre
            #
            # These deliberately do not describe the same physical
            # point. They describe the quantities navigation needs:
            #
            #     how far until the object exterior?
            #     which heading passes through its centre?

            target["distance_m"] = (
                outside_distance_m
            )

            target["bearing_rad"] = (
                centre_bearing_rad
            )

            # --------------------------------------------------
            # Current compatibility interface
            # --------------------------------------------------
            #
            # Existing callers still consume:
            #
            #     distance = mm
            #     bearing  = degrees, positive image-right

            target["distance"] = (
                outside_distance_m
                * 1000.0
            )

            target["bearing"] = (
                -math.degrees(
                    centre_bearing_rad
                )
            )

            # --------------------------------------------------
            # Diagnostics
            # --------------------------------------------------

            target[
                "reconstruction_face_count"
            ] = len(
                estimates
            )

            target[
                "centre_spread_m"
            ] = centre_spread_m

            target[
                "distance_source"
            ] = (
                "nearest_visible_face"
            )

            target[
                "bearing_source"
            ] = (
                "rigid_cube_face_fusion"
            )

            if debug:
                print(
                    "[OBJECT_RECON] "
                    f"kind={kind} "
                    f"id={target_id} "
                    f"faces="
                    f"{len(estimates)} "
                    f"outside="
                    f"{outside_distance_m:.3f}m "
                    f"bearing="
                    f"{math.degrees(centre_bearing_rad):+.2f}deg "
                    f"spread="
                    f"{centre_spread_m:.3f}m"
                )


__all__ = [
    "reconstruct_cube_centres",
]