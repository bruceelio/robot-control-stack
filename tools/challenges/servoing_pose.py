# tools/challenges/servoing_pose.py

from __future__ import annotations

from primitives.base import PrimitiveStatus
from primitives.motion import Drive, Rotate
from perception.perception import Perception, sense
from perception.robot_geometry import (
    gripper_standoff_pose_from_face,
)

from motion_backends.velocity import VelocityMotionBackend

from config import CONFIG

from navigation.control.smooth_control_law import (
    Pose2D,
    SmoothControlParams,
)
from navigation.visual_servoing.pose_servo_controller import (
    PoseServoController,
    PoseServoMode,
)

import math
import time


# ==================================================
# Challenge configuration
# ==================================================

LOOP_DELAY_S = 0.10

POSE_TEST_OBJECT_ID = 162
POSE_TEST_STANDOFF_MM = 300.0
POSE_MOTION_TEST_DURATION_S = 8.00
PBVS_LINEAR_MAX_MPS = 0.90
PBVS_ANGULAR_MAX_RPS = 0.35

# SR2026 object AprilTags.
OBJECT_TAG_MIN = 100
OBJECT_TAG_MAX = 179


# Smooth-Control FOV admissibility selector.
POSE_TEST_CAMERA_FOV_DEG = 45.0
CAMERA_FOV_MARGIN_DEG = 3.0

SMOOTH_FOV_SAFE_HALF_DEG = (
    POSE_TEST_CAMERA_FOV_DEG / 2.0
    - CAMERA_FOV_MARGIN_DEG
)
SMOOTH_FOV_PREDICTION_DT_S = 0.10
SMOOTH_FOV_PREDICTION_MAX_TIME_S = 5.0
SMOOTH_FOV_POSITION_TOLERANCE_M = 0.01


# ==================================================
# Helpers
# ==================================================

def _deg(value):
    if value is None:
        return None

    return math.degrees(float(value))


def _fmt_deg(value):
    value_deg = _deg(value)

    if value_deg is None:
        return "None"

    return f"{value_deg:+.2f}"


def _fmt_mm(value):
    if value is None:
        return "None"

    return f"{float(value):.1f}"


# ==================================================
# Challenge
# ==================================================

def run(controller):
    """
    Initial pose-visual_servoing data challenge.

    A short controlled pose-servo motion test is commanded.

    For every visible object AprilTag, print the raw geometry
    supplied by the vision source:

        distance
        horizontal bearing
        vertical angle
        yaw
        pitch
        roll

    This challenge is deliberately simple. Its purpose is to
    establish the simulator's pose conventions before designing
    ServoingControllerPose.
    """

    print("\n=== SERVOING POSE CHALLENGE ===")
    print(
        "[SERVOING_POSE] "
        "short controlled pose-servo motion test"
    )

    # --------------------------------------------------
    # Move to test observation pose
    # --------------------------------------------------

    motion_backend = controller.motion_backend

    print("[SERVOING_POSE] drive forward 600 mm")

    drive = Drive(distance_mm=600)
    drive.start(motion_backend=motion_backend)

    while (
            drive.update(motion_backend=motion_backend)
            == PrimitiveStatus.RUNNING
    ):
        controller.io.sleep(0.01)



    print("[SERVOING_POSE] rotate right 80 deg")

    rotate = Rotate(angle_deg=-80)
    rotate.start(motion_backend=motion_backend)

    while (
        rotate.update(motion_backend=motion_backend)
        == PrimitiveStatus.RUNNING
    ):
        controller.io.sleep(0.01)

    print("[SERVOING_POSE] movement complete - settling")

    controller.io.sleep(0.5)

    print("[SERVOING_POSE] starting pose observations")

    pose_perception = Perception(controller.io)

    pose_servo = PoseServoController(
        camera_half_fov_rad=math.radians(
            POSE_TEST_CAMERA_FOV_DEG / 2.0
        ),
        fov_activation_margin_rad=math.radians(
            CAMERA_FOV_MARGIN_DEG
        ),
        safe_half_fov_rad=math.radians(
            SMOOTH_FOV_SAFE_HALF_DEG
        ),
        smooth_params=SmoothControlParams(
            k_phi=2,
            linear_max_mps=0.60,
            angular_max_rps=0.35,
        ),
        pbvs_linear_max_mps=PBVS_LINEAR_MAX_MPS,
        pbvs_angular_max_rps=PBVS_ANGULAR_MAX_RPS,
        prediction_dt_s=SMOOTH_FOV_PREDICTION_DT_S,
        prediction_max_time_s=(
            SMOOTH_FOV_PREDICTION_MAX_TIME_S
        ),
        prediction_position_tolerance_m=(
            SMOOTH_FOV_POSITION_TOLERANCE_M
        ),
        config=CONFIG,
    )

    velocity_backend = VelocityMotionBackend(
        lvl2=controller.lvl2,
        config=CONFIG,
    )

    motion_test_started_s = None

    try:
        while True:

            now_s = time.time()

            vision_message = controller._get_vision_message(
                camera_name="front",
                now_s=now_s,
            )

            if vision_message is None:
                print("[SERVOING_POSE] no vision message")

                if motion_test_started_s is not None:
                    velocity_backend.stop()

                    print(
                        "[POSE_MOTION] ABORT "
                        "vision message lost"
                    )

                    return

                controller.io.sleep(
                    LOOP_DELAY_S
                )
                continue

            markers = vision_message.get(
                "markers",
                []
            )

            sense(
                controller.io,
                pose_perception,
                latest_markers=markers,
                latest_vision_message=vision_message,
                camera_name="front",
                stop_robot=False,
            )

            face_groups = []

            for kind in ("acidic", "basic"):
                for object_id, faces in (
                        pose_perception.object_faces[kind].items()
                ):
                    face_groups.append(
                        (kind, object_id, faces)
                    )

            if not face_groups:
                print(
                    "[SERVOING_POSE] "
                    "no object faces visible"
                )

                if motion_test_started_s is not None:
                    velocity_backend.stop()

                    print(
                        "[POSE_MOTION] ABORT "
                        "all object faces lost"
                    )

                    return

                controller.io.sleep(
                    LOOP_DELAY_S
                )
                continue

            total_faces = sum(
                len(faces)
                for _, _, faces in face_groups
            )

            print(
                f"[SERVOING_POSE] "
                f"objects={len(face_groups)} "
                f"faces={total_faces}"
            )

            test_face_seen = False
            for kind, object_id, faces in face_groups:

                print(
                    f"[POSE_OBJECT] "
                    f"id={object_id} "
                    f"kind={kind} "
                    f"faces={len(faces)}"
                )

                selected_face_index = None

                if object_id == POSE_TEST_OBJECT_ID:
                    selected_face_index = min(
                        range(len(faces)),
                        key=lambda i: (
                                abs(float(faces[i]["pitch_deg"]))
                                + abs(float(faces[i]["roll_deg"]))
                        ),
                    )

                for face_index, face in enumerate(faces):
                    print(
                        f"[POSE_FACE] "
                        f"id={object_id} "
                        f"face={face_index} "
                        f"dist={face['distance']:.1f}mm "
                        f"bearing={face['bearing']:+.2f}deg "
                        f"vertical={face['vertical_angle_deg']:+.2f}deg "
                        f"yaw={face['yaw_deg']}deg "
                        f"pitch={face['pitch_deg']}deg "
                        f"roll={face['roll_deg']}deg"
                    )

                    if (
                            object_id == POSE_TEST_OBJECT_ID
                            and face_index == selected_face_index
                    ):
                        test_face_seen = True

                        if motion_test_started_s is None:
                            motion_test_started_s = time.time()

                            print(
                                "[POSE_MOTION] START "
                                f"duration="
                                f"{POSE_MOTION_TEST_DURATION_S:.2f}s"
                            )

                        elapsed_s = (
                            time.time()
                            - motion_test_started_s
                        )

                        if (
                            elapsed_s
                            >= POSE_MOTION_TEST_DURATION_S
                        ):
                            velocity_backend.stop()

                            print(
                                "[POSE_MOTION] COMPLETE "
                                f"elapsed={elapsed_s:.3f}s"
                            )

                            return

                        (
                            goal_x_mm,
                            goal_y_mm,
                            goal_heading_rad,
                        ) = gripper_standoff_pose_from_face(
                            observation=face,
                            standoff_mm=POSE_TEST_STANDOFF_MM,
                            config=CONFIG,
                        )

                        target_pose = Pose2D(
                            x_m=goal_x_mm / 1000.0,
                            y_m=goal_y_mm / 1000.0,
                            heading_rad=goal_heading_rad,
                        )

                        result = pose_servo.update(
                            tag_id=object_id,
                            observation=face,
                            target_pose=target_pose,
                            timestamp=time.time(),
                        )

                        if result.fov_prediction is not None:
                            prediction = result.fov_prediction

                            print(
                                f"[SMOOTH_FOV_PREDICT] "
                                f"current="
                                f"{face['bearing']:+.3f}deg "
                                f"predicted_max="
                                f"{math.degrees(prediction.max_abs_bearing_rad):.3f}deg "
                                f"limit="
                                f"{SMOOTH_FOV_SAFE_HALF_DEG:.3f}deg "
                                f"admissible="
                                f"{prediction.admissible} "
                                f"reached_goal="
                                f"{prediction.reached_goal} "
                                f"steps={prediction.steps}"
                            )



                        if result.mode_changed:
                            print(
                                f"[CONTROL_MODE] "
                                f"mode={result.mode.value.upper()} "
                                f"elapsed={elapsed_s:.3f}s"
                            )

                        if result.mode == PoseServoMode.PBVS:
                            pbvs_result = result.pbvs_result

                            print(
                                f"[PBVS_CONTROL] "
                                f"dir={pbvs_result.direction:+d} "
                                f"rho={pbvs_result.rho_m:.3f}m "
                                f"alpha="
                                f"{math.degrees(pbvs_result.alpha_rad):+.3f}deg "
                                f"beta="
                                f"{math.degrees(pbvs_result.beta_rad):+.3f}deg "
                                f"v="
                                f"{result.command.linear_x_mps:+.3f}m/s "
                                f"w="
                                f"{result.command.angular_z_rps:+.3f}rad/s"
                            )

                            fov = result.pbvs_fov_result

                            if fov is not None:
                                weight_text = (
                                    "inf"
                                    if math.isinf(fov.weight)
                                    else f"{fov.weight:.3f}"
                                )

                                print(
                                    f"[PBVS_FOV] "
                                    f"centre="
                                    f"{face['bearing']:+.3f}deg "
                                    f"feature="
                                    f"{math.degrees(fov.feature_bearing_rad):+.3f}deg "
                                    f"range="
                                    f"{fov.feature_range_m:.3f}m "
                                    f"outward={fov.outward} "
                                    f"active={fov.active} "
                                    f"weight={weight_text} "
                                    f"rate_nom="
                                    f"{fov.nominal_bearing_rate_rps:+.3f} "
                                    f"rate_out="
                                    f"{fov.constrained_bearing_rate_rps:+.3f}"
                                )

                        else:
                            smooth_result = result.smooth_result

                            print(
                                f"[SMOOTH_CONTROL] "
                                f"r={smooth_result.ego.r_m:.3f}m "
                                f"phi="
                                f"{math.degrees(smooth_result.ego.phi_rad):+.3f}deg "
                                f"delta="
                                f"{math.degrees(smooth_result.ego.delta_rad):+.3f}deg "
                                f"v="
                                f"{result.command.linear_x_mps:+.3f}m/s "
                                f"w="
                                f"{result.command.angular_z_rps:+.3f}rad/s"
                            )

                        velocity_command = result.command

                        velocity_backend.update(
                            velocity_command
                        )

                        print(
                            f"[POSE_MOTION] "
                            f"elapsed={elapsed_s:.3f}s "
                            f"v="
                            f"{velocity_command.linear_x_mps:+.3f}m/s "
                            f"w="
                            f"{velocity_command.angular_z_rps:+.3f}rad/s"
                        )

            if (
                motion_test_started_s is not None
                and not test_face_seen
            ):
                velocity_backend.stop()

                print(
                    "[POSE_MOTION] ABORT "
                    "test face lost"
                )

                return

            controller.io.sleep(
                LOOP_DELAY_S
            )



    finally:

        velocity_backend.stop()

        print("[SERVOING_POSE] drivetrain stopped")

        print("=== SERVOING POSE CHALLENGE COMPLETE ===\n")