import math

from perception.robot_geometry import (
    gripper_standoff_pose_from_face,
)

from navigation.servoing.smooth_control_law import (
    Pose2D,
    SmoothControlLaw,
    SmoothControlParams,
)


class TestConfig:
    camera_mounts = {
        "front": {
            "x_mm": 0.0,
            "y_mm": 0.0,
            "yaw_deg": 0.0,
        }
    }

    gripper_mount = {
        "x_mm": 0.0,
        "y_mm": 0.0,
        "yaw_deg": 0.0,
    }


config = TestConfig()

law = SmoothControlLaw(
    SmoothControlParams(
        linear_max_mps=0.60,
        angular_max_rps=0.35,
    )
)


tests = [
    (
        "ID129 -90",
        {
            "distance": 1739.0,
            "bearing": 20.62,
            "yaw_deg": 5.28,
            "camera": "front",
        },
    ),
    (
        "ID129 -100",
        {
            "distance": 1735.0,
            "bearing": 9.93,
            "yaw_deg": -5.766,
            "camera": "front",
        },
    ),
    (
        "ID156 -90",
        {
            "distance": 2719.0,
            "bearing": 15.84,
            "yaw_deg": -41.23,
            "camera": "front",
        },
    ),
    (
        "ID156 -100",
        {
            "distance": 2721.0,
            "bearing": 5.26,
            "yaw_deg": -51.608,
            "camera": "front",
        },
    ),
]


for name, observation in tests:

    # Project-specific geometry:
    # one observed face -> desired robot pose.
    x_mm, y_mm, heading_rad = (
        gripper_standoff_pose_from_face(
            observation=observation,
            standoff_mm=300.0,
            config=config,
        )
    )

    # Explicit project units -> generic controller SI units.
    target = Pose2D(
        x_m=x_mm / 1000.0,
        y_m=y_mm / 1000.0,
        heading_rad=heading_rad,
    )

    # Generic pose control.
    result = law.calculate_regular_velocity(
        target
    )

    print(
        f"{name:11s} "
        f"goal=({x_mm:+7.1f}, {y_mm:+7.1f}, "
        f"{math.degrees(heading_rad):+7.3f}deg) "
        f"r={result.ego.r_m:.3f}m "
        f"phi={math.degrees(result.ego.phi_rad):+7.3f}deg "
        f"delta={math.degrees(result.ego.delta_rad):+7.3f}deg "
        f"k={result.curvature_per_m:+6.3f} "
        f"v={result.command.linear_x_mps:+.3f} "
        f"w={result.command.angular_z_rps:+.3f}"
    )