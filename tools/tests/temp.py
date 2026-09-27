import math

from navigation.servoing.pbvs_nonholonomic import (
    NonHolonomicPBVS,
)
from navigation.servoing.servoing_types import Pose2D


LINEAR_MAX_MPS = 0.9
ANGULAR_MAX_RPS = 0.35


def wrap_to_pi(angle):
    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def limit_velocity(v, omega):
    """
    Scale v and omega together so neither exceeds its limit.

    This preserves omega / v and therefore preserves the
    curvature requested by the PBVS controller.
    """

    scale = max(
        1.0,
        abs(v) / LINEAR_MAX_MPS,
        abs(omega) / ANGULAR_MAX_RPS,
    )

    return (
        v / scale,
        omega / scale,
    )


def simulate(
    name,
    current,
    desired,
    *,
    dt=0.01,
    max_time_s=30.0,
):
    controller = NonHolonomicPBVS()

    for step in range(int(max_time_s / dt)):

        result = controller.calculate(
            current=current,
            desired=desired,
        )

        v, omega = limit_velocity(
            result.velocity.linear_mps,
            result.velocity.angular_rps,
        )

        current = Pose2D(
            x_m=current.x_m
                + v * math.cos(current.heading_rad) * dt,

            y_m=current.y_m
                + v * math.sin(current.heading_rad) * dt,

            heading_rad=wrap_to_pi(
                current.heading_rad + omega * dt
            ),
        )

        position_error = math.hypot(
            desired.x_m - current.x_m,
            desired.y_m - current.y_m,
        )

        heading_error = wrap_to_pi(
            desired.heading_rad - current.heading_rad
        )

        if (
            position_error < 0.001
            and abs(heading_error) < math.radians(0.1)
        ):
            break

    print(
        f"{name:18s} "
        f"dir={controller.direction:+d}  "
        f"end=({current.x_m:+.4f},{current.y_m:+.4f},"
        f"{math.degrees(current.heading_rad):+.3f}deg)  "
        f"pos_err={position_error:.6f}m  "
        f"hdg_err={math.degrees(heading_error):+.3f}deg  "
        f"t={(step + 1) * dt:.2f}s"
    )


simulate(
    "front left",
    Pose2D(0.0, 0.0, 0.0),
    Pose2D(1.0, 1.0, 0.0),
)

simulate(
    "rear left",
    Pose2D(0.0, 0.0, 0.0),
    Pose2D(-1.0, 1.0, 0.0),
)

simulate(
    "front left -> 90",
    Pose2D(0.0, 0.0, 0.0),
    Pose2D(1.0, 1.0, math.radians(90.0)),
)

simulate(
    "rear left -> 90",
    Pose2D(0.0, 0.0, 0.0),
    Pose2D(-1.0, 1.0, math.radians(90.0)),
)