from navigation.visual_servoing.ibvs_target_centering import (
    IBVSParams,
    ImageBasedVisualServo,
)

import math


ibvs = ImageBasedVisualServo(
    IBVSParams(
        gain=1.0,
    )
)

tests_deg = [
    -15.0,
    -10.0,
    0.0,
    10.0,
    15.0,
    19.5,
]

for bearing_deg in tests_deg:
    bearing_rad = math.radians(bearing_deg)

    normalized_x = ibvs.normalized_x_from_bearing(
        bearing_rad
    )

    result = ibvs.calculate_target_centering(
        normalized_x=normalized_x
    )

    print(
        f"bearing={bearing_deg:+5.1f} deg  "
        f"x={result.normalized_x:+.4f}  "
        f"error={result.error:+.4f}  "
        f"L={result.interaction:.4f}  "
        f"omega={result.angular_z_rps:+.4f} rad/s"
    )