# localisation/providers/twodeadwheelimu.py

from __future__ import annotations

from hw_io.encoder import Encoder
from localisation.pose_types import Pose, PoseObservation
from localisation.providers.base import PoseProvider


class TwoDeadwheelImuProvider(PoseProvider):
    name = "twodeadwheelimu"

    def __init__(self, config) -> None:
        self.config = config

        self.deadwheel_parallel_encoder = Encoder(config.encoders["deadwheel_parallel"])
        self.deadwheel_perpendicular_encoder = Encoder(config.encoders["deadwheel_perpendicular"])

    def estimate(
        self,
        *,
        io,
        now_s: float,
        current_pose: Pose | None,
        arena_detections=None,
    ) -> PoseObservation | None:
        # Require hardware
        if not hasattr(io, "deadwheels"):
            return None
        if not hasattr(io, "imu"):
            return None

        # Raw counts
        raw_parallel = io.deadwheels["parallel"]
        raw_perpendicular = io.deadwheels["perpendicular"]

        # IMU data
        heading_rad = io.imu.get("heading_rad")
        if heading_rad is None:
            return None

        # Process encoders
        parallel = self.deadwheel_parallel_encoder.update(raw_parallel, now_s)
        perpendicular = self.deadwheel_perpendicular_encoder.update(raw_perpendicular, now_s)

        if not parallel.valid or not perpendicular.valid:
            return None

        # If no current pose exists yet, cannot integrate position reliably.
        if current_pose is None:
            return PoseObservation(
                x=0.0,
                y=0.0,
                heading=heading_rad,
                confidence=0.25,
                source=self.name,
                timestamp=now_s,
                meta={
                    "mode": "heading_seed_only",
                    "deadwheel_parallel_delta_mm": parallel.delta_units,
                    "deadwheel_perpendicular_delta_mm": perpendicular.delta_units,
                },
            )

        # Body-frame deltas (mm)
        forward_mm = parallel.delta_units
        lateral_mm = perpendicular.delta_units

        # Transform into world frame using current heading
        # For first pass, use current pose heading as the integration heading.
        import math

        theta = current_pose.heading if current_pose.heading is not None else heading_rad
        dx_world = forward_mm * math.cos(theta) - lateral_mm * math.sin(theta)
        dy_world = forward_mm * math.sin(theta) + lateral_mm * math.cos(theta)

        return PoseObservation(
            x=current_pose.x + dx_world,
            y=current_pose.y + dy_world,
            heading=heading_rad,
            confidence=0.85,
            source=self.name,
            timestamp=now_s,
            meta={
                "deadwheel_parallel_count": parallel.count,
                "deadwheel_perpendicular_count": perpendicular.count,
                "deadwheel_parallel_delta_mm": forward_mm,
                "deadwheel_perpendicular_delta_mm": lateral_mm,
            },
        )