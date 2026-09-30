# tools/challenges/lift_vision_sweep.py
"""
Lift-position / vision-occlusion diagnostic.

Run from the normal starting position with NO cube attached to the gripper.

Purpose:
- Move the lift through the useful servo range.
- At each position, let the normal controller perception/localisation path run.
- Record which AprilTags are visible and whether localisation is still using vision.
- Establish whether the bare lift/gripper obstructs the camera.

This challenge does not move the drivetrain or actuate the gripper.
"""

from __future__ import annotations

import math

from config import CONFIG
from config.strategy import STARTUP_SCRIPT, StartupScript


WEBOTS_POSITIONS = (
    -1.00,
    -0.75,
    -0.50,
    -0.25,
    0.00,
    0.25,
    0.50,
    0.75,
    1.00,
)

# Bobbot positions should only be enabled after confirming the complete
# mechanical travel is safe on the physical lift.
BOBBOT_POSITIONS = None

SETTLE_S = 0.60
SAMPLES_PER_POSITION = 5


def _is_configured(config, category: str, name: str) -> bool:
    has_io = getattr(config, "has_io", None)
    if callable(has_io):
        try:
            return bool(has_io(category, name))
        except KeyError:
            return False

    return bool(
        getattr(config, "io", {}).get(f"{category}.{name}")
    )


def _positions_for(config):
    robot_id = str(getattr(config, "robot_id", ""))

    if robot_id == "webots_robot":
        return WEBOTS_POSITIONS

    if robot_id == "bob_bot":
        if BOBBOT_POSITIONS is None:
            raise ValueError(
                "Set BOBBOT_POSITIONS to mechanically checked servo targets "
                "before running the real-robot lift vision sweep."
            )
        return BOBBOT_POSITIONS

    raise ValueError(
        f"No approved lift vision sweep for robot_id={robot_id!r}"
    )


def _current_tag_ids(controller) -> list[int]:
    """
    Read only the canonical AprilTag observations from the latest completed
    perception frame. This deliberately avoids perception memory.
    """
    perception = getattr(controller, "perception", None)
    observations = (
        []
        if perception is None
        else list(
            getattr(
                perception,
                "latest_apriltag_observations",
                [],
            )
            or []
        )
    )

    ids = []
    for observation in observations:
        tag_id = getattr(observation, "tag_id", None)
        if tag_id is None:
            continue

        try:
            ids.append(int(tag_id))
        except (TypeError, ValueError):
            continue

    return sorted(ids)


def _localisation_source(controller) -> str:
    localisation = getattr(controller, "localisation", None)
    pose = None if localisation is None else getattr(localisation, "pose", None)

    if pose is None:
        return "none"

    return str(getattr(pose, "source", "unknown"))


def _vision_localisation_active(source: str) -> bool:
    return (
        source not in ("none", "unknown", "startup_config", "commanded_motion")
        and (
            "markers" in source
            or "apriltag" in source
            or "vision" in source
        )
    )


def run(controller):
    """
    Sweep the lift at the start position and measure camera visibility.

    PRECONDITION:
    - no cube attached to the gripper
    - robot remains at the normal starting position
    - STARTUP_SCRIPT = NONE
    """
    config = getattr(controller, "config", CONFIG)
    io = controller.io

    if STARTUP_SCRIPT != StartupScript.NONE:
        raise RuntimeError(
            "Lift vision sweep requires STARTUP_SCRIPT=NONE so the robot "
            "starts with no cube attached."
        )

    if not _is_configured(config, "servo", "lift"):
        raise RuntimeError(
            "servo.lift is not enabled in the resolved robot profile"
        )

    positions = tuple(_positions_for(config))

    if any(
        not isinstance(position, (int, float))
        or not math.isfinite(position)
        or not -1.0 <= position <= 1.0
        for position in positions
    ):
        raise ValueError(
            "All lift sweep positions must be finite values in [-1.0, +1.0]"
        )

    lift = io.servo["lift"]
    carry = getattr(config, "lift_carry_position", None)

    print("\n=== LIFT VISION SWEEP ===")
    print(
        f"[LIFT_VISION] robot={getattr(config, 'robot_id', 'unknown')} "
        f"positions={positions}"
    )
    print(
        "[LIFT_VISION] PRECONDITION: no cube attached; "
        "drivetrain remains stationary"
    )

    results = []

    try:
        for position in positions:
            print(f"\n[LIFT_VISION][MOVE] position={position:+.2f}")

            lift.position = float(position)
            io.sleep(SETTLE_S)

            # Discard one post-move frame, then record fresh frames.
            controller.tick()

            frames = []

            for sample in range(1, SAMPLES_PER_POSITION + 1):
                controller.tick()

                ids = _current_tag_ids(controller)
                arena_ids = [tag_id for tag_id in ids if tag_id < 100]
                object_ids = [
                    tag_id
                    for tag_id in ids
                    if 100 <= tag_id <= 179
                ]

                source = _localisation_source(controller)
                vision_loc = _vision_localisation_active(source)

                frames.append({
                    "position": float(position),
                    "sample": sample,
                    "tag_ids": tuple(ids),
                    "arena_ids": tuple(arena_ids),
                    "object_ids": tuple(object_ids),
                    "localisation_source": source,
                    "vision_localisation": vision_loc,
                })

                print(
                    f"[LIFT_VISION][FRAME] "
                    f"position={position:+.2f} "
                    f"sample={sample}/{SAMPLES_PER_POSITION} "
                    f"tags={len(ids)} ids={ids} "
                    f"arena={arena_ids} objects={object_ids} "
                    f"loc={source}"
                )

            frames_with_tags = sum(
                1 for frame in frames if frame["tag_ids"]
            )
            frames_with_arena = sum(
                1 for frame in frames if frame["arena_ids"]
            )
            frames_with_objects = sum(
                1 for frame in frames if frame["object_ids"]
            )
            frames_with_vision_loc = sum(
                1 for frame in frames if frame["vision_localisation"]
            )

            all_ids = sorted({
                tag_id
                for frame in frames
                for tag_id in frame["tag_ids"]
            })
            all_arena_ids = sorted({
                tag_id
                for frame in frames
                for tag_id in frame["arena_ids"]
            })
            all_object_ids = sorted({
                tag_id
                for frame in frames
                for tag_id in frame["object_ids"]
            })

            results.append({
                "position": float(position),
                "frames": len(frames),
                "frames_with_tags": frames_with_tags,
                "frames_with_arena": frames_with_arena,
                "frames_with_objects": frames_with_objects,
                "frames_with_vision_localisation": frames_with_vision_loc,
                "tag_ids": tuple(all_ids),
                "arena_ids": tuple(all_arena_ids),
                "object_ids": tuple(all_object_ids),
            })

            print(
                f"[LIFT_VISION][SUMMARY] "
                f"position={position:+.2f} "
                f"tags={frames_with_tags}/{len(frames)} "
                f"arena={frames_with_arena}/{len(frames)} "
                f"objects={frames_with_objects}/{len(frames)} "
                f"vision_loc={frames_with_vision_loc}/{len(frames)} "
                f"ids={all_ids}"
            )

    finally:
        if carry is not None:
            lift.position = float(carry)
            io.sleep(SETTLE_S)
            print(
                f"\n[LIFT_VISION][RESTORE] "
                f"carry={float(carry):+.2f}"
            )

    print("\n=== LIFT VISION SWEEP COMPLETE ===")
    return results
