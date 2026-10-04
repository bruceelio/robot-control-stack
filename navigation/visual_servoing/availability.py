# navigation/visual_servoing/availability.py

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class VisualServoAvailability:
    """
    Result of evaluating whether visual servoing may currently be used.

    This module deliberately does not determine camera health or observation
    reliability itself. Those are external health metrics supplied by the
    perception / runtime-health layer.

    Current policy:

        available =
            servoing_enabled
            AND camera_healthy
            AND observations_reliable
            AND observation_fresh

    For the initial implementation, callers may pass:

        camera_healthy=True
        observations_reliable=True

    This allows real health metrics to be introduced later without changing
    consumers such as target approach, visual alignment, docking, or return
    guidance.
    """

    available: bool

    servoing_enabled: bool
    camera_healthy: bool
    observations_reliable: bool
    observation_fresh: bool

    observation_age_s: float | None
    max_observation_age_s: float

    reason: str | None


def evaluate_visual_servo_availability(
    *,
    servoing_enabled: bool,
    observation_age_s: float | None,
    max_observation_age_s: float,
    camera_healthy: bool = True,
    observations_reliable: bool = True,
) -> VisualServoAvailability:
    """
    Evaluate whether closed-loop visual servoing is currently available.

    Parameters
    ----------
    servoing_enabled:
        Master configuration switch for visual servoing.

    observation_age_s:
        Age of the observation that would drive the servo controller.
        None means that no usable observation is currently available.

    max_observation_age_s:
        Maximum acceptable observation age.

    camera_healthy:
        External camera-health result.

        Currently callers may leave this at True. In future this can be
        supplied by camera/runtime health monitoring using metrics such as
        process state, frame cadence, dropped frames, or acquisition errors.

    observations_reliable:
        External perception-reliability result.

        Currently callers may leave this at True. In future this can represent
        metrics such as detection continuity, outlier rate, pose stability,
        or repeated target loss.

    Notes
    -----
    This function is intentionally policy-only.

    It does not:
        - inspect a camera
        - inspect perception
        - calculate detection confidence
        - choose a controller
        - command motion

    It only combines already-known capability/health information into one
    reusable availability decision.
    """

    max_age_s = float(max_observation_age_s)

    if not math.isfinite(max_age_s) or max_age_s < 0.0:
        raise ValueError(
            "max_observation_age_s must be finite and >= 0"
        )

    enabled = bool(servoing_enabled)
    camera_ok = bool(camera_healthy)
    reliable = bool(observations_reliable)

    if observation_age_s is None:
        age_s = None
        fresh = False

    else:
        age_s = float(observation_age_s)

        fresh = (
            math.isfinite(age_s)
            and age_s >= 0.0
            and age_s <= max_age_s
        )

    if not enabled:
        reason = "servoing_disabled"

    elif not camera_ok:
        reason = "camera_unhealthy"

    elif not reliable:
        reason = "observations_unreliable"

    elif age_s is None:
        reason = "observation_unavailable"

    elif not math.isfinite(age_s) or age_s < 0.0:
        reason = "observation_age_invalid"

    elif not fresh:
        reason = "observation_stale"

    else:
        reason = None

    available = reason is None

    return VisualServoAvailability(
        available=available,
        servoing_enabled=enabled,
        camera_healthy=camera_ok,
        observations_reliable=reliable,
        observation_fresh=fresh,
        observation_age_s=age_s,
        max_observation_age_s=max_age_s,
        reason=reason,
    )


__all__ = [
    "VisualServoAvailability",
    "evaluate_visual_servo_availability",
]