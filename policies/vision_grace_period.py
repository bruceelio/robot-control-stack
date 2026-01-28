# policies/vision_grace_period.py

from dataclasses import dataclass


@dataclass(frozen=True)
class VisionGraceResult:
    lost_long_enough: bool
    age_s: float
    grace_s: float


class VisionGracePeriod:
    """
    Policy: prevent overreaction to brief vision loss.

    Input:
      - visible_now: bool
      - age_s: seconds since last observation
      - vision_grace_s: allowed loss window

    Output:
      - lost_long_enough: True iff loss exceeds grace
    """

    def __init__(self, vision_grace_s: float):
        self.vision_grace_s = float(vision_grace_s)

    def evaluate(self, *, visible_now: bool, age_s: float) -> VisionGraceResult:
        if visible_now:
            return VisionGraceResult(
                lost_long_enough=False,
                age_s=0.0,
                grace_s=self.vision_grace_s,
            )

        lost_long = age_s > self.vision_grace_s

        return VisionGraceResult(
            lost_long_enough=lost_long,
            age_s=age_s,
            grace_s=self.vision_grace_s,
        )
