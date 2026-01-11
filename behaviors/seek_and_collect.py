# behaviors/seek_and_collect.py

from navigation import get_closest_target, drive_to_target
from motion import ROTATE_FOR


class SeekAndCollect:
    def __init__(self, tolerance_mm=50, max_drive_mm=500):
        self.tolerance_mm = tolerance_mm
        self.max_drive_mm = max_drive_mm

    def update(self, lvl2, perception, position, heading, kind="acidic"):
        target = get_closest_target(perception, kind)
        if target is None:
            return False, position, heading

        # Rotate towards target
        heading = ROTATE_FOR(lvl2, target["bearing"], heading)

        # Drive one step towards target
        position, heading = drive_to_target(
            lvl2,
            target,
            position,
            heading,
            max_drive_mm=self.max_drive_mm,
            tolerance_mm=self.tolerance_mm,
        )

        found = target["distance"] <= self.tolerance_mm
        return found, position, heading
