# navigation/height_model.py

from collections import deque

class HeightModel:
    """
    Lightweight belief model for marker height (high vs low).

    - Updates only when close enough
    - Commits once confidence is sufficient
    - Never changes after commit
    """

    def __init__(self, window=5):
        self.samples = deque(maxlen=window)
        self._committed = False
        self._is_high = None

    def update(self, *, pitch_deg: float):
        if self._committed:
            return

        self.samples.append(pitch_deg)

    def try_commit(self, *, high_thresh, low_thresh):
        """
        Decide height if samples are decisive.
        """
        if self._committed or not self.samples:
            return False

        avg_pitch = sum(self.samples) / len(self.samples)

        if avg_pitch >= high_thresh:
            self._commit(True)
            return True

        if avg_pitch <= low_thresh:
            self._commit(False)
            return True

        return False

    def _commit(self, is_high: bool):
        self._committed = True
        self._is_high = is_high

    # -------------------
    # Query interface
    # -------------------

    def is_committed(self) -> bool:
        return self._committed

    def is_high(self) -> bool:
        return bool(self._is_high)

    def debug_state(self) -> str:
        if not self.samples:
            return "HeightModel: no samples"

        avg = sum(self.samples) / len(self.samples)
        return (
            f"HeightModel(samples={len(self.samples)}, "
            f"avg_pitch={avg:.3f}, committed={self._committed}, "
            f"is_high={self._is_high})"
        )
