class MotionBackend:
    """
    Abstract dead_reckoning backend.
    """

    def start_drive(self, *, distance_mm, localisation):
        raise NotImplementedError

    def update_drive(self, localisation):
        raise NotImplementedError

    def start_rotate(self, *, angle_deg, localisation):
        raise NotImplementedError

    def update_rotate(self, localisation):
        raise NotImplementedError
