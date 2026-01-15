# localisation.py

class Localisation:
    def __init__(self, initial_position=(0.0, 0.0), initial_heading=0.0):
        self.position = initial_position
        self.heading = initial_heading
        self.valid = False

    # Dead-reckoning update (motion model)
    def apply_motion(self, dx, dy, new_heading=None):
        self.position = (
            self.position[0] + dx,
            self.position[1] + dy
        )
        if new_heading is not None:
            self.heading = new_heading
        # IMPORTANT: does NOT set valid = True

    # Absolute correction (vision, beacons, etc.)
    def set_pose(self, position, heading):
        self.position = position
        self.heading = heading
        self.valid = True

    def invalidate(self):
        self.valid = False

    def has_pose(self):
        return self.valid

    def get_pose(self):
        if not self.valid:
            return None
        return self.position, self.heading

