# localisation.py

class Localisation:
    def __init__(self, initial_position=(0.0, 0.0), initial_heading=0.0):
        self.position = initial_position
        self.heading = initial_heading

    def apply_motion(self, dx, dy, new_heading=None):
        self.position = (
            self.position[0] + dx,
            self.position[1] + dy
        )
        if new_heading is not None:
            self.heading = new_heading

    def get_pose(self):
        return self.position, self.heading
