# localisation.py (updated with heading)
from arena_marker_coordinates import MARKER_POSITIONS  # in mm
from math import cos, sin, radians, sqrt, atan2, degrees
from myrobot import MyRobot

class Localisation:
    """
    Robot localisation using arena markers.

    Coordinate system:
        (0,0) = arena centre
        +X     = right
        +Y     = forward
    Units: mm
    """

    def __init__(self, robot: MyRobot):
        self.robot = robot

    # --------------------------------------------------------
    # Circle intersection logic
    # --------------------------------------------------------
    def circle_intersection(self, c1, r1, c2, r2):
        x0, y0 = c1
        x1, y1 = c2
        dx, dy = x1 - x0, y1 - y0
        d = sqrt(dx**2 + dy**2)
        if d > r1 + r2 or d < abs(r1 - r2) or (d == 0 and r1 == r2):
            return []
        a = (r1**2 - r2**2 + d**2) / (2 * d)
        h = sqrt(max(0, r1**2 - a**2))
        xm = x0 + a * dx / d
        ym = y0 + a * dy / d
        xs1 = xm + h * dy / d
        ys1 = ym - h * dx / d
        xs2 = xm - h * dy / d
        ys2 = ym + h * dx / d
        return [(xs1, ys1), (xs2, ys2)]

    # --------------------------------------------------------
    # Heading calculation from one marker
    # --------------------------------------------------------
    def compute_heading(self, robot_pos, marker_pos, marker_angle_deg):
        """
        Compute robot heading (degrees) based on a marker.
        """
        dx = marker_pos[0] - robot_pos[0]
        dy = marker_pos[1] - robot_pos[1]
        abs_angle = atan2(dy, dx)  # angle in radians in arena frame
        heading_rad = abs_angle - radians(marker_angle_deg)
        heading_deg = degrees(heading_rad)
        # Normalize to [-180, 180]
        while heading_deg > 180:
            heading_deg -= 360
        while heading_deg < -180:
            heading_deg += 360
        return heading_deg

    # --------------------------------------------------------
    # Estimate position + heading
    # --------------------------------------------------------
    def estimate_pose(self, front_markers, back_markers):
        """
        Returns (x, y, heading_deg) or None if no markers.
        """

        front_markers = [(m.id, m.position.distance, m.position.horizontal_angle) for m in front_markers]
        back_markers  = [(m.id, m.position.distance, m.position.horizontal_angle) for m in back_markers]

        # -------- Priority 1: one marker each camera --------
        if front_markers and back_markers:
            f_id, f_dist, f_ang = front_markers[0]
            b_id, b_dist, b_ang = back_markers[0]
            f_center = MARKER_POSITIONS[f_id]
            b_center = MARKER_POSITIONS[b_id]
            points = self.circle_intersection(f_center, f_dist, b_center, b_dist)
            if not points:
                return None
            # Use front marker angle to pick correct intersection
            f_theta = radians(f_ang)
            robot_pos = max(points, key=lambda p: cos(f_theta)*(p[0]-f_center[0]) + sin(f_theta)*(p[1]-f_center[1]))
            # Compute heading using both markers
            heading_f = self.compute_heading(robot_pos, f_center, f_ang)
            heading_b = self.compute_heading(robot_pos, b_center, b_ang) + 180  # back camera sees rear
            heading = (heading_f + heading_b) / 2
            return (*robot_pos, heading)

        # -------- Priority 2: two markers from same camera --------
        elif len(front_markers) >= 2:
            m1, m2 = front_markers[:2]
            p1 = MARKER_POSITIONS[m1[0]]
            p2 = MARKER_POSITIONS[m2[0]]
            points = self.circle_intersection(p1, m1[1], p2, m2[1])
            if not points:
                return None
            theta = radians(m1[2])
            robot_pos = max(points, key=lambda p: cos(theta)*(p[0]-p1[0]) + sin(theta)*(p[1]-p1[1]))
            heading = self.compute_heading(robot_pos, p1, m1[2])
            return (*robot_pos, heading)

        # -------- Priority 3: only one marker --------
        elif front_markers:
            m = front_markers[0]
            p = MARKER_POSITIONS[m[0]]
            robot_pos = (p[0], p[1])
            heading = None  # unknown
            return (*robot_pos, heading)

        return None

# ============================================================
# Example usage
# ============================================================
if __name__ == "__main__":
    robot = MyRobot()
    localisation = Localisation(robot)
    front_markers = robot.camera.find_all_markers()
    back_markers  = []
    pose = localisation.estimate_pose(front_markers, back_markers)
    print("Estimated robot pose (x, y, heading_deg):", pose)
