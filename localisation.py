# localisation.py
from math import sqrt, atan2, degrees, radians, cos, sin
from arena_marker_coordinates import MARKER_POSITIONS  # in mm

class Localisation:
    """
    Robot localisation using arena markers.

    Coordinate system:
        (0,0) = arena centre
        +X     = right
        +Y     = forward
    Units: mm
    """

    def __init__(self, robot):
        self.robot = robot

    # --------------------------------------------------------
    # Circle intersection logic
    # --------------------------------------------------------
    @staticmethod
    def circle_intersection(c1, r1, c2, r2):
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
    @staticmethod
    def compute_heading(robot_pos, marker_pos, marker_angle_deg):
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
    def estimate_pose(self, markers):
        """
        Estimate robot pose from a list of SR markers (camera.see()).
        Returns (x_mm, y_mm, heading_deg) or None if insufficient markers.
        """
        # Convert SR marker coordinates to distance + horizontal angle
        marker_list = []
        for m in markers:
            x, y, z = m.position.x, m.position.y, m.position.z
            distance = sqrt(x**2 + y**2 + z**2) * 1000  # meters -> mm
            horizontal_angle = degrees(atan2(y, x))
            marker_list.append((m.id, distance, horizontal_angle))

        if len(marker_list) < 1:
            return None

        # Use the two closest markers if available
        marker_list.sort(key=lambda m: m[1])  # closest first
        m1 = marker_list[0]
        if len(marker_list) > 1:
            m2 = marker_list[1]
        else:
            m2 = None

        # Lookup arena coordinates
        p1 = MARKER_POSITIONS[m1[0]]
        r1 = m1[1]

        if m2:
            p2 = MARKER_POSITIONS[m2[0]]
            r2 = m2[1]
            points = self.circle_intersection(p1, r1, p2, r2)
            if not points:
                return None
            theta = radians(m1[2])
            robot_pos = max(points, key=lambda p: cos(theta)*(p[0]-p1[0]) + sin(theta)*(p[1]-p1[1]))
            heading = self.compute_heading(robot_pos, p1, m1[2])
        else:
            # Only one marker -> position along marker vector
            robot_pos = (p1[0] - r1*cos(radians(m1[2])),
                         p1[1] - r1*sin(radians(m1[2])))
            heading = None

        return (*robot_pos, heading)


# ============================================================
# Example usage (direct SR call)
# ============================================================
if __name__ == "__main__":
    from sr.robot import Robot  # official SR API
    robot = Robot()
    localisation = Localisation(robot)

    markers = robot.camera.see()  # list of Marker objects
    pose = localisation.estimate_pose(markers)
    print("Estimated robot pose (x_mm, y_mm, heading_deg):", pose)
