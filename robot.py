# robot.py
from sr.robot3 import Robot
from motion import drive_distance, rotate_angle
from navigation import drive_to_coordinate
from location import marker_location, find_location
from calibration import drive_duration, rotate_duration
from config import distance_scale
import itertools
import math


# --- Setup ---
robot = Robot()
ARENA_SIZE = 6000               # Set to 6000 if marker_location expects 0->6000
MARKERS = marker_location(ARENA_SIZE)
SLEEP_TIME = 0.5

# --- Calibration log at startup ---
def print_calibration_summary():
    print("=== Calibration Summary ===")

    # Short drive (250 mm)
    d_short, p_short = drive_duration(250)
    print(f"Short drive (250 mm): duration={d_short:.2f}s, power={p_short:.2f}")

    # Long drive (1000 mm)
    d_long, p_long = drive_duration(1000)
    print(f"Long drive (1000 mm): duration={d_long:.2f}s, power={p_long:.2f}")

    # Rotation 90 degrees
    r90, rpwr = rotate_duration(90)
    print(f"Rotation 90°: duration={r90:.2f}s, power={rpwr:.2f}")

print_calibration_summary()


# --- Functions ---

def read_markers(robot):
    """
    Reads markers seen by the camera, prints their info,
    triangulates robot position if at least two markers visible,
    and filters positions inside the arena bounds.
    """

    def filter_inside_arena(C1, C2):
        """Return the points that lie inside the arena bounds."""
        half = ARENA_SIZE / 2  # uses the global ARENA_SIZE
        points = []
        for x, y in [C1, C2]:
            if -half <= x <= half and -half <= y <= half:
                points.append((x, y))
        return points

    markers = robot.camera.see()
    # Only keep arena markers with ID < 20
    visible = [m for m in markers if m.id < 20]

    if len(visible) == 0:
        print("No arena markers visible")
        return [], []

    print("Visible arena markers:")
    for m in visible:
        x, y = MARKERS[m.id]  # lookup marker coordinates in arena
        print(f"  ID {m.id} at location (x={x:.1f}, y={y:.1f}), distance={m.position.distance:.1f}")

    # Triangulate if at least two markers
    all_positions = []
    if len(visible) >= 2:
        for m1, m2 in itertools.combinations(visible, 2):
            A = MARKERS[m1.id]
            B = MARKERS[m2.id]
            # Apply distance scaling for simulation calibration
            AC = m1.position.distance * distance_scale
            BC = m2.position.distance * distance_scale
            try:
                C1, C2 = find_location(A, B, AC, BC)
                valid_positions = filter_inside_arena(C1, C2)
                if valid_positions:
                    print(f"Markers {m1.id}, {m2.id} -> Valid robot positions inside arena:")
                    for x, y in valid_positions:
                        print(f"  ({x:.1f}, {y:.1f})")
                        all_positions.append((x, y))
                else:
                    print(f"Markers {m1.id}, {m2.id} -> No valid positions inside arena")
            except ValueError:
                print(f"Cannot triangulate with markers {m1.id}, {m2.id}")

    return visible, all_positions


# --- Modes ---
MODE = "run_all"   # run_all, marker_test

if MODE == "marker_test":
    print("=== Marker Test Mode ===")
    for _ in range(20):  # read markers 20 times
        read_markers(robot)
        robot.sleep(SLEEP_TIME)


elif MODE == "run_all":
    print("=== Full Run Mode ===")
    # Example run: read markers once at start
    read_markers(robot)

    # --- Rest of your competition code goes here ---
    # e.g., move robot, manipulate objects, etc.
    # Make sure to call read_markers(robot) whenever you want an updated reading

    # --- Example moves using distance/angle ---
    # Drive x mm forward
    drive_distance(robot, 999)

    # Rotate y degrees clockwise
    # rotate_angle(robot, 90)

    read_markers(robot)