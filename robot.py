# robot.py
from sr.robot3 import Robot
from motion import drive_distance, rotate_angle
from navigation import seek_and_collect
from location import marker_location, find_location
from calibration import drive_duration, rotate_duration
from config import distance_scale, ARENA_SIZE, DEFAULT_COLLECT_MODE
from perception import Perception, sense
import itertools
import math

# --- Setup ---
robot = Robot()
MARKERS = marker_location(ARENA_SIZE)
SLEEP_TIME = 0.5

def inside_arena(pos):
    """
    Returns True if the position (x, y) is inside the arena bounds.
    """
    half = ARENA_SIZE / 2
    return -half <= pos[0] <= half and -half <= pos[1] <= half

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
                valid_positions = [p for p in [C1, C2] if inside_arena(p)]
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

# --- Example event routines ---
def grab_object():
    """Perform object grabbing."""
    print("[ACTION] Grabbing object...")
    # TODO: insert motor/servo commands here
    robot.sleep(1)  # simulate time to grab


def return_to_base():
    """Return robot to a base or home position."""
    print("[ACTION] Returning to base...")
    # TODO: insert navigation commands
    drive_distance(robot, 500)  # example forward move
    rotate_angle(robot, 180)    # example rotate
    robot.sleep(0.5)


# --- Main loop ---
def main_loop():
    perception = Perception()

    # Initial positioning
    position = (0.0, 0.0)
    heading = 0.0

    # drive forward
    position = drive_distance(robot, 200, position, heading)

    # rotate
    heading = rotate_angle(robot, 20, heading)

    while True:
        # Read markers and update pose
        read_markers(robot)
        pose, objects = sense(robot, perception)

        # --- Event overrides ---
        # if hasattr(robot, "bumped") and robot.bumped():
        #    print("[EVENT] Bumper hit, grabbing object...")
        #    grab_object()
        #    return_to_base()
        #    continue  # resume main loop

        # Layer 1: Reactive
        if seek_and_collect(robot, perception, kind=DEFAULT_COLLECT_MODE):
            print("[L1] Chasing acidic target (reactive)")
            robot.sleep(0.1)

        # Layer 2: Global recovery
        elif pose is not None:
            print("[L2] Pose known, reorienting")
            rotate_angle(robot, 45)
            robot.sleep(0.1)

        # Lost / search
        else:
            print("[SEARCH] No targets, no pose — rotating")
            rotate_angle(robot, 30)
            robot.sleep(0.1)


# --- Modes ---
MODE = "run_all"  # run_all, marker_test

if MODE == "marker_test":
    print("=== Marker Test Mode ===")
    for _ in range(20):
        read_markers(robot)
        robot.sleep(SLEEP_TIME)

elif MODE == "run_all":
    print("=== Full Run Mode ===")
    read_markers(robot)
    main_loop()
