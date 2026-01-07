# robot.py
from sr.robot3 import Robot
from location import marker_location, find_location
import itertools
import math

# --- Calibration factors ---
rotate_factor = 1.0  # multiplier for rotation commands
drive_factor = 1.0   # multiplier for drive distances


# --- Setup ---
robot = Robot()
ARENA_SIZE = 6000               # Set to 6000 if marker_location expects 0->6000
MARKERS = marker_location(ARENA_SIZE)
SLEEP_TIME = 0.5
DISTANCE_SCALE = 1.327          # Adjust simulation distances to match true arena scale

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
            AC = m1.position.distance * DISTANCE_SCALE
            BC = m2.position.distance * DISTANCE_SCALE
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

def drive_to_coordinate(robot, target_x, target_y, step_distance=300, stop_distance=100):
    """
    Drive robot to (target_x, target_y) using small steps and rotation.
    Robot stops and reads markers after each step.
    """
    print(f"Driving towards ({target_x}, {target_y})...")

    while True:
        # --- Step 1: read current position ---
        visible, positions = read_markers(robot)
        if not positions:
            print("No valid positions yet, waiting...")
            robot.sleep(0.5)
            continue

        # Compute average position
        x = sum(p[0] for p in positions) / len(positions)
        y = sum(p[1] for p in positions) / len(positions)

        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)

        if distance < stop_distance:
            print(f"Reached target! Distance remaining: {distance:.1f} mm")
            robot.stop()
            break

        # --- Step 2: compute angle to target ---
        target_angle = math.degrees(math.atan2(dy, dx))  # in degrees
        print(f"Current pos: ({x:.1f}, {y:.1f}), distance={distance:.1f}, rotate to {target_angle:.1f}°")

        # --- Step 3: rotate robot to face target ---
        robot.turn_to(target_angle * rotate_factor)  # apply rotation factor
        robot.sleep(0.2)

        # Optional: verify heading with markers here if desired

        # --- Step 4: drive a short step ---
        step = min(step_distance, distance) * drive_factor  # apply drive factor
        vx = step * math.cos(math.radians(target_angle))
        vy = step * math.sin(math.radians(target_angle))

        # Normalize vx, vy for drive fraction
        max_step = max(abs(vx), abs(vy))
        fx = vx / max_step if max_step != 0 else 0
        fy = vy / max_step if max_step != 0 else 0

        robot.drive_xy(fx, fy)
        robot.sleep(1.0)  # adjust time for simulation vs real life
        robot.stop()

        # --- Step 5: pause to read markers again ---
        robot.sleep(0.2)


# --- Modes ---
MODE = "run_all"   # Change to "run_all" for full program

if MODE == "marker_test":
    print("=== Marker Test Mode ===")
    while True:
        read_markers(robot)
        robot.sleep(SLEEP_TIME)

elif MODE == "run_all":
    print("=== Full Run Mode ===")
    # Example run: read markers once at start
    read_markers(robot)

    # --- Rest of your competition code goes here ---
    # e.g., move robot, manipulate objects, etc.
    # Make sure to call read_markers(robot) whenever you want an updated reading

    robot.motor_board.motors[0].power = 0.25
    robot.motor_board.motors[1].power = 0.25

    robot.sleep(6)   # run motors for x seconds

    robot.motor_board.motors[0].power = 0
    robot.motor_board.motors[1].power = 0

    read_markers(robot)