# navigation.py
from motion import drive_distance, rotate_angle
import math

# --- Rotation Utilities ---
def rotate_degrees(robot, degrees):
    """
    Rotate robot by a given number of degrees.
    Positive = clockwise, negative = counter-clockwise.
    Uses calibrated rotation duration and power.
    """
    rotate_angle(robot, degrees)


# --- Drive Utilities ---
def drive_step(robot, distance_mm):
    """
    Drive forward a specific distance in mm using calibrated drive duration.
    """
    drive_distance(robot, distance_mm)

# --- Navigation Logic ---
def angle_to_target(current_pos, target_pos):
    dx = target_pos[0] - current_pos[0]
    dy = target_pos[1] - current_pos[1]
    return math.degrees(math.atan2(dy, dx))

def distance_to_target(current_pos, target_pos):
    dx = target_pos[0] - current_pos[0]
    dy = target_pos[1] - current_pos[1]
    return math.hypot(dx, dy)


def drive_to_coordinate(robot, current_pos, target_pos, step_distance=200, stop_distance=150):
    """
    Incremental navigation toward target_pos using rotation + drive_distance.
    """
    while True:
        distance = distance_to_target(current_pos, target_pos)
        if distance < stop_distance:
            print("Target reached")
            break

        angle = angle_to_target(current_pos, target_pos)
        print(f"Rotating {angle:.1f}° toward target")
        rotate_degrees(robot, angle)

        # Drive a small step
        step = min(step_distance, distance)
        print(f"Driving {step:.1f} mm")
        drive_distance(robot, step)

        # --- Update position (placeholder for real sensor) ---
        dx = step * math.cos(math.radians(angle))
        dy = step * math.sin(math.radians(angle))
        current_pos = (current_pos[0] + dx, current_pos[1] + dy)

    return current_pos
