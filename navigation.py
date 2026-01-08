# navigation.py
from motion import drive_for_time, rotate_for_time
from config import (
    drive_factor, rotate_factor,
    rotate_delay, rotate_time_90,
    drive_delay, drive_time_1000
)
import math

# --- Rotation Utilities ---
def rotate_degrees(robot, degrees, power=0.4):
    """
    Rotate robot by a given number of degrees.
    """
    duration = (abs(degrees) / 90.0) * rotate_time_90
    duration += rotate_delay
    duration *= rotate_factor

    direction = 1 if degrees >= 0 else -1
    rotate_for_time(robot, power=power * direction, duration=duration)


# --- Drive Utilities ---
def drive_distance(robot, distance_mm, power=0.5):
    """
    Drive forward a specific distance in mm.
    """
    # Calculate time to drive the distance
    duration = (distance_mm / 1000.0) * drive_time_1000
    duration += drive_delay
    duration *= drive_factor

    drive_for_time(robot, power=power, duration=duration)


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
