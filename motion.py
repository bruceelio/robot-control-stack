# motion.py
import math
from config import drive_factor, rotate_factor, motor_polarity
from calibration import drive_duration, rotate_duration

# -----------------------------
# Helper functions
# -----------------------------

def _clamp(value, low=-1.0, high=1.0):
    """Clamp a value to the motor power range."""
    return max(low, min(high, value))

def _set_drive_power(robot, left_power, right_power):
    motor_count = len(motor_polarity)
    available = len(robot.motor_board.motors)

    if motor_count > available:
        raise RuntimeError(
            f"Config expects {motor_count} motors, "
            f"but motor board only has {available}"
        )

    for i, polarity in enumerate(motor_polarity):
        if i % 2 == 0:  # even index = left side
            robot.motor_board.motors[i].power = _clamp(left_power) * polarity
        else:           # odd index = right side
            robot.motor_board.motors[i].power = _clamp(right_power) * polarity

def stop(robot):
    for i in range(len(motor_polarity)):
        robot.motor_board.motors[i].power = 0

# -----------------------------
# Motion commands
# -----------------------------

def drive_distance(robot, distance_mm, position=(0.0, 0.0), heading=0.0):
    """
    Drive straight for a specified distance (mm) and update local position.
    Returns new position.
    """
    duration, power = drive_duration(abs(distance_mm))

    # Apply direction
    if distance_mm < 0:
        power = -power

    print(f"Driving {distance_mm} mm at power {power:.2f} for {duration * drive_factor:.2f}s")

    # Set motors
    _set_drive_power(robot, power, power)

    # Sleep for movement
    robot.sleep(duration * drive_factor)

    # Stop motors
    stop(robot)

    # Update position
    x, y = position
    dx = distance_mm * math.cos(heading)
    dy = distance_mm * math.sin(heading)
    new_position = (x + dx, y + dy)
    return new_position

def rotate_angle(robot, angle_deg, heading=0.0):
    """
    Rotate robot in place by angle (degrees) and update local heading.
    Returns new heading in radians.
    """
    duration, power = rotate_duration(abs(angle_deg))

    # Apply direction
    if angle_deg < 0:
        power = -power

    print(
        f"Rotating {angle_deg}° at power {power:.2f} for {duration * rotate_factor:.2f}s "
        f"(heading={math.degrees(heading + math.radians(angle_deg)) % 360:.1f}°)"
    )

    # Apply motor power for tank turn
    _set_drive_power(robot, power, -power)

    # Execute rotation
    robot.sleep(duration * rotate_factor)

    # Stop motors
    stop(robot)

    # Update heading locally
    new_heading = (heading + math.radians(angle_deg)) % (2 * math.pi)
    return new_heading
