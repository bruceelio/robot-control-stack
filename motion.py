# motion.py
import math
from config import drive_factor, rotate_factor, motor_polarity
from calibration import drive_duration, rotate_duration
from sr.robot3 import INPUT
from iomap import Hardware


# -----------------------------
# Helper functions
# -----------------------------

def is_front_bumper_pressed(robot):
    # returns True if either front switch is pressed
    return robot.arduino.pins[10].digital_read() or robot.arduino.pins[11].digital_read()


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


def drive_distance(robot, distance_mm, position=(0.0, 0.0), heading=0.0, step_mm=50):
    """
    Drive straight for a specified distance in small steps.
    Stops immediately if the front bumper is pressed.

    Args:
        robot      : Robot instance
        distance_mm: distance to drive (positive = forward, negative = backward)
        position   : current (x, y) tuple
        heading    : current heading in radians
        step_mm    : distance per incremental step for calibration

    Returns:
        new_position : updated (x, y) tuple
    """

    remaining = distance_mm
    x, y = position

    while abs(remaining) > 0:
        step = min(abs(remaining), step_mm) * (1 if remaining > 0 else -1)

        # Get calibrated duration and power from your existing function
        duration, power = drive_duration(abs(step))
        if step < 0:
            power = -power

        # Start driving continuously
        _set_drive_power(robot, power, power)

        # Monitor bumper in a tight loop during this step
        start_time = robot.time()
        while robot.time() - start_time < duration:
            if is_front_bumper_pressed(robot):
                stop(robot)
                print("Front bumper pressed! Stopping drive.")
                return (x, y)
            robot.sleep(0.01)

        # Stop at the end of step
        stop(robot)

        # Update position
        dx = step * math.cos(heading)
        dy = step * math.sin(heading)
        x += dx
        y += dy
        remaining -= step

    return (x, y)


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
