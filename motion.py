# motion.py
from config import drive_factor, rotate_factor, motor_polarity
from calibration import drive_duration, rotate_duration

# -----------------------------
# Helper functions
# -----------------------------

def _clamp(value, low=-1.0, high=1.0):
    """Clamp a value to the motor power range."""
    return max(low, min(high, value))


def _set_drive_power(robot, left_power, right_power):
    """
    Apply left/right power to all drive motors,
    respecting motor polarity and motor count.
    """
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
    """Immediately stop all drive motors."""
    for i in range(len(motor_polarity)):
        robot.motor_board.motors[i].power = 0


# -----------------------------
# Motion commands
# -----------------------------

def drive_distance(robot, distance_mm):
    """
    Drive straight for a specified distance in millimeters.
    Supports forward (positive) and backward (negative) distances.
    Uses calibration curves to determine duration and power.
    """
    # Get calibrated duration and power
    duration, power = drive_duration(abs(distance_mm))

    # Apply direction
    if distance_mm < 0:
        power = -power

    print(f"Driving {distance_mm} mm at power {power:.2f} for {duration * drive_factor:.2f}s")

    # Set motors
    _set_drive_power(robot, power, power)

    # Sleep with surface adjustment
    robot.sleep(duration * drive_factor)

    # Stop motors
    stop(robot)


def rotate_angle(robot, angle_deg):
    """
    Rotate in place by a specified angle in degrees.
    Positive angle = clockwise, negative = counter-clockwise.
    Uses calibration curve to determine duration and power.
    """
    duration, power = rotate_duration(abs(angle_deg))

    # Apply direction
    if angle_deg < 0:
        power = -power

    print(f"Rotating {angle_deg}° at power {power:.2f} for {duration * rotate_factor:.2f}s")

    # Set motors (left/right opposite for rotation)
    _set_drive_power(robot, power, -power)

    # Sleep with surface adjustment
    robot.sleep(duration * rotate_factor)

    # Stop motors
    stop(robot)
