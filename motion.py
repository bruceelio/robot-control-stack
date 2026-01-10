# motion.py
import math
import time
from config import drive_factor, rotate_factor, motor_polarity
from calibration import drive_duration, rotate_duration
from sr.robot3 import INPUT, INPUT_PULLUP
from level2_canonical import Level2


# -----------------------------
# Helper functions
# -----------------------------

def init_sensors(robot, use_pullup=True):
    """
    Initialize the Arduino pins for the bump switches.
    - use_pullup: set True for real robot with pull-ups, False for simulator
    """
    pin_mode = INPUT_PULLUP if use_pullup else INPUT
    for pin in [10, 11]:  # front left and right bumpers
        robot.arduino.pins[pin].mode = pin_mode

def is_front_bumper_pressed(robot):
    """
    Returns True if either front bumper is pressed.
    Handles simulator (normal INPUT) or real robot (INPUT_PULLUP).
    """
    # check if pins are using pull-ups
    pin0_mode = getattr(robot.arduino.pins[10], 'mode', INPUT)
    pullup = pin0_mode == INPUT_PULLUP

    left = robot.arduino.pins[10].digital_read()
    right = robot.arduino.pins[11].digital_read()

    if pullup:
        # pressed = LOW when using INPUT_PULLUP
        return not left or not right
    else:
        # pressed = HIGH for simulator
        return left or right


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
