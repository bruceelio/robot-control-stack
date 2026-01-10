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


def DRIVE_FOR(lvl2: Level2, distance_mm: float, heading_rad: float = 0.0, step_mm: float = 50):
    """
    Drive straight for a specified distance using lvl2.DRIVE.
    Stops immediately if front bumper pressed.
    Returns cumulative dx, dy relative to starting point.
    """
    remaining = distance_mm
    dx_total = 0.0
    dy_total = 0.0

    while abs(remaining) > 0:
        step = min(abs(remaining), step_mm) * (1 if remaining > 0 else -1)

        # calibrated duration & power
        duration, power = drive_duration(abs(step))
        if step < 0:
            power = -power

        # execute step via lvl2
        lvl2.DRIVE(power, power, duration)

        # monitor bumper during step
        start_time = time.time()
        while time.time() - start_time < duration:
            if is_front_bumper_pressed(lvl2.robot):
                lvl2._stop_motors()
                print("Front bumper pressed! Stopping DRIVE_FOR.")
                return dx_total, dy_total
            time.sleep(0.01)

        # update remaining distance
        remaining -= step

        # update dx/dy
        dx_total += step * math.cos(heading_rad)
        dy_total += step * math.sin(heading_rad)

    return dx_total, dy_total


def ROTATE_FOR(lvl2: Level2, angle_deg: float, heading_rad: float = 0.0):
    """
    Rotate in place by a specified angle (degrees) using lvl2.ROTATE.
    Returns new heading in radians.
    """
    lvl2.ROTATE(angle_deg)
    new_heading = (heading_rad + math.radians(angle_deg)) % (2 * math.pi)
    return new_heading
