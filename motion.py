# motion.py

from config import drive_factor, rotate_factor

# Assumes:
# motors[0] = left motor
# motors[1] = right motor


def _clamp(value, low=-1.0, high=1.0):
    """Clamp a value to the motor power range."""
    return max(low, min(high, value))


def stop(robot):
    """Immediately stop both drive motors."""
    robot.motor_board.motors[0].power = 0
    robot.motor_board.motors[1].power = 0

#   Alternative stop behaviour (kept for future tuning):
#
#   def stop(robot, mode="coast"):
#       if mode == "brake":
#           robot.motor_board.motors[0].brake()
#           robot.motor_board.motors[1].brake()
#       else:
#           robot.motor_board.motors[0].coast()
#           robot.motor_board.motors[1].coast()


def drive_for_time(robot, power=0.5, duration=1.0, forward=True):
    """
    Drive straight for a fixed duration (seconds), then stop.
    """
    p = _clamp(power)
    if not forward:
        p = -p

    robot.motor_board.motors[0].power = p
    robot.motor_board.motors[1].power = p

    robot.sleep(duration * drive_factor)

    stop(robot)


def rotate_for_time(robot, power=0.4, duration=0.5, clockwise=True):
    """
    Rotate in place for a fixed duration.
    Positive power magnitude, direction set by clockwise flag.
    """
    p = _clamp(power)
    if not clockwise:
        p = -p

    robot.motor_board.motors[0].power = p
    robot.motor_board.motors[1].power = -p

    robot.sleep(duration * rotate_factor)

    stop(robot)
