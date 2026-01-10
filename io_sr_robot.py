# io_sr_robot.py
from io_canonical import *

SR_ROBOT_BOARD_NAME = "Arduino1"

sr_robot_io = {
    # Digital Inputs (Microswitches)
    DI_FRONT_BUMPER: 10,
    DI_REAR_BUMPER: 12,
    DI_LEFT_BUMPER: None,   # SR Robot doesn’t have a separate left bumper
    DI_RIGHT_BUMPER: None,

    # Analog Inputs (Ultrasonic)
    AI_ULTRASONIC_FRONT: 2,
    AI_ULTRASONIC_LEFT: 4,
    AI_ULTRASONIC_RIGHT: 6,
    AI_ULTRASONIC_REAR: 8,

    # Digital Outputs
    DO_DRIVE_LEFT_ENABLE: "MOT_LEFT_ENABLE",
    DO_DRIVE_RIGHT_ENABLE: "MOT_RIGHT_ENABLE",
    DO_GRIPPER_SOLENOID: "GRIP_SOL"
}
