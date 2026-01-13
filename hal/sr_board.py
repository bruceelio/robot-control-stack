# sr_board.py
"""
Canonical-to-pin mapping for the main SR Robot Arduino board.
This maps canonical names (from canonical.py) to the Arduino pins.
"""

from .canonical import *

SR_ROBOT_BOARD_NAME = "Arduino1"

# Pin mapping for SR Robot Arduino
# Digital pins: 2-13 (0,1 reserved for serial)
# Analog pins: A0-A5
# Notes:
# - Microswitches use INPUT_PULLUP
# - Ultrasonic sensors may use trigger/echo separately
# - Outputs are digital pins

sr_robot_io = {
    # ---------------------------
    # Digital Inputs (Microswitches / bumpers)
    # ---------------------------
    DI_BUMPER_FRONT_LEFT:   10,  # Front Left Microswitch
    DI_BUMPER_FRONT_RIGHT:  11,  # Front Right Microswitch
    DI_BUMPER_REAR_LEFT:    12,  # Rear Left Microswitch
    DI_BUMPER_REAR_RIGHT:   13,  # Rear Right Microswitch

    # ---------------------------
    # Analog Inputs (Ultrasonic sensors)
    # ---------------------------
    AI_ULTRASONIC_FRONT: 2,  # Trigger pin
    AI_ULTRASONIC_LEFT:  4,  # Trigger pin
    AI_ULTRASONIC_RIGHT: 6,  # Trigger pin
    AI_ULTRASONIC_REAR:  8,  # Trigger pin

    # Optional: separate echo pins if driver needs them
    # AI_ULTRASONIC_FRONT_ECHO: 3,
    # AI_ULTRASONIC_LEFT_ECHO:  5,
    # AI_ULTRASONIC_RIGHT_ECHO: 7,
    # AI_ULTRASONIC_REAR_ECHO:  9,

    # ---------------------------
    # Digital Outputs
    # ---------------------------
    DO_DRIVE_LEFT_ENABLE:  "MOT_LEFT_ENABLE",   # replace with actual pin if defined
    DO_DRIVE_RIGHT_ENABLE: "MOT_RIGHT_ENABLE",  # replace with actual pin if defined
    DO_GRIPPER_SOLENOID:   "GRIP_SOL",          # replace with actual pin if defined

    # ---------------------------
    # Analog Inputs (Reflectance / optional)
    # ---------------------------
    # AI_REFLECTANCE_LEFT:  A0,
    # AI_REFLECTANCE_CENTER: A1,
    # AI_REFLECTANCE_RIGHT: A2,
}

# Optional: print mapping for debug
if __name__ == "__main__":
    print(f"\n=== SR Robot IO Mapping ({SR_ROBOT_BOARD_NAME}) ===")
    for name, pin in sr_robot_io.items():
        print(f"{name}: {pin}")
    print("============================================\n")
