# canonical/canonical.py

"""
Canonical robot I/O definitions.

This file defines the complete vocabulary of possible robot signals.
These identifiers are hardware-agnostic and stable.

IMPORTANT:
- Canonical identifiers are STRINGS, not None.
- Mapping to physical pins happens elsewhere.
"""

# ===========================
# Digital Inputs (DI)
# ===========================

DI_BUMPER_FRONT_LEFT   = "DI_BUMPER_FRONT_LEFT"
DI_BUMPER_FRONT_RIGHT  = "DI_BUMPER_FRONT_RIGHT"
DI_BUMPER_REAR_LEFT    = "DI_BUMPER_REAR_LEFT"
DI_BUMPER_REAR_RIGHT   = "DI_BUMPER_REAR_RIGHT"

DI_ARM_LIMIT_TOP       = "DI_ARM_LIMIT_TOP"
DI_ARM_LIMIT_BOTTOM    = "DI_ARM_LIMIT_BOTTOM"

DI_GRIPPER_CLOSED      = "DI_GRIPPER_CLOSED"
DI_GRIPPER_OPEN        = "DI_GRIPPER_OPEN"

DI_PROXIMITY_FRONT     = "DI_PROXIMITY_FRONT"
DI_PROXIMITY_REAR      = "DI_PROXIMITY_REAR"

DI_ESTOP               = "DI_ESTOP"

# Encoders
DI_ENCODER_LEFT_A      = "DI_ENCODER_LEFT_A"
DI_ENCODER_LEFT_B      = "DI_ENCODER_LEFT_B"
DI_ENCODER_RIGHT_A     = "DI_ENCODER_RIGHT_A"
DI_ENCODER_RIGHT_B     = "DI_ENCODER_RIGHT_B"
DI_ENCODER_ARM_A       = "DI_ENCODER_ARM_A"
DI_ENCODER_ARM_B       = "DI_ENCODER_ARM_B"
DI_ENCODER_SCISSOR_A   = "DI_ENCODER_SCISSOR_A"
DI_ENCODER_SCISSOR_B   = "DI_ENCODER_SCISSOR_B"


# ===========================
# Digital Outputs (DO)
# ===========================

DO_GRIPPER_SOLENOID    = "DO_GRIPPER_SOLENOID"
DO_ARM_MOTOR_ENABLE    = "DO_ARM_MOTOR_ENABLE"
DO_DRIVE_LEFT_ENABLE   = "DO_DRIVE_LEFT_ENABLE"
DO_DRIVE_RIGHT_ENABLE  = "DO_DRIVE_RIGHT_ENABLE"
DO_SCISSOR_LIFT_ENABLE = "DO_SCISSOR_LIFT_ENABLE"
DO_LIGHT_INDICATOR     = "DO_LIGHT_INDICATOR"
DO_HORN                = "DO_HORN"
DO_STATUS_LED          = "DO_STATUS_LED"


# ===========================
# Analog Inputs (AI)
# ===========================

AI_ARM_POSITION        = "AI_ARM_POSITION"
AI_SCISSOR_HEIGHT      = "AI_SCISSOR_HEIGHT"

AI_IR_FRONT            = "AI_IR_FRONT"
AI_IR_REAR             = "AI_IR_REAR"
AI_IR_LEFT             = "AI_IR_LEFT"
AI_IR_RIGHT            = "AI_IR_RIGHT"

AI_ULTRASONIC_FRONT    = "AI_ULTRASONIC_FRONT"
AI_ULTRASONIC_REAR     = "AI_ULTRASONIC_REAR"
AI_ULTRASONIC_LEFT     = "AI_ULTRASONIC_LEFT"
AI_ULTRASONIC_RIGHT    = "AI_ULTRASONIC_RIGHT"

AI_BATTERY_VOLTAGE     = "AI_BATTERY_VOLTAGE"
AI_MOTOR_CURRENT_LEFT  = "AI_MOTOR_CURRENT_LEFT"
AI_MOTOR_CURRENT_RIGHT = "AI_MOTOR_CURRENT_RIGHT"
AI_ARM_CURRENT         = "AI_ARM_CURRENT"
AI_SCISSOR_CURRENT     = "AI_SCISSOR_CURRENT"

AI_IMU_ACCEL_X         = "AI_IMU_ACCEL_X"
AI_IMU_ACCEL_Y         = "AI_IMU_ACCEL_Y"
AI_IMU_ACCEL_Z         = "AI_IMU_ACCEL_Z"
AI_IMU_GYRO_X          = "AI_IMU_GYRO_X"
AI_IMU_GYRO_Y          = "AI_IMU_GYRO_Y"
AI_IMU_GYRO_Z          = "AI_IMU_GYRO_Z"


# ===========================
# Analog Outputs (AO)
# ===========================

AO_DRIVE_LEFT_PWM      = "AO_DRIVE_LEFT_PWM"
AO_DRIVE_RIGHT_PWM     = "AO_DRIVE_RIGHT_PWM"
AO_ARM_MOTOR_PWM       = "AO_ARM_MOTOR_PWM"
AO_SCISSOR_LIFT_PWM    = "AO_SCISSOR_LIFT_PWM"
AO_GRIPPER_PWM         = "AO_GRIPPER_PWM"


# ===========================
# Software / Virtual Inputs (SI)
# ===========================

SI_BUMPER_FRONT        = "SI_BUMPER_FRONT"
SI_BUMPER_REAR         = "SI_BUMPER_REAR"

SI_ESTIMATED_POSITION  = "SI_ESTIMATED_POSITION"
SI_ESTIMATED_HEADING   = "SI_ESTIMATED_HEADING"

SI_TARGET_DISTANCE     = "SI_TARGET_DISTANCE"

SI_ARM_LOAD            = "SI_ARM_LOAD"
SI_SCISSOR_LOAD        = "SI_SCISSOR_LOAD"


# ===========================
# Helper functions
# ===========================

def update_virtual_bumpers(read_di):
    """
    Update SI bumper states based on physical DI bumper inputs.

    :param read_di: function that accepts a DI_* canonical string
                    and returns True/False
    """
    return {
        SI_BUMPER_FRONT: (
            read_di(DI_BUMPER_FRONT_LEFT) or
            read_di(DI_BUMPER_FRONT_RIGHT)
        ),
        SI_BUMPER_REAR: (
            read_di(DI_BUMPER_REAR_LEFT) or
            read_di(DI_BUMPER_REAR_RIGHT)
        ),
    }
