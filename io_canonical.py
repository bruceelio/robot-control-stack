# io_canonical.py
# ===========================
# Full Canonical Robot I/O Table
# ===========================
# Naming conventions:
# DI = Digital Input
# DO = Digital Output
# AI = Analog Input
# AO = Analog Output
# SI = Software inferred / virtual (optional)
# ---------------------------

# ---------------------------
# Digital Inputs (DI)
# ---------------------------
DI_BUMPER_FRONT_LEFT   = None  # Front bumper switch
DI_BUMPER_FRONT_RIGHT  = None  # Rear bumper switch
DI_BUMPER_REAR_LEFT    = None  # Left bumper
DI_BUMPER_REAR_RIGHT   = None  # Right bumper
DI_ARM_LIMIT_TOP       = None  # Arm upper limit switch
DI_ARM_LIMIT_BOTTOM    = None  # Arm lower limit switch
DI_GRIPPER_CLOSED      = None  # Gripper fully closed
DI_GRIPPER_OPEN        = None  # Gripper fully open
DI_PROXIMITY_FRONT     = None  # Digital proximity sensor (front)
DI_PROXIMITY_REAR      = None  # Digital proximity sensor (rear)
DI_ESTOP               = None  # Emergency stop button

# Encoders (quadrature digital inputs)
DI_ENCODER_LEFT_A      = None
DI_ENCODER_LEFT_B      = None
DI_ENCODER_RIGHT_A     = None
DI_ENCODER_RIGHT_B     = None
DI_ENCODER_ARM_A       = None
DI_ENCODER_ARM_B       = None
DI_ENCODER_SCISSOR_A   = None
DI_ENCODER_SCISSOR_B   = None

# ---------------------------
# Digital Outputs (DO)
# ---------------------------
DO_GRIPPER_SOLENOID    = None  # Gripper actuator
DO_ARM_MOTOR_ENABLE    = None  # Enable/disable arm motor
DO_DRIVE_LEFT_ENABLE   = None  # Left drive motor enable
DO_DRIVE_RIGHT_ENABLE  = None  # Right drive motor enable
DO_SCISSOR_LIFT_ENABLE = None  # Scissor lift motor enable
DO_LIGHT_INDICATOR     = None  # LED indicator
DO_HORN                = None  # Piezo or buzzer
DO_STATUS_LED          = None  # RGB or simple status LED

# ---------------------------
# Analog Inputs (AI)
# ---------------------------
AI_ARM_POSITION        = None  # Arm potentiometer
AI_SCISSOR_HEIGHT      = None  # Scissor lift potentiometer
AI_IR_FRONT            = None  # Front IR distance sensor
AI_IR_REAR             = None  # Rear IR distance sensor
AI_IR_LEFT             = None  # Left IR distance sensor
AI_IR_RIGHT            = None  # Right IR distance sensor
AI_ULTRASONIC_FRONT    = None  # Front ultrasonic distance sensor
AI_ULTRASONIC_REAR     = None  # Rear ultrasonic distance sensor
AI_ULTRASONIC_LEFT     = None  # Left ultrasonic distance sensor
AI_ULTRASONIC_RIGHT    = None  # Right ultrasonic distance sensor
AI_BATTERY_VOLTAGE     = None  # Battery voltage
AI_MOTOR_CURRENT_LEFT  = None  # Current sensor left drive motor
AI_MOTOR_CURRENT_RIGHT = None  # Current sensor right drive motor
AI_ARM_CURRENT         = None  # Current sensor for arm motor
AI_SCISSOR_CURRENT     = None  # Current sensor for scissor lift
AI_IMU_ACCEL_X         = None  # IMU accelerometer X
AI_IMU_ACCEL_Y         = None  # IMU accelerometer Y
AI_IMU_ACCEL_Z         = None  # IMU accelerometer Z
AI_IMU_GYRO_X          = None  # IMU gyro X
AI_IMU_GYRO_Y          = None  # IMU gyro Y
AI_IMU_GYRO_Z          = None  # IMU gyro Z

# ---------------------------
# Analog Outputs (AO)
# ---------------------------
AO_DRIVE_LEFT_PWM      = None  # Left drive motor PWM
AO_DRIVE_RIGHT_PWM     = None  # Right drive motor PWM
AO_ARM_MOTOR_PWM       = None  # Arm motor PWM
AO_SCISSOR_LIFT_PWM    = None  # Scissor lift PWM
AO_GRIPPER_PWM         = None  # Optional gripper speed PWM

# ---------------------------
# Software / Virtual Inputs (SI)
# ---------------------------
SI_BUMPER_FRONT  = None  # True if front left OR front right bumper is pressed
SI_BUMPER_REAR   = None  # True if rear left OR rear right bumper is pressed
SI_ESTIMATED_POSITION  = None  # Odometry / estimated position
SI_ESTIMATED_HEADING   = None  # Orientation from encoders / IMU
SI_TARGET_DISTANCE     = None  # Distance to a target (e.g., marker)
SI_ARM_LOAD            = None  # Derived from motor current
SI_SCISSOR_LOAD        = None  # Derived from scissor current

# ---------------------------
# Helper function to update virtual bumpers
# ---------------------------
def update_virtual_bumpers(read_di):
    """
    Update SI bumpers based on physical DI bumper states.

    :param read_di: function that accepts a DI_* constant and returns True/False
    :return: dict with updated SI_BUMPER_FRONT and SI_BUMPER_REAR
    """
    return {
        SI_BUMPER_FRONT: read_di(DI_BUMPER_FRONT_LEFT) or read_di(DI_BUMPER_FRONT_RIGHT),
        SI_BUMPER_REAR:  read_di(DI_BUMPER_REAR_LEFT)  or read_di(DI_BUMPER_REAR_RIGHT),
    }