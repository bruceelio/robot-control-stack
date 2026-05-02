"""Rules for routing Pi-facing IO paths to checkout behaviours."""

TEST_RULES = {
    "io.bumper": "digital_input",
    "io.limit": "digital_input",

    "io.reflectance": "analog_delta",
    "io.ultrasonic": "analog_delta",
    "io.current": "analog_delta",
    "io.voltage": "range_check",

    "io.motor": "motor_output",
    "io.servo": "servo_output",

    "io.encoder": "encoder",
    "io.imu": "imu",
    "io.otos": "otos",

    "io.camera": "external_protocol",
}

THRESHOLDS = {
    "analog_delta": 0.15,
    "ultrasonic_delta": 150.0,
    "current_delta_amps": 0.2,
    "encoder_state_changes": 3,
    "imu_degrees": 5.0,
    "otos_position": 0.05,
    "otos_heading_degrees": 5.0,
}

RANGES = {
    "io.voltage[\"battery\"].volts": (10.5, 13.5),
}

DEFAULT_SERVO_RANGE = (-0.5, 0.0, 0.5)  # min, neutral, max
DEFAULT_MOTOR_POWER = 0.3
DEFAULT_MOTOR_PULSE_SECONDS = 0.4
