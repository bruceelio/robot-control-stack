# config/voltage_sensors/arduino_voltage_sensor_0_25v.py

"""
Stemedu voltage sensor module, nominal 0-25V input.
Used for calibrated battery voltage readings.
"""

SENSOR_TYPE = "voltage_divider"
UNITS = "volts"

INPUT_MIN_VOLTS = 0.0
INPUT_MAX_VOLTS = 25.0

# Nominal module scaling: Arduino ADC 0-5V corresponds to sensed 0-25V.
VOLTAGE_SCALE = 5.0
VOLTAGE_OFFSET = 0.0

DEFAULT_INVERT = False