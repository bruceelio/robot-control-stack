# config/robots.py

VALID_ROBOT_IDS = ("sim", "sr1", "sr2")
VALID_DRIVE_LAYOUTS = ("2WD", "3WD", "4WD")
VALID_WHEEL_TYPES = ("standard", "mecanum", "omni", "tracked")

ROBOT_CONFIGS = {
    "sim": {
        "motor_polarity": {
            "2WD": [1, 1],
            "3WD": [1, 1, 1],
            "4WD": [1, 1, 1, 1],
        },
        "grab_distance_mm": 120,
    },
    "sr1": {
        "motor_polarity": {
            "2WD": [1, -1],
            "3WD": [1, -1, 1],
            "4WD": [1, -1, 1, -1],
        },
        "grab_distance_mm": 80,
    },
    "sr2": {
        "motor_polarity": {
            "2WD": [-1, 1],
            "3WD": [-1, 1, -1],
            "4WD": [-1, 1, -1, 1],
        },
        "grab_distance_mm": 100,
    },
}
