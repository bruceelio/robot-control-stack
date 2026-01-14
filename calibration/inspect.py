"""
Calibration inspection utilities.

This module is for HUMAN inspection only.
It must not be imported by runtime robot logic.
"""

from .base import (
    DRIVE_POWER_SHORT,
    DRIVE_POWER_LONG,
    DRIVE_SWITCH_MM,
    ROTATE_POWER,
    drive_duration,
    rotate_duration,
)


def show_drive_calibration():
    print("\n=== DRIVE CALIBRATION ===")
    print(f"Switch distance        : {DRIVE_SWITCH_MM} mm")
    print(f"Short drive power      : {DRIVE_POWER_SHORT}")
    print(f"Long drive power       : {DRIVE_POWER_LONG}")

    test_distances = [100, 250, 500, 750, 1000, 2000, 4000]
    print("\nDistance (mm) -> Duration (s), Power")
    for d in test_distances:
        duration, power = drive_duration(d)
        print(f"{d:>6} mm -> {duration:>5.2f} s @ power {power}")


def show_rotation_calibration():
    print("\n=== ROTATION CALIBRATION ===")
    print(f"Rotation power         : {ROTATE_POWER}")

    test_angles = [45, 90, 180, 270, 360]
    print("\nAngle (deg) -> Duration (s)")
    for a in test_angles:
        duration, power = rotate_duration(a)
        print(f"{a:>6} deg -> {duration:>5.2f} s @ power {power}")


def show_all():
    show_drive_calibration()
    show_rotation_calibration()


if __name__ == "__main__":
    show_all()
