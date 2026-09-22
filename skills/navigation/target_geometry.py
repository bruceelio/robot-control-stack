# skills/navigation/target_geometry.py

"""
Compatibility wrapper.

Robot-relative perception geometry now lives in:
    perception.robot_geometry
"""

from perception.robot_geometry import target_from_gripper

__all__ = [
    "target_from_gripper",
]