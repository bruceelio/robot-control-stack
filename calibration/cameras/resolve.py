# calibration/cameras/resolve.py

from __future__ import annotations

from importlib import import_module


def resolve_camera_calibration(name: str):
    """
    Load a camera calibration module by name.

    Example:
        resolve_camera_calibration("pi3_640_480")
    -> imports calibration.cameras.pi3_640_480
    """
    return import_module(f"calibration.cameras.{name}")