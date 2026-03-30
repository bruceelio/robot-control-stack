# config/cameras/resolve.py

from __future__ import annotations

from importlib import import_module


def resolve_camera_config(name: str):
    """
    Load a camera runtime config module by name.

    Example:
        resolve_camera_config("pi3")
    -> imports config.cameras.pi3
    """
    return import_module(f"config.cameras.{name}")