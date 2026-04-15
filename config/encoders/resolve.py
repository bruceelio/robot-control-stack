# config/encoders/resolve.py

from __future__ import annotations

from importlib import import_module


def resolve_encoder_config(name: str):
    """
    Load an encoder device config module by name.

    Example:
        resolve_encoder_config("gobilda_4bar_odometry_pod_32mm")
    """
    return import_module(f"config.encoders.{name}")