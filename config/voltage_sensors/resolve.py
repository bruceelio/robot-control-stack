# config/voltage_sensors/resolve.py

from __future__ import annotations

from importlib import import_module


def resolve_voltage_sensor_config(name: str):
    return import_module(f"config.voltage_sensors.{name}")