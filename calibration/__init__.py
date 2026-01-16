# calibration/__init__.py
from calibration.resolve import resolve
from config import CONFIG

CALIBRATION = resolve(config=CONFIG)

__all__ = ["CALIBRATION"]
